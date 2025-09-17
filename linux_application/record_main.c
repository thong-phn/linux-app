#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <mqueue.h>
#include <time.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <string.h>
#include <signal.h>
#include <alsa/asoundlib.h>

#define AUDIO_FRAME_SAMPLES 320            // 1 frame, 20ms @ 16kHz, 16-bit, mono = 320 samples
#define AUDIO_FRAME_SIZE    (AUDIO_FRAME_SAMPLES * 2) // 640 bytes (16-bit samples)
#define FRAME_DURATION_MS   20              // 20 ms
#define WAV_HEADER_SIZE     44              // Standard WAV header size, 44 bytes
#define TTY_DEVICE          "/dev/ttyRPMSG1"
#define QUEUE_NAME          "/audio_queue"
#define QUEUE_SIZE          10              // 200ms of audio
#define DEFAULT_PCM_DEVICE  "hw:0,0"       // Default PCM device

typedef struct {
    char data[AUDIO_FRAME_SIZE];        // Buffer capacity. Note: Char is more flexible than uint16_t
    size_t size;                        // Actual data. Note: Actual data may smaller than capacity
    int frame_number;                   // For debugging/tracking
} audio_frame_t;

typedef struct {
    mqd_t queue;                        // Message queue descriptor
    int file_fd;
    int tty_fd;
    volatile int should_stop;
    snd_pcm_t *pcm_handle;              // ALSA PCM handle
} shared_queue_t;

// Global flag for signal handler
static volatile int g_should_stop = 0;

void sigint_handler(int sig) {
    printf("\n[L] Ctrl+C detected. Stopping...\n");
    g_should_stop = 1;
}

long long get_current_time_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long) (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

void* producer_thread(void *arg) {
    shared_queue_t *data = (shared_queue_t *)arg;       // Shared queue
    audio_frame_t frame;                                // Audio frame
    snd_pcm_sframes_t frames_read;
    int frame_number = 0;
    int ret;

    printf("[L] Producer:  Producer thread started\n");

    // Recording loop
    while(!data->should_stop) {
        // Read 1 audio frame using ALSA
        frames_read = snd_pcm_readi(data->pcm_handle, frame.data, AUDIO_FRAME_SAMPLES);

        if (frames_read < 0) {
            if (frames_read == -EPIPE) {
                fprintf(stderr, "[L] Producer:  Overrun occurred\n");
                snd_pcm_prepare(data->pcm_handle);
                continue;
            }
            // When snd_pcm_abort is called, readi returns -ESTRPIPE
            if (frames_read == -ESTRPIPE) {
                printf("[L] Producer: PCM stream aborted, stopping.\n");
                break;
            }
            fprintf(stderr, "[L] Producer:  Error from snd_pcm_readi: %s\n", snd_strerror(frames_read));
            break;
        }

        if (frames_read != AUDIO_FRAME_SAMPLES) {
            fprintf(stderr, "[L] Producer:  Short read, expected %d frames, got %ld\n", AUDIO_FRAME_SAMPLES, frames_read);
        }

        frame.size = (size_t)(frames_read * 2); // 2 bytes per sample (S16_LE)
        frame.frame_number = ++frame_number;

        // Send to shared queue
        ret = mq_send(data->queue, (const char *)&frame, sizeof(frame), 0);
        if (ret < 0) {
            // If queue is full, drop frame and continue.
            if (errno == EAGAIN) {
                printf("[L] Producer:  Queue is full, dropping frame.\n");
                continue;
            }
            perror("[L] Producer:  mq_send failed");
            break;
        }
    }

    // Send EOF frame to signal consumer to stop
    printf("[L] Producer:  Sending EOF to consumer\n");
    frame.size = 0;
    frame.frame_number = -1;
    ret = mq_send(data->queue, (const char *)&frame, sizeof(frame), 0);
    if (ret < 0) {
        perror("[L] Producer:  mq_send EOF failed");
    }

    printf("[L] Producer:  Producer stopping\n");
    return NULL;
}

void* consumer_thread(void *arg) {
    shared_queue_t *data = (shared_queue_t *)arg;       // Shared queue
    audio_frame_t frame;                                // Audio frame
    ssize_t msg_size, bytes_sent;
    long long current_time, next_send_time, sleep_duration;
    int eof_received = 0;
    struct timespec timeout;
    
    printf("[L] Consumer:  Consumer thread started\n");

    next_send_time = get_current_time_ms();

    // Sending loop
    while(!data->should_stop || !eof_received) {
        // Use mq_timedreceive to avoid blocking forever
        clock_gettime(CLOCK_REALTIME, &timeout);
        timeout.tv_sec += 1; // Set a 1-second timeout

        msg_size = mq_timedreceive(data->queue, (char *)&frame, sizeof(frame), NULL, &timeout);
        if (msg_size == -1) {
            if (errno == ETIMEDOUT) {
                // Timeout occurred, just loop again to check should_stop
                continue;
            }
            perror("[L] Consumer:  mq_receive failed");
            break;
        }

        // Check for EOF
        if (frame.frame_number == -1) {
            printf("[L] Consumer:  EOF frame received, stopping\n");
            eof_received = 1;
            break; // Exit after processing EOF
        }

        // Pacing
        current_time = get_current_time_ms();
        if (current_time < next_send_time) {
            sleep_duration = next_send_time - current_time;
            usleep(sleep_duration * 1000);
        }

        // Send frames through RPMSG
        bytes_sent = write(data->tty_fd, frame.data, frame.size);
        if (bytes_sent < 0) {
            perror("[L] Consumer:  Failed to send frame through RPMSG\n");
            break;
        }

        // Update next send_time
        next_send_time += FRAME_DURATION_MS;
    }

    // Send EOF to Zephyr only if EOF frame was received
    if (eof_received) {
        const char* eof_marker = "EOF";
        (void)write(data->tty_fd, eof_marker, strlen(eof_marker));
        printf("[L] Consumer:  EOF marker sent to Zephyr\n");
    }
    printf("[L] Consumer:  Consumer thread finished\n");
    return NULL;
}

int setup_pcm(shared_queue_t *data, const char *pcm_device) {
    snd_pcm_hw_params_t *params;
    unsigned int sample_rate = 16000;
    int dir;
    int err;

    // Open PCM device for recording
    err = snd_pcm_open(&data->pcm_handle, pcm_device, SND_PCM_STREAM_CAPTURE, 0);
    if (err < 0) {
        fprintf(stderr, "Error: Unable to open PCM device %s: %s\n", pcm_device, snd_strerror(err));
        return err;
    }

    // Allocate a hardware parameters object
    snd_pcm_hw_params_alloca(&params);

    // Fill it in with default values
    snd_pcm_hw_params_any(data->pcm_handle, params);

    // Set desired hardware parameters
    snd_pcm_hw_params_set_access(data->pcm_handle, params, SND_PCM_ACCESS_RW_INTERLEAVED);
    snd_pcm_hw_params_set_format(data->pcm_handle, params, SND_PCM_FORMAT_S16_LE);
    snd_pcm_hw_params_set_channels(data->pcm_handle, params, 1); // Mono
    snd_pcm_hw_params_set_rate_near(data->pcm_handle, params, &sample_rate, &dir);

    // Set period size to our frame size
    snd_pcm_uframes_t frames = AUDIO_FRAME_SAMPLES;
    snd_pcm_hw_params_set_period_size_near(data->pcm_handle, params, &frames, &dir);

    // Write the parameters to the driver
    err = snd_pcm_hw_params(data->pcm_handle, params);
    if (err < 0) {
        fprintf(stderr, "Error: Unable to set hw parameters: %s\n", snd_strerror(err));
        return err;
    }

    printf("[L] PCM device %s configured for 16kHz, S16_LE, Mono\n", pcm_device);
    return 0;
}

void print_usage(const char *program_name) {
    printf("Usage: %s [PCM_DEVICE]\n", program_name);
    printf("  PCM_DEVICE: ALSA PCM device name (default: %s)\n", DEFAULT_PCM_DEVICE);
    printf("Examples:\n");
    printf("  %s              # Use default device (hw:0,0)\n", program_name);
    printf("  %s hw:1,0       # Use card 1, device 0\n", program_name);
    printf("  %s default      # Use default ALSA device\n", program_name);
}

int main(int argc, char* argv[]) {
    shared_queue_t shared_data;             // Shared queue between 2 threads
    struct mq_attr queue_attr;              // Message queue attributes
    pthread_t producer_tid, consumer_tid;   // Thread IDs
    const char *pcm_device = DEFAULT_PCM_DEVICE;
    int err;

    // Parse command line arguments
    if (argc > 2) {
        print_usage(argv[0]);
        return 1;
    }
    
    if (argc == 2) {
        if (strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        }
        pcm_device = argv[1];
    }

    printf("[L] Using PCM device: %s\n", pcm_device);

    // Ensure shared_data is zero-initialized
    memset(&shared_data, 0, sizeof(shared_data));

    // Setup signal handler for Ctrl+C
    signal(SIGINT, sigint_handler);

    // Open TTY device
    shared_data.tty_fd = open(TTY_DEVICE, O_WRONLY);
    if (shared_data.tty_fd < 0) {
        printf("Error: Failed to open TTY %s\n", TTY_DEVICE);
        return 1;
    }

    // Setup ALSA PCM device
    if (setup_pcm(&shared_data, pcm_device) != 0) {
        goto cleanup;
    }

    // Create message queue
    mq_unlink(QUEUE_NAME);
    queue_attr.mq_flags = O_NONBLOCK; // Use non-blocking to prevent producer from getting stuck
    queue_attr.mq_maxmsg = QUEUE_SIZE;
    queue_attr.mq_msgsize = sizeof(audio_frame_t);
    queue_attr.mq_curmsgs = 0;

    shared_data.queue = mq_open(QUEUE_NAME, O_CREAT | O_RDWR, 0644, &queue_attr);
    if (shared_data.queue == (mqd_t)-1) {
        printf("Error: Failed to create message queue\n");
        goto cleanup_pcm;
    }

    // Create threads (start consumer first so it blocks on mq_receive)
    if (pthread_create(&consumer_tid, NULL, consumer_thread, &shared_data)) {
        printf("Error: Failed to create consumer thread\n");
        goto cleanup_queue;
    }

    if (pthread_create(&producer_tid, NULL, producer_thread, &shared_data)) {
        printf("Error: Failed to create producer thread\n");
        g_should_stop = 1; // Signal other thread to stop
        pthread_join(consumer_tid, NULL);
        goto cleanup_queue;
    }

    // Wait for Ctrl+C or for threads to finish
    while (!g_should_stop) {
        sleep(1);
    }
    shared_data.should_stop = 1;

    // Abort the PCM stream to unblock the producer thread
    if (shared_data.pcm_handle) {
        snd_pcm_abort(shared_data.pcm_handle);
    }

    pthread_join(producer_tid, NULL);
    pthread_join(consumer_tid, NULL);

cleanup_queue:
    mq_close(shared_data.queue);
    mq_unlink(QUEUE_NAME);

cleanup_pcm:
    if (shared_data.pcm_handle) {
        snd_pcm_close(shared_data.pcm_handle);
    }

cleanup:
    close(shared_data.tty_fd);
    printf("[L] Application finished.\n");
    return 0;
}
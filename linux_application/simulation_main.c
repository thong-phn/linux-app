#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <mqueue.h>
#include <time.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <string.h>

#define AUDIO_FRAME_SIZE    640            // 1 frame, 10ms = 320 bytes 
#define FRAME_DURATION_MS   20              // 10 ms
#define WAV_HEADER_SIZE     44              // Standard WAV header size, 44 bytes 
#define DEFAULT_TTY_DEVICE  "/dev/ttyRPMSG1"  // Default TTY device
#define QUEUE_NAME          "/audio_queue"
#define QUEUE_SIZE          10

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
} shared_queue_t;

long long get_current_time_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long) (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

void* producer_thread(void *arg) {
    shared_queue_t *data = (shared_queue_t *)arg;       // Shared queue
    audio_frame_t frame;                                // Audio frame
    ssize_t byte_read;
    int frame_number = 0;
    int ret;
    
    printf("[L] Producer:  Producer thread started\n");

    // Recording loop
    while(!data->should_stop) {
        // Read 1 audio frame (simulating snd_pcm_readi)
        byte_read = read(data->file_fd, frame.data, AUDIO_FRAME_SIZE);
        if (byte_read <= 0) {
            printf("[L] Producer:  End of file reached\n");
            // Send EOF frame then stop
            frame.size = 0;
            frame.frame_number = -1;
            ret = mq_send(data->queue, (const char *)&frame, sizeof(frame), 0);
            if (ret < 0) {
                perror("[L] Producer:  mq_send EOF failed");
            }
            break;
        }

        // Simulate audio capturing
        usleep(FRAME_DURATION_MS * 1000);

        frame.size = (size_t)byte_read;
        frame.frame_number = ++frame_number;

        // Send to shared queue
        ret = mq_send(data->queue, (const char *)&frame, sizeof(frame), 0);
        if (ret < 0) {
            perror("[L] Producer:  mq_send failed");
            break;
        }
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
    
    printf("[L] Consumer:  Consumer thread started\n");

    next_send_time = get_current_time_ms();

    // Sending loop
    while(!data->should_stop) {
        // mq_receive: Wait for frame from queue
        msg_size = mq_receive(data->queue, (char *)&frame, sizeof(frame), NULL);
        if (msg_size == -1) {
            perror("[L] Consumer:  mq_receive failed");
            break;
        }

        // Check for EOF
        if (frame.frame_number == -1) {
            printf("[L] Consumer:  EOF frame received, stopping\n");
            eof_received = 1;
            break; // Stop the loop
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

int main(int argc, char* argv[]) {
    shared_queue_t shared_data;             // Shared queue between 2 threads
    struct stat file_stat;                  // Wav file information
    struct mq_attr queue_attr;              // Message queue attributes
    pthread_t producer_tid, consumer_tid;   // Thread IDs
    const char* wave_filename;
    const char* tty_device;
    
    if (argc < 2) {
        printf("Usage: %s <wave_file> [tty_device]\n", argv[0]);
        printf("  wave_file: Path to the WAV file\n");
        printf("  tty_device: TTY device path (default: %s)\n", DEFAULT_TTY_DEVICE);
        return 1;
    }

    wave_filename = argv[1];
    
    // Use provided TTY device or default
    if (argc >= 3) {
        tty_device = argv[2];
    } else {
        tty_device = DEFAULT_TTY_DEVICE;
    }

    // Ensure shared_data is zero-initialized 
    memset(&shared_data, 0, sizeof(shared_data));

    // Open WAV file
    shared_data.file_fd = open(wave_filename, O_RDONLY);
    if (shared_data.file_fd < 0) {
        printf("Error: Failed to open WAV file %s\n", wave_filename);
        return 1;
    }

    // Open TTY device
    shared_data.tty_fd = open(tty_device, O_WRONLY);
    if (shared_data.tty_fd < 0) {
        printf("Error: Failed to open TTY %s\n", tty_device);
        return 1;
    }

    printf("Using TTY device: %s\n", tty_device);

    // [DEBUG] Get file info
    if (fstat(shared_data.file_fd, &file_stat) == 0) {
        printf("Expect audio frames: %ld\n", (file_stat.st_size - WAV_HEADER_SIZE)/AUDIO_FRAME_SIZE); 
    }

    // Skip audio header
    if (lseek(shared_data.file_fd, WAV_HEADER_SIZE, SEEK_SET) == -1) {
        printf("Error: Failed to skip WAV header\n");
        goto cleanup;
    }

    // Create message queue
    mq_unlink(QUEUE_NAME);
    queue_attr.mq_flags = 0;
    queue_attr.mq_maxmsg = QUEUE_SIZE;
    queue_attr.mq_msgsize = sizeof(audio_frame_t);
    queue_attr.mq_curmsgs = 0;

    shared_data.queue = mq_open(QUEUE_NAME, O_CREAT | O_RDWR, 0644, &queue_attr);
    if (shared_data.queue == (mqd_t)-1) {
        printf("Error: Failed to create message queue\n");
        goto cleanup;
    }

    // Create threads (start consumer first so it blocks on mq_receive)
    if (pthread_create(&consumer_tid, NULL, consumer_thread, &shared_data)) {
        printf("Error: Failed to create consumer thread\n");
        goto cleanup_queue;
    }

    if (pthread_create(&producer_tid, NULL, producer_thread, &shared_data)) {
        printf("Error: Failed to create producer thread\n");
        shared_data.should_stop = 1;
        pthread_join(consumer_tid, NULL);
        goto cleanup_queue;
    }

    pthread_join(producer_tid, NULL);
    pthread_join(consumer_tid, NULL);

cleanup_queue:
    mq_close(shared_data.queue);
    mq_unlink(QUEUE_NAME);

cleanup:
    close(shared_data.file_fd);
    close(shared_data.tty_fd);
    return 0;
}
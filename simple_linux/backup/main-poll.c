#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <string.h>
#include <poll.h>
#include <sys/select.h>  // For select() and fd_set

#define TTY_DEVICE "/dev/ttyRPMSG1"     // TODO: Remove hardcoded device name
#define CHUNK_SIZE 496                  // Use RPMsg data limit
#define DELAY_MS 100                    // Delay between chunks
#define WAV_HEADER_SIZE 44              // Standard WAV header size

int main(int argc, char* argv[]) {
    int tty_fd, file_fd;
    char buffer[CHUNK_SIZE];
    ssize_t bytes_read, bytes_sent;
    int total_sent = 0;
    struct stat file_stat;
    
    // Wav filename
    const char* wav_filename;
    
    if (argc > 1) {
        wav_filename = argv[1];
    } else {
        printf("[L] No filename provided, using default: %s\n", wav_filename);
    }
    
    // Open the WAV file and TTY device
    file_fd = open(wav_filename, O_RDONLY);
    tty_fd = open(TTY_DEVICE, O_RDWR);      // Read/Write for bidirectional communication
    
    if (file_fd < 0) {
        printf("[L] Error: Cannot open WAV file %s\n", wav_filename);
        return 1;
    }
    
    if (tty_fd < 0) {
        printf("[L] Error: Cannot open %s\n", TTY_DEVICE);
        close(file_fd);
        return 1;
    }
     
    // Get file size
    if (fstat(file_fd, &file_stat) == 0) {
        printf("[L] WAV file size: %ld bytes\n", file_stat.st_size);
        printf("[L] Audio data size (without header): %ld bytes\n", file_stat.st_size - WAV_HEADER_SIZE);
        printf("[L] Expected chunks: %ld\n", (file_stat.st_size - WAV_HEADER_SIZE + CHUNK_SIZE - 1) / CHUNK_SIZE);
    }
    
    // Skip WAV header - seek to audio data
    if (lseek(file_fd, WAV_HEADER_SIZE, SEEK_SET) == -1) {
        printf("[L] Error: Cannot skip WAV header\n");
        close(file_fd);
        close(tty_fd);
        return 1;
    }

    // Debug print
    printf("[L] Skipped WAV header (%d bytes) \n", WAV_HEADER_SIZE);
    printf("[L] Starting audio data transfer \n");
    
    // Send audio data in chunks
    int chunk_number = 0;
    
    struct pollfd pfd;
    pfd.fd = tty_fd;
    pfd.events = POLLIN;
    while ((bytes_read = read(file_fd, buffer, CHUNK_SIZE)) > 0) {
        chunk_number++;
        printf("[L] Audio Chunk %d: %zd bytes\n", chunk_number, bytes_read);
                
        bytes_sent = write(tty_fd, buffer, bytes_read);
        if (bytes_sent < 0) {
            printf("Error: Cannot send data\n");
            break;
        }

        total_sent += bytes_sent;
        printf("[L] Sent %zd bytes (total audio data: %d)\n", bytes_sent, total_sent);
        
        // Option1: Fixed delay between chunks
        // usleep(DELAY_MS * 1000); 
       
        // Option2: Use poll()
        printf("[L] Waiting for ready signal \n");
        
        // Reset poll structure for each iteration
        struct pollfd pfd;
        pfd.fd = tty_fd;
        pfd.events = POLLIN;
        pfd.revents = 0;  // Explicitly clear revents
        
        // Wait for ready message from the DSP
        int poll_result = poll(&pfd, 1, 4000);
        
        printf("[L] Poll result: %d\n", poll_result);  
        printf("[L] Poll revents: %d\n", pfd.revents);
        
        if (poll_result > 0 && (pfd.revents & POLLIN)) {
            char ready_buffer[7];  
            memset(ready_buffer, 0, sizeof(ready_buffer)); 

            ssize_t ready_bytes = read(tty_fd, ready_buffer, sizeof(ready_buffer));
            printf("[L] Raw bytes: ");
                for (int i = 0; i < ready_bytes; i++) {
                    printf("0x%02x ", (unsigned char)ready_buffer[i]);
                }
            printf("\n");
            
            if (ready_bytes > 0) {
                ready_buffer[ready_bytes] = '\0';
                // Debug: Print raw bytes received
                // printf("[L] Raw bytes: ");
                // for (int i = 0; i < ready_bytes; i++) {
                //     printf("0x%02x ", (unsigned char)ready_buffer[i]);
                // }
                // printf("\n");

                printf("[L] Received ready signal: '%s'\n", ready_buffer);
            } else {
                printf("[L] Warning: Read returned %zd bytes\n", ready_bytes);
                break;
            }
        } else if (poll_result == 0) {
            printf("[L] Warning: No response from DSP (timeout)\n");
            break;  // Use break instead of return to allow cleanup
        } else {
            printf("[L] Error during poll: %d\n", poll_result);
            break;
        }
        
    }
    
    // Send EOF marker to signal end of transfer
    const char* eof_marker = "EOF";
    bytes_sent = write(tty_fd, eof_marker, strlen(eof_marker));
    if (bytes_sent > 0) {
        printf("[L] EOF marker sent: %zd bytes\n", bytes_sent);
    } else {
        printf("[L] Warning: Could not send EOF marker\n");
    }
    
    // Final status
    printf("[L] Audio data transfer complete. Total: %d bytes (header excluded)\n", total_sent);
    close(file_fd);
    close(tty_fd);
    return 0;
}

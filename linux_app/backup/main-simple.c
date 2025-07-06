#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <signal.h>

#define TTY_DEVICE "/dev/ttyRPMSG1"
#define BUFFER_SIZE 10  // Read up to 9 characters at a time

// Global flag for signal handling
volatile sig_atomic_t keep_running = 1;

// Signal handler for Ctrl+C
void signal_handler(int sig) {
    printf("\n[L] Received signal %d, exiting...\n", sig);
    keep_running = 0;
}

int main(int argc, char* argv[]) {
    int tty_fd;
    char buffer[BUFFER_SIZE];
    char message_buffer[100]; // Buffer to accumulate messages
    int message_len = 0;
    ssize_t bytes_read;
    
    // Set up signal handler for graceful exit
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    printf("[L] Opening TTY device: %s\n", TTY_DEVICE);
    
    // Open the TTY device for reading (non-blocking)
    tty_fd = open(TTY_DEVICE, O_RDWR | O_NONBLOCK);
    
    if (tty_fd < 0) {
        printf("[L] Error: Cannot open %s: %s\n", TTY_DEVICE, strerror(errno));
        return 1;
    }
    
    printf("[L] Successfully opened %s (non-blocking mode)\n", TTY_DEVICE);
    printf("[L] Read buffer size: %d characters\n", BUFFER_SIZE - 1);
    printf("[L] Waiting for messages from remote core...\n");
    printf("[L] Press Ctrl+C to exit\n\n");
    
    // Initialize message buffer
    memset(message_buffer, 0, sizeof(message_buffer));
    
    // Main read loop
    while (keep_running) {
        memset(buffer, 0, BUFFER_SIZE);
        
        // Non-blocking read - reads maximum 9 characters at a time
        bytes_read = read(tty_fd, buffer, BUFFER_SIZE - 1);
        
        if (bytes_read > 0) {
            printf("[L] Read chunk (%zd bytes): '", bytes_read);
            for (int i = 0; i < bytes_read; i++) {
                if (buffer[i] == '\n') {
                    printf("\\n");
                } else if (buffer[i] == '\r') {
                    printf("\\r");
                } else if (buffer[i] >= 32 && buffer[i] <= 126) {
                    printf("%c", buffer[i]);
                } else {
                    printf("[0x%02x]", (unsigned char)buffer[i]);
                }
            }
            printf("'\n");
            
            // Add new data to message buffer
            if (message_len + bytes_read < sizeof(message_buffer) - 1) {
                memcpy(message_buffer + message_len, buffer, bytes_read);
                message_len += bytes_read;
                message_buffer[message_len] = '\0';
                
                // Check for complete message (ends with newline)
                char *newline_pos = strchr(message_buffer, '\n');
                if (newline_pos != NULL) {
                    // Found complete message
                    *newline_pos = '\0'; // Remove newline
                    
                    // Remove carriage return if present
                    int len = strlen(message_buffer);
                    if (len > 0 && message_buffer[len-1] == '\r') {
                        message_buffer[len-1] = '\0';
                    }
                    
                    printf("[L] *** COMPLETE MESSAGE: '%s' ***\n\n", message_buffer);
                    
                    // Reset for next message
                    message_len = 0;
                    message_buffer[0] = '\0';
                } else {
                    printf("[L] Partial: '%s' (need newline)\n", message_buffer);
                }
            } else {
                printf("[L] Buffer full, resetting\n");
                message_len = 0;
                message_buffer[0] = '\0';
            }
            
        } else if (bytes_read == 0) {
            usleep(100000); // 100ms
        } else {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                usleep(100000); // 100ms
            } else {
                printf("[L] Error: %s\n", strerror(errno));
                break;
            }
        }
    }
    
    printf("[L] Cleaning up...\n");
    close(tty_fd);
    printf("[L] TTY device closed\n");
    printf("[L] Program exited\n");
    
    return 0;
}

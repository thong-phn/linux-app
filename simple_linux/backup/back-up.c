#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>

#define TTY_DEVICE "/dev/ttyRPMSG1"
#define FILE_SIZE 100
#define CHUNK_SIZE 32
#define DELAY_MS 100

int main() {
    int tty_fd, file_fd;
    char buffer[CHUNK_SIZE];
    ssize_t bytes_read, bytes_sent;
    int total_sent = 0;
    
    // Open file
    file_fd = open("test_100bytes.txt", O_RDONLY);
    if (file_fd < 0) {
        printf("Error: Cannot open file\n");
        return 1;
    }
    
    // Open TTY device
    tty_fd = open(TTY_DEVICE, O_WRONLY);
    if (tty_fd < 0) {
        printf("Error: Cannot open %s\n", TTY_DEVICE);
        close(file_fd);
        return 1;
    }
    
    // Read file
    while (bytes_read = read(file_fd, buffer, CHUNK_SIZE)) {

    }


    
    bytes_read = read(file_fd, buffer, FILE_SIZE);
    if (bytes_read <= 0) {
        printf("Error: Cannot read file\n");
        goto cleanup;
    }
    
    // Send to Zephyr
    bytes_sent = write(tty_fd, buffer, bytes_read);
    if (bytes_sent < 0) {
        printf("Error: Cannot send data\n");
        goto cleanup;
    }
    
    printf("Sent %zd bytes to Zephyr\n", bytes_sent);
    
cleanup:
    close(file_fd);
    close(tty_fd);
    return 0;
}
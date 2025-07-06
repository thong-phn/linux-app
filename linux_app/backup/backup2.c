#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>

#define TTY_DEVICE "/dev/ttyRPMSG1"
#define CHUNK_SIZE 512      // Total chunk size
#define DATA_PAYLOAD 496    // Actual data payload (512 - 16 for RPMsg header)
#define DELAY_MS 100        // Reduce delay for larger chunks

int main() {
    int tty_fd, file_fd;
    char buffer[CHUNK_SIZE];
    ssize_t bytes_read, bytes_sent;
    int total_sent = 0;
    struct stat file_stat;
    
    // Replace with your WAV file name
    const char* wav_filename = "audio.wav";  // Change this to your WAV file
    
    file_fd = open(wav_filename, O_RDONLY);
    tty_fd = open(TTY_DEVICE, O_WRONLY);
    
    if (file_fd < 0) {
        printf("Error: Cannot open WAV file %s\n", wav_filename);
        return 1;
    }
    
    if (tty_fd < 0) {
        printf("Error: Cannot open %s\n", TTY_DEVICE);
        close(file_fd);
        return 1;
    }
    
    // Get file size
    if (fstat(file_fd, &file_stat) == 0) {
        printf("WAV file size: %ld bytes\n", file_stat.st_size);
        printf("Expected chunks: %ld\n", (file_stat.st_size + CHUNK_SIZE - 1) / CHUNK_SIZE);
    }
    
    // Simple loop with fixed delay
    while ((bytes_read = read(file_fd, buffer, CHUNK_SIZE)) > 0) {
        printf("Chunk: ");
        for (int i = 0; i < bytes_read; i++) {
            printf("%02x ", (unsigned char)buffer[i]);
        }
        printf("\n");
        
        bytes_sent = write(tty_fd, buffer, bytes_read);
        if (bytes_sent < 0) {
            printf("Error: Cannot send data\n");
            break;
        }
        
        total_sent += bytes_sent;
        printf("Sent %zd bytes (total: %d)\n", bytes_sent, total_sent);
        
        // Fixed delay between chunks
        usleep(DELAY_MS * 1000);  // 1 second delay
    }
    
    printf("Transfer complete. Total: %d bytes\n", total_sent);
    close(file_fd);
    close(tty_fd);
    return 0;
}

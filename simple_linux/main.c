#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <string.h>

#define TTY_DEVICE "/dev/ttyRPMSG1"
#define CHUNK_SIZE 496      // Use RPMsg data limit
#define DELAY_MS 100        // Delay between chunks
#define WAV_HEADER_SIZE 44  // Standard WAV header size

int main(int argc, char* argv[]) {
    int tty_fd, file_fd;
    char buffer[CHUNK_SIZE];
    ssize_t bytes_read, bytes_sent;
    int total_sent = 0;
    struct stat file_stat;
    
    // Replace with your WAV file name
    const char* wav_filename;
    
    if (argc > 1) {
        wav_filename = argv[1];
    } else {
        printf("[L] No filename provided, using default: %s\n", wav_filename);
    }
    
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
        printf("[L] WAV file size: %ld bytes\n", file_stat.st_size);
        printf("[L] Audio data size (without header): %ld bytes\n", file_stat.st_size - WAV_HEADER_SIZE);
        printf("[L] Expected chunks: %ld\n", (file_stat.st_size - WAV_HEADER_SIZE + CHUNK_SIZE - 1) / CHUNK_SIZE);
    }
    
    // Skip WAV header - seek to audio data
    if (lseek(file_fd, WAV_HEADER_SIZE, SEEK_SET) == -1) {
        printf("Error: Cannot skip WAV header\n");
        close(file_fd);
        close(tty_fd);
        return 1;
    }

    printf("[L] Skipped WAV header (%d bytes), starting audio data transfer \n", WAV_HEADER_SIZE);
    
    // Simple loop with fixed delay
    int chunk_number = 0;
    while ((bytes_read = read(file_fd, buffer, CHUNK_SIZE)) > 0) {
        chunk_number++;
        printf("[L] Audio Chunk %d: %zd bytes\n", chunk_number, bytes_read);
        
        // // Print first few bytes of chunk for debugging (only first chunk)
        // if (chunk_number == 1) {
        //     printf("[L] First 16 bytes of audio data: ");
        //     for (int i = 0; i < (bytes_read < 16 ? bytes_read : 16); i++) {
        //         printf("%02x ", (unsigned char)buffer[i]);
        //     }
        //     printf("\n");
        // }
        
        bytes_sent = write(tty_fd, buffer, bytes_read);
        if (bytes_sent < 0) {
            printf("Error: Cannot send data\n");
            break;
        }

        total_sent += bytes_sent;
        printf("[L] Sent %zd bytes (total audio data: %d)\n", bytes_sent, total_sent);
        
        // Fixed delay between chunks
        usleep(DELAY_MS * 1000); 
    }
    
    // Send EOF marker to signal end of transfer
    const char* eof_marker = "EOF";
    bytes_sent = write(tty_fd, eof_marker, strlen(eof_marker));
    if (bytes_sent > 0) {
        printf("[L] EOF marker sent: %zd bytes\n", bytes_sent);
    } else {
        printf("[L] Warning: Could not send EOF marker\n");
    }
    
    printf("[L] Audio data transfer complete. Total: %d bytes (header excluded)\n", total_sent);
    close(file_fd);
    close(tty_fd);
    return 0;
}
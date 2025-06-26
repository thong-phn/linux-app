#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>

#define TTY_DEVICE "/dev/ttyRPMSG1"     // Named pipe for testing
#define CHUNK_SIZE 496                  // Use RPMsg data limit
#define DELAY_MS 100                    // Delay between chunks
#define READ_BUFFER_SIZE 10 

int main(int argc, char* argv[]) {
    int tty_fd;
    char read_buffer[READ_BUFFER_SIZE];
    ssize_t bytes_read;
    
    // open tty
    tty_fd = open(TTY_DEVICE, O_RDWR | O_NONBLOCK); // Open in read/write mode, non-blocking
    if (tty_fd < 0) {
        printf("[L] Error: Cannot open %s: %s\n", TTY_DEVICE, strerror(-tty_fd));
        return 1;
    }

    // init read buffer
    memset(read_buffer, 0, sizeof(read_buffer));

    // Print initial message
    printf("[L] Successfully opened %s (non-blocking mode)\n", TTY_DEVICE);

    // read loop
    while (1) {
        int error_count = 0;
	    // copy_to_user has the format ( * to, *from, size) and returns 0 on success
	    error_count = copy_to_user(read_buffer, tty_fd, sizeof(READ_BUFFER_SIZE));

	    if (error_count==0){	// if true then have success
		    printk("Sent %d characters to the user\n", sizeof);
		    return (size_of_message=0);	// clear the position to the start and return 0
        } else {
            printk(KERN_INFO "Failed to send %d characters to the user\n", error_count);
            return -EFAULT;	// Failed -- return



        // bytes_read = read(tty_fd, read_buffer, sizeof(read_buffer) - 1);

        // if (bytes_read > 0) { // data available
        //     // add null terminator
        //     read_buffer[bytes_read] = '\0';
        //     printf("[L] Read %zd bytes: '%s'\n", bytes_read, read_buffer);
            
        //     // clear buffer
        //     memset(read_buffer, 0, sizeof(read_buffer));
        
        // } else if (bytes_read == 0) { // no data available
        //     usleep(DELAY_MS * 100); // sleep
        
        // } else {
        //     // bytes_read == -1, check errno
        //     if (errno == EAGAIN || errno == EWOULDBLOCK) {
        //         // No data available - this is NORMAL for non-blocking read
        //         usleep(DELAY_MS * 100); // Convert to microseconds
        //     } else {
        //         // Real error occurred
        //         printf("[L] Read error: %s\n", strerror(errno));
        //     }
        // }
    }

    // cleanup
    close(tty_fd);
    return 0;
}

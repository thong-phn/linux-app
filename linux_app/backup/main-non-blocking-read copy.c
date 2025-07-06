#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <string.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>

#define TTY_DEVICE "/dev/ttyRPMSG1"     // Named pipe for testing
#define CHUNK_SIZE 496                  // Use RPMsg data limit
#define DELAY_MS 100                    // Delay between chunks
#define WAV_HEADER_SIZE 44              // Standard WAV header size
#define READ_BUFFER_SIZE 4 

int openTtyRpmsg(int ttyNb)
{
    char devName[50];
    sprintf(devName, "/dev/ttyRPMSG%d", ttyNb);
    int FdRpmsg = open(devName, O_RDWR |  O_NOCTTY | O_NONBLOCK);
    if (FdRpmsg < 0) {
        printf(" Error opening ttyRPMSG%d, err=-%d\n", ttyNb, errno);
        return (errno * -1);
    }
    return FdRpmsg;
}

int ControlTtyRpmsg(int FdRpmsg, int modeRaw)
{
    struct termios tiorpmsg;

    /* get current port settings */
    tcgetattr(FdRpmsg,&tiorpmsg);
    if (modeRaw) {
        /* Terminal controls are deactivated, used to transfer raw data without control */
        memset(&tiorpmsg, 0, sizeof(tiorpmsg));
        tiorpmsg.c_cflag = (CS8 | CLOCAL | CREAD);
        tiorpmsg.c_iflag = IGNPAR;
        tiorpmsg.c_oflag = 0;
        tiorpmsg.c_lflag = 0;
        tiorpmsg.c_cc[VTIME] = 0;
        tiorpmsg.c_cc[VMIN] = 1;
        cfmakeraw(&tiorpmsg);
    } else {
        /* ECHO off, other bits unchanged */
        tiorpmsg.c_lflag &= ~ECHO;
        /*do not convert LF to CR LF */
        tiorpmsg.c_oflag &= ~ONLCR;
    }
    if (tcsetattr(FdRpmsg, TCSANOW, &tiorpmsg) < 0) {
        printf("Error %d in %s  tcsetattr", errno, __func__);
        return (errno * -1);
    }
    return 0;
}

int readTtyRpmsg(int FdRpmsg, int len, char* pData)
{
    int byte_rd, byte_avail;
    int result = 0;
    if (FdRpmsg < 0) {
        printf("CA7 : Error reading ttyRPMSG, fileDescriptor is not set\n");
        return FdRpmsg;
    }
    ioctl(FdRpmsg, FIONREAD, &byte_avail);
    if (byte_avail > 0) {
        if (byte_avail >= len) {
            byte_rd = read(FdRpmsg, pData, len);
        } else {
            byte_rd = read(FdRpmsg, pData, byte_avail);
        }
        result = byte_rd;
    } else {
        result = 0;
    }
    return result;
}

int main(int argc, char* argv[]) {
    int tty_fd;
    char read_buffer[READ_BUFFER_SIZE];
    ssize_t bytes_read;
    
    // open tty
    tty_fd = openTtyRpmsg(1); // Open ttyRPMSG1
    if (tty_fd < 0) {
        printf("[L] Error: Cannot open %s: %s\n", TTY_DEVICE, strerror(-tty_fd));
        return 1;

    // init read buffer
    memset(read_buffer, 0, sizeof(read_buffer));

    // Print initial message
    printf("[L] Successfully opened %s (non-blocking mode)\n", TTY_DEVICE);

    // read loop
    while (1) {
        bytes_read = read(tty_fd, read_buffer, sizeof(read_buffer) - 1);

        if (bytes_read > 0) { // data available
            // add null terminator
            read_buffer[bytes_read] = '\0';
            // print read 
            printf("[L] Read %zd bytes: '%s'\n", bytes_read, read_buffer);
            // clear buffer
            memset(read_buffer, 0, sizeof(read_buffer));
        } else if (bytes_read == 0) { // no data available
            usleep(DELAY_MS * 100); // sleep
        } else {
            // bytes_read == -1, check errno
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // No data available - this is NORMAL for non-blocking read
                usleep(DELAY_MS * 100); // Convert to microseconds
            } else {
                // Real error occurred
                printf("[L] Read error: %s\n", strerror(errno));
            }
        }
    }

    // cleanup
    close(tty_fd);
    return 0;
}

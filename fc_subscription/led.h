#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

enum LedColor{
    red = 23,
    yellow = 24,
    green = 25
};

struct LED{
    enum LedColor led;
    bool toggle;
};

struct Status{
    struct LED led_light;
};

int trigger(struct Status *status)
{
    // Export the desired pin by writing to /sys/class/gpio/export
    status->led_light.toggle = !(status->led_light.toggle);

    char pin[3];
    snprintf(pin, 3, "%d", status->led_light.led);

    int fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd == -1) {
        perror("Unable to open /sys/class/gpio/export");
        exit(1);
    }

    if (write(fd, pin, 2) != 2) {
        perror("Error writing to /sys/class/gpio/export");
        exit(1);
    }

    close(fd);

    // Set the pin to be an output by writing "out" to /sys/class/gpio/gpio24/direction
    
    char buffer[33];
    snprintf(buffer, 33, "/sys/class/gpio/gpio%d/direction", status->led_light.led);

    fd = open(buffer, O_WRONLY);
    if (fd == -1) {
        // perror("Unable to open %s", buffer);
        printf("Unable to open %s \n", buffer);
        exit(1);
    }

    if (write(fd, "out", 3) != 3) {
        // perror("Error writing to %s", buffer);
        printf("Error writing to %s \n", buffer);
        exit(1);
    }

    close(fd);

    char buffer2[29];
    snprintf(buffer2, 29, "/sys/class/gpio/gpio%d/value", status->led_light.led);

    fd = open(buffer2, O_WRONLY);
    if (fd == -1) {
        // perror("Unable to open %s", buffer);
        printf("Unable to open %s \n", buffer);
        exit(1);
    }

    // Toggle LED 100 ms on, 100ms off, till toggle is true
    if (status->led_light.toggle == true){
        if (write(fd, "1", 1) != 1) {
            // perror("Error writing to %s", buffer2);
            printf("Error writing to %s \n", buffer2);
            exit(1);
        }
    }
    else{
        if (write(fd, "0", 1) != 1) {
            // perror("Error writing to %s", buffer2);
            printf("Error writing to %s \n", buffer2);
            exit(1);
        }       
    }
    
    close(fd);

    // Unexport the pin by writing to /sys/class/gpio/unexport

    fd = open("/sys/class/gpio/unexport", O_WRONLY);
    if (fd == -1) {
        perror("Unable to open /sys/class/gpio/unexport");
        printf("Unable to open /sys/class/gpio/unexport \n");
        exit(1);
    }

    if (write(fd, pin, 2) != 2) {
        perror("Error writing to /sys/class/gpio/unexport");
        printf("Error writing to /sys/class/gpio/unexport \n");
        exit(1);
    }

    close(fd);
    // And exit
    return 0;
}

const int time_ms = 500;
int time_us = time_ms * 1000;

extern void startupSequence(struct Status* flag) {
    for (int i = 0; i < 5; i++) {
	flag->led_light.led = red;
        trigger(flag);
        usleep(time_us);
        trigger(flag);
        usleep(time_us);
        flag->led_light.led = yellow;
        trigger(flag);
        usleep(time_us);
        trigger(flag);
        usleep(time_us);
        flag->led_light.led = green;
        trigger(flag);
        usleep(time_us);
        trigger(flag);
        usleep(time_us);
    }
}
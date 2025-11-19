#include "hal/hx711.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define SYSFS_GPIO_DIR "/sys/class/gpio"
#define NSEC_PER_MSEC 1000000L

static int write_str(const char *path, const char *val) {
    int fd = open(path, O_WRONLY);
    if (fd < 0) {
        return -errno;
    }
    if (write(fd, val, strlen(val)) < 0) {
        int e = -errno;
        close(fd);
        return e;
    }
    close(fd);
    return 0;
}

static int gpio_export(int gpio) {
    char buf[64];
    snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/export");
    char val[16];
    snprintf(val, sizeof(val), "%d", gpio);
    int rc = write_str(buf, val);
    if (rc == -EBUSY) {
        return 0; // already exported
    }
    return rc;
}

static int gpio_unexport(int gpio) {
    char buf[64];
    snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/unexport");
    char val[16];
    snprintf(val, sizeof(val), "%d", gpio);
    return write_str(buf, val);
}

static int gpio_direction(int gpio, const char *dir) {
    char path[96];
    snprintf(path, sizeof(path), SYSFS_GPIO_DIR "/gpio%d/direction", gpio);
    return write_str(path, dir);
}

static int gpio_open_value_fd(int gpio, int is_output) {
    char path[96];
    snprintf(path, sizeof(path), SYSFS_GPIO_DIR "/gpio%d/value", gpio);
    return open(path, is_output ? O_WRONLY : O_RDONLY);
}

static int gpio_write_fd(int fd, int value) {
    const char ch = value ? '1' : '0';
    if (lseek(fd, 0, SEEK_SET) < 0) {
        return -errno;
    }
    if (write(fd, &ch, 1) != 1) {
        return -errno;
    }
    return 0;
}

static int gpio_read_fd(int fd, int *value) {
    char ch;
    if (lseek(fd, 0, SEEK_SET) < 0) {
        return -errno;
    }
    if (read(fd, &ch, 1) != 1) {
        return -errno;
    }
    *value = (ch == '1');
    return 0;
}

static void sleep_us(long us) {
    struct timespec ts;
    ts.tv_sec = us / 1000000L;
    ts.tv_nsec = (us % 1000000L) * 1000L;
    nanosleep(&ts, NULL);
}

int hx711_init(hx711_t *dev, int dout_gpio, int sck_gpio, hx711_gain_t gain) {
    if (!dev) {
        return -EINVAL;
    }
    memset(dev, 0, sizeof(*dev));
    dev->dout_gpio = dout_gpio;
    dev->sck_gpio = sck_gpio;
    dev->gain_sel = gain;
    dev->fd_dout = -1;
    dev->fd_sck = -1;
    pthread_mutex_init(&dev->lock, NULL);

    int rc = gpio_export(dout_gpio);
    if (rc && rc != -EBUSY) return rc;
    rc = gpio_export(sck_gpio);
    if (rc && rc != -EBUSY) return rc;

    if ((rc = gpio_direction(dout_gpio, "in")) < 0) return rc;
    if ((rc = gpio_direction(sck_gpio, "out")) < 0) return rc;

    dev->fd_dout = gpio_open_value_fd(dout_gpio, 0);
    dev->fd_sck = gpio_open_value_fd(sck_gpio, 1);
    if (dev->fd_dout < 0 || dev->fd_sck < 0) {
        return -EIO;
    }

    gpio_write_fd(dev->fd_sck, 0);
    return 0;
}

void hx711_close(hx711_t *dev) {
    if (!dev) return;
    if (dev->fd_dout >= 0) close(dev->fd_dout);
    if (dev->fd_sck >= 0) close(dev->fd_sck);
    gpio_unexport(dev->dout_gpio);
    gpio_unexport(dev->sck_gpio);
    pthread_mutex_destroy(&dev->lock);
}

int hx711_ready_wait(hx711_t *dev, int timeout_ms) {
    const long limit_ns = timeout_ms * NSEC_PER_MSEC;
    struct timespec start, now;
    clock_gettime(CLOCK_MONOTONIC, &start);
    int val = 1;
    while (val != 0) {
        if (gpio_read_fd(dev->fd_dout, &val) < 0) {
            return -EIO;
        }
        if (val == 0) break;
        clock_gettime(CLOCK_MONOTONIC, &now);
        long ns = (now.tv_sec - start.tv_sec) * 1000000000L + (now.tv_nsec - start.tv_nsec);
        if (ns > limit_ns) {
            return -ETIMEDOUT;
        }
        sleep_us(1000);
    }
    return 0;
}

static int32_t read_raw_one(hx711_t *dev) {
    int32_t data = 0;
    for (int i = 0; i < 24; i++) {
        gpio_write_fd(dev->fd_sck, 1);
        int bit = 0;
        gpio_read_fd(dev->fd_dout, &bit);
        data = (data << 1) | (bit & 0x1);
        gpio_write_fd(dev->fd_sck, 0);
    }
    int gain_pulses = dev->gain_sel == HX711_GAIN_128 ? 1 :
                      dev->gain_sel == HX711_GAIN_64  ? 3 : 2;
    for (int i = 0; i < gain_pulses; i++) {
        gpio_write_fd(dev->fd_sck, 1);
        gpio_write_fd(dev->fd_sck, 0);
    }
    if (data & 0x800000) {
        data |= ~0xFFFFFF; // sign-extend
    }
    return data;
}

int hx711_read_average(hx711_t *dev, int samples, int32_t *out) {
    if (!dev || !out || samples <= 0) {
        return -EINVAL;
    }
    pthread_mutex_lock(&dev->lock);
    int rc = hx711_ready_wait(dev, 1000);
    if (rc) {
        pthread_mutex_unlock(&dev->lock);
        return rc;
    }
    int64_t sum = 0;
    for (int i = 0; i < samples; i++) {
        sum += read_raw_one(dev);
        sleep_us(2000); // small settling
    }
    pthread_mutex_unlock(&dev->lock);
    *out = (int32_t)(sum / samples);
    return 0;
}

int hx711_power_down(hx711_t *dev) {
    pthread_mutex_lock(&dev->lock);
    gpio_write_fd(dev->fd_sck, 1);
    sleep_us(80);
    pthread_mutex_unlock(&dev->lock);
    return 0;
}

int hx711_power_up(hx711_t *dev) {
    pthread_mutex_lock(&dev->lock);
    gpio_write_fd(dev->fd_sck, 0);
    pthread_mutex_unlock(&dev->lock);
    sleep_us(80);
    return 0;
}

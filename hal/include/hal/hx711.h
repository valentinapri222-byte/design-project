#pragma once

#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>

// HX711 gain/channel selection values mapped to pulse count after 24-bit read.
typedef enum {
    HX711_GAIN_128 = 1, // Channel A, gain 128
    HX711_GAIN_64  = 3, // Channel A, gain 64
    HX711_GAIN_32  = 2  // Channel B, gain 32
} hx711_gain_t;

typedef struct {
    int dout_gpio;
    int sck_gpio;
    hx711_gain_t gain_sel;
    int fd_dout;
    int fd_sck;
    pthread_mutex_t lock;
} hx711_t;

int hx711_init(hx711_t *dev, int dout_gpio, int sck_gpio, hx711_gain_t gain);
void hx711_close(hx711_t *dev);
int hx711_ready_wait(hx711_t *dev, int timeout_ms);
int hx711_read_average(hx711_t *dev, int samples, int32_t *out);
int hx711_power_down(hx711_t *dev);
int hx711_power_up(hx711_t *dev);

#pragma once

#include <pthread.h>
#include <stdint.h>

#include "hal/hx711.h"

typedef struct {
    hx711_t *hx;
    double scale_factor;  // user units per count
    int32_t offset;
    pthread_mutex_t lock;
} load_cell_sensor_t;

int load_cell_init(load_cell_sensor_t *s, hx711_t *hx, double scale);
int load_cell_tare(load_cell_sensor_t *s, int samples);
int load_cell_calibrate(load_cell_sensor_t *s, double known_weight, int samples);
int load_cell_read_raw(load_cell_sensor_t *s, int samples, int32_t *out);
int load_cell_read_weight(load_cell_sensor_t *s, int samples, double *out);
void load_cell_close(load_cell_sensor_t *s);

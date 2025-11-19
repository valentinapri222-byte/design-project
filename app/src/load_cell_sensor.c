#include "load_cell_sensor.h"

#include <errno.h>
#include <string.h>

int load_cell_init(load_cell_sensor_t *s, hx711_t *hx, double scale) {
    if (!s || !hx) {
        return -EINVAL;
    }
    memset(s, 0, sizeof(*s));
    s->hx = hx;
    s->scale_factor = scale;
    pthread_mutex_init(&s->lock, NULL);
    return 0;
}

void load_cell_close(load_cell_sensor_t *s) {
    if (!s) return;
    pthread_mutex_destroy(&s->lock);
}

int load_cell_tare(load_cell_sensor_t *s, int samples) {
    if (!s) return -EINVAL;
    int32_t raw = 0;
    int rc = hx711_read_average(s->hx, samples, &raw);
    if (rc) return rc;
    pthread_mutex_lock(&s->lock);
    s->offset = raw;
    pthread_mutex_unlock(&s->lock);
    return 0;
}

int load_cell_calibrate(load_cell_sensor_t *s, double known_weight, int samples) {
    if (!s || known_weight == 0.0) {
        return -EINVAL;
    }
    int32_t raw = 0;
    int rc = hx711_read_average(s->hx, samples, &raw);
    if (rc) return rc;
    pthread_mutex_lock(&s->lock);
    s->scale_factor = known_weight / (double)(raw - s->offset);
    pthread_mutex_unlock(&s->lock);
    return 0;
}

int load_cell_read_raw(load_cell_sensor_t *s, int samples, int32_t *out) {
    if (!s || !out) return -EINVAL;
    return hx711_read_average(s->hx, samples, out);
}

int load_cell_read_weight(load_cell_sensor_t *s, int samples, double *out) {
    if (!s || !out) {
        return -EINVAL;
    }
    int32_t raw = 0;
    int rc = hx711_read_average(s->hx, samples, &raw);
    if (rc) return rc;
    pthread_mutex_lock(&s->lock);
    double val = (double)(raw - s->offset) * s->scale_factor;
    pthread_mutex_unlock(&s->lock);
    *out = val;
    return 0;
}

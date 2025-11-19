// BeagleBoard AI load-cell demo using HX711 via sysfs GPIO.
// Wire DT (DOUT) to P9_23 (GPIO49) and SCK to P9_27 (GPIO115) at 3.3V.

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "hal/hx711.h"
#include "load_cell_sensor.h"

static volatile sig_atomic_t keep_running = 1;
static void on_sigint(int _) {
    (void)_;
    keep_running = 0;
}

int main(void)
{
    signal(SIGINT, on_sigint);

    // Adjust if you wire to different lines.
    const int DOUT_GPIO = 49;   // P9_23
    const int SCK_GPIO  = 115;  // P9_27
    const double KNOWN_WEIGHT = 2.0; // kg (change to your calibration mass)

    hx711_t hx;
    load_cell_sensor_t sensor;

    if (hx711_init(&hx, DOUT_GPIO, SCK_GPIO, HX711_GAIN_128) != 0) {
        fprintf(stderr, "HX711 init failed\n");
        return EXIT_FAILURE;
    }
    if (load_cell_init(&sensor, &hx, 1.0) != 0) {
        fprintf(stderr, "Sensor init failed\n");
        hx711_close(&hx);
        return EXIT_FAILURE;
    }

    printf("Taring... remove weight and wait.\n");
    if (load_cell_tare(&sensor, 10) != 0) {
        fprintf(stderr, "Tare failed\n");
        goto out;
    }
    printf("Place %.2f kg and press Enter to calibrate.\n", KNOWN_WEIGHT);
    getchar();
    if (load_cell_calibrate(&sensor, KNOWN_WEIGHT, 10) != 0) {
        fprintf(stderr, "Calibration failed\n");
        goto out;
    }
    printf("Calibrated. Reading weights (Ctrl+C to exit)...\n");

    while (keep_running) {
        double weight = 0.0;
        if (load_cell_read_weight(&sensor, 5, &weight) == 0) {
            printf("Weight: %.3f\n", weight);
        } else {
            fprintf(stderr, "Read error\n");
        }
        usleep(500000);
    }

out:
    load_cell_close(&sensor);
    hx711_power_down(&hx);
    hx711_close(&hx);
    return EXIT_SUCCESS;
}

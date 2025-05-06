#include <stdio.h>
#include <time.h>

#include "sensirion_common.h"
#include "sensirion_i2c.h"
#include "sensirion_i2c_hal.h"
#include "sgp40_i2c.h"
#include "sht3x_i2c.h"

int main(void) {
    int16_t error = 0;

    sensirion_i2c_hal_init();
    sht3x_init(SHT31_I2C_ADDR_44);

    FILE *logfile = fopen("log.csv", "a");
    if (!logfile) {
        perror("Failed to open log file");
        return 1;
    }

    // Optional: write header only if file is new
    fprintf(logfile, "Timestamp,Temperature_C,Humidity_pct,Raw_VOC\n");

    for (int i = 0; i < 900; i++) {
        uint16_t sraw_voc = 0;
        float temperature = 0;
        float humidity = 0;

        sensirion_i2c_hal_sleep_usec(1000000);  // 1 second delay

        error = sht3x_measure_single_shot(REPEATABILITY_HIGH, false,
                                          &temperature, &humidity);
        if (error != NO_ERROR) {
            fprintf(stderr, "Error executing measure_single_shot(): %i\n", error);
            continue;
        }

        error = sgp40_measure_raw_signal(humidity, temperature, &sraw_voc);
        if (error) {
            fprintf(stderr, "Error executing sgp40_measure_raw_signal(): %i\n", error);
            continue;
        }

        // Get timestamp
        time_t now = time(NULL);
        char timestamp[32];
        strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", localtime(&now));

        // Log to file
        fprintf(logfile, "%s,%.2f,%.2f,%u\n", timestamp, temperature, humidity, sraw_voc);
        fflush(logfile);  // Ensure it's written

        // Print to terminal
        printf("[%s] Temp: %.2f °C | Hum: %.2f %% | VOC Raw: %u\n",
               timestamp, temperature, humidity, sraw_voc);
    }

    fclose(logfile);
    return 0;
}

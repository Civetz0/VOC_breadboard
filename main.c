#include <stdio.h>
#include <time.h>

#include "sensirion_common.h"
#include "sensirion_i2c.h"
#include "sensirion_i2c_hal.h"
#include "sgp40_i2c.h"
#include "sht3x_i2c.h"
#include "svm41_i2c.h"

int main(void) {
    int16_t error = 0;

    sensirion_i2c_hal_init();

    sht3x_init(SHT31_I2C_ADDR_44);

    error = svm41_device_reset();
    if (error) {
        printf("Error executing svm41_device_reset(): %i\n", error);
    }



    FILE *logfile = fopen("log.csv", "a");
    if (!logfile) {
        perror("Failed to open log file");
        return 1;
    }

    error = svm41_start_measurement();
    if (error) {
        printf("Error executing svm41_start_measurement() : %i\n",
               error);
    }

    // Write header only if file is new
    fseek(logfile, 0, SEEK_END);
    long size = ftell(logfile);
    if (size == 0) {
        fprintf(logfile, "Timestamp,Temperature_C_sht31,Humidity_pct_sht31,Raw_VOC_sgp40,Temperature_C_svm41,Humidity_pct_svm41,Raw_VOC_svm41,Raw_NOX_svm41\n");
    }
    rewind(logfile);

    while (1) {

        // sgp40 variables
        uint16_t sgp40_sraw_voc = 0;

        //sht31 variables
        uint16_t sht31_temperature_ticks = 0;
        uint16_t sht31_humidity_ticks = 0;
        float sht31_humidity = 0;
        float sht31_temperature = 0;

        float sht31_humidity_offset = 2.8; //offset to sht31 humidity in %


        //svm41 variables
        int16_t svm41_humidity_ticks;
        int16_t svm41_temperature_ticks;
        uint16_t svm41_raw_voc;
        uint16_t svm41_raw_nox;

        float svm41_humidity;
        float svm41_temperature;


        sensirion_i2c_hal_sleep_usec(1000000);  // 1-second delay

        // sht31 get temperature and humidity in ticks
        error = sht3x_measure_single_shot(REPEATABILITY_HIGH, false,
                                          &sht31_temperature_ticks, &sht31_humidity_ticks);
        if (error != NO_ERROR) {
            fprintf(stderr, "Error executing measure_single_shot(): %i\n", error);
            continue;
        }

        //Humidity compensation in ticks
        sht31_humidity_ticks = sht31_humidity_ticks + (uint16_t)((sht31_humidity_offset * 65535.0) / 100.0);

        // Convert to true values in  °C and %
        sht31_humidity = signal_humidity(sht31_humidity_ticks);
        sht31_temperature = signal_temperature(sht31_temperature_ticks);


        error = sgp40_measure_raw_signal(sht31_humidity_ticks, sht31_temperature_ticks, &sgp40_sraw_voc);
        if (error) {
            fprintf(stderr, "Error executing sgp40_measure_raw_signal(): %i\n", error);
            continue;
        }

        //Get svm41 raw data
        error = svm41_read_measured_raw_values(&svm41_humidity_ticks, &svm41_temperature_ticks, &svm41_raw_voc,
                                               &svm41_raw_nox);
        if (error) {
            printf("Error executing svm41_read_measured_raw_values(): "
                   "%i\n",
                   error);
        }
        //Convert temperature and humidity to correct units
        svm41_humidity = svm41_humidity_ticks / 100.0f;
        svm41_temperature = svm41_temperature_ticks / 200.0f;



        // Get timestamp
        time_t now = time(NULL);
        char timestamp[32];
        strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", localtime(&now));

        // Log to file
        fprintf(logfile, "%s,%.2f,%.2f,%u,%.2f,%.2f,%u,%u\n", timestamp, sht31_temperature, sht31_humidity,
                sgp40_sraw_voc, svm41_temperature, svm41_humidity, svm41_raw_voc, svm41_raw_nox);
        fflush(logfile);  // Ensure it's written

        // Print to terminal
        printf("[SHT31 + SGP40] Temp: %.2f °C | Hum: %.2f %% | VOC Raw: %u\n", sht31_temperature, sht31_humidity, sgp40_sraw_voc);
        printf("[SVM41]         Temp: %.2f °C | Hum: %.2f %% | VOC Raw: %u | NOX Raw: %u\n", svm41_temperature, svm41_humidity, svm41_raw_voc, svm41_raw_nox);
        printf("Timestamp:     %s\n\n", timestamp);
}

    fclose(logfile);
    return 0;
}

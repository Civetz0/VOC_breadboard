#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <sys/stat.h>
#include <sys/types.h>


#include "sensirion_common.h"
#include "sensirion_i2c.h"
#include "sensirion_i2c_hal.h"
#include "sgp40_i2c.h"
#include "sht3x_i2c.h"
#include "svm41_i2c.h"


#define LOG_DIR "../logs"
#define CONFIG_FILE "../config.txt"


void get_timestamp(char* buffer, size_t size) {
    time_t now = time(NULL);
    strftime(buffer, size, "%Y-%m-%d_%H:%M:%S", localtime(&now));
}

// Function to read config file and extract parameters
int read_config(int* oversample_count, float* humidity_offset) {
    FILE* config_file = fopen(CONFIG_FILE, "r");
    if (config_file == NULL) {
        fprintf(stderr, "Config file not found, using default values.\n");
        return -1; // Return error if file not found
    }

    char line[128];
    while (fgets(line, sizeof(line), config_file)) {
        char key[64], value[64];

        // Skip empty lines and comments
        if (line[0] == '\0' || line[0] == '#') continue;

        // Parse key-value pairs
        if (sscanf(line, "%63s = %63s", key, value) == 2) {
            if (strcmp(key, "oversample_count") == 0) {
                *oversample_count = atoi(value);
                if (*oversample_count <= 0) {
                    fprintf(stderr, "Invalid oversample_count in config, using default value of 5.\n");
                    *oversample_count = 5;
                }
            } else if (strcmp(key, "humidity_offset") == 0) {
                *humidity_offset = atof(value);
                if (*humidity_offset < 0) {
                    fprintf(stderr, "Invalid humidity_offset in config, using default value of 2.8.\n");
                    *humidity_offset = 2.8;
                }
            }
        }
    }

    fclose(config_file);
    return 0; // Success
}


int main(int argc, char* argv[]) {


    char filename[128];
    time_t now = time(NULL);
    struct tm* t = localtime(&now);

    // Format timestamp
    char timestamp[32];
    strftime(timestamp, sizeof(timestamp), "%Y-%m-%d_%H-%M-%S", t);

    if (argc >= 2) {
        snprintf(filename, sizeof(filename), "%s/%s_%s.csv", LOG_DIR, argv[1], timestamp);
    } else {
        snprintf(filename, sizeof(filename), "%s/log_%s.csv", LOG_DIR, timestamp);
    }

    int oversample_count = 5;  // Default to 5 samples (5 seconds)
    float sht31_humidity_offset = 2.8; // Default humidity offset

    // Read configuration from file
    if (read_config(&oversample_count, &sht31_humidity_offset) != 0) {
        printf("Using default configuration: oversample_count = %d, humidity_offset = %.2f\n", oversample_count, sht31_humidity_offset);
    } else {
        printf("Loaded configuration: oversample_count = %d, humidity_offset = %.2f\n", oversample_count, sht31_humidity_offset);
    }

    int16_t error = 0;

    sensirion_i2c_hal_init();

    sht3x_init(SHT31_I2C_ADDR_44);

    error = svm41_device_reset();
    if (error) {
        printf("Error executing svm41_device_reset(): %i\n", error);
    }

    mkdir(LOG_DIR, 0755);  // Safe: will do nothing if already exists


    FILE *logfile = fopen(filename, "a");
    if (!logfile) {
        perror("Failed to open log file");
        return 1;
    }

    printf("Logging to file: %s\n", filename);


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

    // Accumulators for averaging
    float sht31_temp_sum = 0, sht31_hum_sum = 0;
    float svm41_temp_sum = 0, svm41_hum_sum = 0;
    uint32_t sgp40_voc_sum = 0, svm41_voc_sum = 0, svm41_nox_sum = 0;
    int sample_counter = 0;

    while (1) {

        sensirion_i2c_hal_sleep_usec(1000000);  // 1-second delay

        //------------------------------------SHT31----------------------------------
        //sht31 variables
        uint16_t sht31_temperature_ticks = 0;
        uint16_t sht31_humidity_ticks = 0;
        float sht31_humidity = 0;
        float sht31_temperature = 0;

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
        //----------------------------------SGP40------------------------------------------
        // sgp40 variables
        uint16_t sgp40_raw_voc = 0;

        //----Read sgp40----
        error = sgp40_measure_raw_signal(sht31_humidity_ticks, sht31_temperature_ticks, &sgp40_raw_voc);
        if (error) {
            fprintf(stderr, "Error executing sgp40_measure_raw_signal(): %i\n", error);
            continue;
        }

        //-----------------------------------SVM41-----------------------------------------
        //svm41 variables
        int16_t svm41_humidity_ticks;
        int16_t svm41_temperature_ticks;
        uint16_t svm41_raw_voc;
        uint16_t svm41_raw_nox;

        //----Read svm41----
        error = svm41_read_measured_raw_values(&svm41_humidity_ticks, &svm41_temperature_ticks, &svm41_raw_voc,
                                               &svm41_raw_nox);
        if (error) {
            printf("Error executing svm41_read_measured_raw_values(): "
                   "%i\n",
                   error);
        }
        //--------------------------OVERSAMPLING----------------------------

        //Convert temperature and humidity to correct units
        float svm41_humidity = svm41_humidity_ticks / 100.0f;
        float svm41_temperature = svm41_temperature_ticks / 200.0f;

        // --- Accumulate ---
        sht31_temp_sum += sht31_temperature;
        sht31_hum_sum += sht31_humidity;
        sgp40_voc_sum += sgp40_raw_voc;

        svm41_temp_sum += svm41_temperature;
        svm41_hum_sum += svm41_humidity;
        svm41_voc_sum += svm41_raw_voc;
        svm41_nox_sum += svm41_raw_nox;

        sample_counter++;

        // --- Log every N samples ---
        if (sample_counter >= oversample_count) {
        // Get timestamp
        get_timestamp(timestamp, sizeof(timestamp));

            // Compute and store averages before printing
            float avg_sht31_temp = sht31_temp_sum / oversample_count;
            float avg_sht31_hum = sht31_hum_sum / oversample_count;
            unsigned int avg_sgp40_voc = sgp40_voc_sum / oversample_count;

            float avg_svm41_temp = svm41_temp_sum / oversample_count;
            float avg_svm41_hum = svm41_hum_sum / oversample_count;
            unsigned int avg_svm41_voc = svm41_voc_sum / oversample_count;
            unsigned int avg_svm41_nox = svm41_nox_sum / oversample_count;

            fprintf(logfile, "%s,%.2f,%.2f,%u,%.2f,%.2f,%u,%u\n",
                    timestamp,
                    avg_sht31_temp,
                    avg_sht31_hum,
                    avg_sgp40_voc,
                    avg_svm41_temp,
                    avg_svm41_hum,
                    avg_svm41_voc,
                    avg_svm41_nox);

            // Print to terminal — now using averaged values
            printf("[SHT31 + SGP40] Temp: %.2f °C | Hum: %.2f %% | VOC Raw: %u\n",
                   avg_sht31_temp, avg_sht31_hum, avg_sgp40_voc);
            printf("[SVM41]         Temp: %.2f °C | Hum: %.2f %% | VOC Raw: %u | NOX Raw: %u\n",
                   avg_svm41_temp, avg_svm41_hum, avg_svm41_voc, avg_svm41_nox);
            printf("Timestamp:     %s\n\n", timestamp);

            fflush(logfile); //Ensure it's written

            // Reset accumulators
            sht31_temp_sum = sht31_hum_sum = 0;
            sgp40_voc_sum = 0;
            svm41_temp_sum = svm41_hum_sum = 0;
            svm41_voc_sum = svm41_nox_sum = 0;
            sample_counter = 0;

        }


        }

    fclose(logfile);
    return 0;
}

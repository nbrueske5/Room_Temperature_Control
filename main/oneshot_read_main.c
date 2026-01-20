/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

const static char *TAG = "EXAMPLE";

/*---------------------------------------------------------------
        ADC General Macros
---------------------------------------------------------------*/
//ADC1 Channel 3
#define EXAMPLE_ADC1_CHAN           ADC_CHANNEL_3

#define EXAMPLE_ADC_ATTEN           ADC_ATTEN_DB_12

static int adc_raw[10];
static int voltage[10];

// Averaging
#define AVG_SAMPLES 100
#define AVG_SAMPLES_SPEED_MS 2000 // Total delay time for averaging samples
float avg_voltage = 0;


static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);

void app_main(void)
{
    //-------------ADC1 Init---------------//
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .atten = EXAMPLE_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, EXAMPLE_ADC1_CHAN, &config));

    //-------------ADC1 Calibration Init---------------//
    adc_cali_handle_t adc1_cali_chan_handle = NULL;
    bool do_calibration1 = adc_calibration_init(ADC_UNIT_1, EXAMPLE_ADC1_CHAN, EXAMPLE_ADC_ATTEN, &adc1_cali_chan_handle);

    while (1) {
        // Average out the input
        avg_voltage = 0;
        for (int i = 0; i < AVG_SAMPLES; i++) {
            ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN, &adc_raw[0]));
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan_handle, adc_raw[0], &voltage[0]));
            avg_voltage += voltage[0];
            vTaskDelay(pdMS_TO_TICKS(AVG_SAMPLES_SPEED_MS / AVG_SAMPLES));
        }
        avg_voltage = avg_voltage / AVG_SAMPLES;
        ESP_LOGI(TAG, "Average Cali voltage: %.2f mV", avg_voltage);
        ESP_LOGI(TAG, "Temp: %.2f C", (avg_voltage - 500) / 10.0); // For TMP36 sensor
    }
}

/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

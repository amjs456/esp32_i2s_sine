#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include "driver/i2s_common.h"
#include "driver/i2s_types.h"
#include "esp_err.h"
#include "hal/adc_types.h"
#include "hal/i2s_types.h"
#include "driver/i2s_std.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "soc/adc_channel.h"
#include "esp_adc/adc_oneshot.h"

#define FRAMES_PER_BUFFER 256
#define CHANNEL_COUNT 2
#define TWO_PI 6.28318530717958647692f

static const char *TAG = "i2s_sine";

static int16_t s_samples[FRAMES_PER_BUFFER * CHANNEL_COUNT];

esp_err_t init_i2s(i2s_chan_handle_t *tx_channel){
    i2s_chan_config_t channel_conf = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    channel_conf.auto_clear_before_cb = true;

    ESP_ERROR_CHECK(i2s_new_channel(&channel_conf, tx_channel, NULL));

    const i2s_std_config_t std_conf = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(48000),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = CONFIG_SINE_BCLK_GPIO,
            .ws = CONFIG_SINE_WS_GPIO,
            .dout = CONFIG_SINE_DOUT_GPIO,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .bclk_inv = false,
                .mclk_inv = false,
                .ws_inv = false
            }
        }
    };

    esp_err_t err = i2s_channel_init_std_mode(*tx_channel, &std_conf);
    if(err != ESP_OK){
        i2s_del_channel(*tx_channel);
        *tx_channel = NULL;
        ESP_LOGE(TAG, "i2s_channel_init_std_mode failed");
        return err;
    }

    err = i2s_channel_enable(*tx_channel);
    if (err != ESP_OK){
        i2s_del_channel(*tx_channel);
        *tx_channel = NULL;
        ESP_LOGE(TAG, "i2s_channel_enable failed");
        return err;
    }

    return ESP_OK;

}

static void fill_sine_buffer(float *phase, adc_oneshot_unit_handle_t adc_handle, adc_channel_t channel){
    int raw = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, channel, &raw));
    float freq_percent = raw / 4096.0f;
    float freq = 20000 * freq_percent;
    const float phase_step = TWO_PI * (float)freq / (float)CONFIG_SINE_SAMPLE_RATE;
    const float amplitude = 32767.0f * (float)CONFIG_SINE_AMPLITUDE_PERCENT / 100.0f;
    for(size_t frame = 0; frame<FRAMES_PER_BUFFER; ++frame){
        const int16_t sample = (int16_t)lrintf(sinf(*phase) * amplitude);
        s_samples[frame * CHANNEL_COUNT] = sample;
        s_samples[frame * CHANNEL_COUNT + 1] = sample;

        *phase += phase_step;
        if(*phase >= TWO_PI){
            *phase -= TWO_PI;
        }
    }
}

static esp_err_t init_adc(adc_oneshot_unit_handle_t *adc_handle, adc_channel_t channel){
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, adc_handle));
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(*adc_handle, channel, &config));
    return ESP_OK;
}

static esp_err_t write_all(i2s_chan_handle_t tx_channel, const void *data, size_t data_size){
    const uint8_t *next = (const uint8_t *)data;
    size_t remaining = data_size;

    while(remaining > 0){
        size_t bytes_written = 0;
        ESP_ERROR_CHECK(
            i2s_channel_write(tx_channel, next, remaining, &bytes_written, portMAX_DELAY)
        );
        if(bytes_written == 0){
            return ESP_ERR_INVALID_SIZE;
        }

        next += bytes_written;
        remaining -= bytes_written;
    }
    return ESP_OK;
}

void app_main(void){
    i2s_chan_handle_t tx_channel= NULL;
    ESP_ERROR_CHECK(init_i2s(&tx_channel));

    adc_oneshot_unit_handle_t adc_handle = NULL;
    adc_channel_t channel = ADC_CHANNEL_6;
    ESP_ERROR_CHECK(init_adc(&adc_handle, channel));
    float phase = 0.0f;
    while(true){
        fill_sine_buffer(&phase, adc_handle, channel);
        ESP_ERROR_CHECK(
            write_all(tx_channel, s_samples, sizeof(s_samples))
        );
    }
}

/*
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "esp_log.h"


void app_main(void)
{
    if (CONFIG_SINE_FREQUENCY * 2 >= CONFIG_SINE_SAMPLE_RATE) {
        ESP_LOGE(TAG,
                 "Tone frequency (%d Hz) must be below half the sample rate "
                 "(%d Hz)",
                 CONFIG_SINE_FREQUENCY,
                 CONFIG_SINE_SAMPLE_RATE);
        return;
    }

    i2s_chan_handle_t tx_channel = NULL;
    ESP_ERROR_CHECK(init_i2s(&tx_channel));

    ESP_LOGI(TAG,
             "Output: %d Hz sine, %d Hz sample rate, %d%% amplitude",
             CONFIG_SINE_FREQUENCY,
             CONFIG_SINE_SAMPLE_RATE,
             CONFIG_SINE_AMPLITUDE_PERCENT);
    ESP_LOGI(TAG,
             "Pins: BCLK=GPIO%d, WS/LRCLK=GPIO%d, DOUT=GPIO%d",
             CONFIG_SINE_BCLK_GPIO,
             CONFIG_SINE_WS_GPIO,
             CONFIG_SINE_DOUT_GPIO);

    float phase = 0.0f;
    while (true) {
        fill_sine_buffer(&phase);
        ESP_ERROR_CHECK(
            write_all(tx_channel, s_samples, sizeof(s_samples)));
    }
}
*/
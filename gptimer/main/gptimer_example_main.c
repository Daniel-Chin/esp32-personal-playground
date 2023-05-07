/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gptimer.h"
#include "esp_log.h"
#include "driver/dac_oneshot.h"

#include "kit.c"
#include "music.c"

#define WAVE_TABLE_N_SAMPLES 255
#define WAVE_TABLE_N_F0S 256
static const double WAVE_TABLE_MAX_F0_INDEX = WAVE_TABLE_N_F0S - 1.0001;
#define WAVE_TABLE_F0_MIN 250     // ~ C4
#define WAVE_TABLE_F0_MAX 4200    // ~ C8
static const double WAVE_TABLE_INV_D_FREQ = 1 / ((
    WAVE_TABLE_F0_MAX - WAVE_TABLE_F0_MIN
) / (double)(WAVE_TABLE_N_F0S - 1));

#define CLOCK_FREQ 1e6

static const char *PROJECT_TAG = "Music X Flute";

gptimer_handle_t synth_timer;

static uint8_t wave_table[WAVE_TABLE_N_F0S][WAVE_TABLE_N_SAMPLES];
static uint8_t wave_table_cursor = 0;
double synth_frequency;

static bool IRAM_ATTR nextAudioSample(
    gptimer_handle_t timer, 
    const gptimer_alarm_event_data_t *edata, 
    void *user_data
) {
    double index = (synth_frequency - WAVE_TABLE_F0_MIN) * WAVE_TABLE_INV_D_FREQ;
    index = (index < 0 ? 0 : index);
    index = (index > WAVE_TABLE_MAX_F0_INDEX ? WAVE_TABLE_MAX_F0_INDEX : index);
    int left = (int)floor(index);
    double w = index - left;
    // double value = (
    //     (1 - w) * wave_table[left    ][wave_table_cursor]
    //     +     w * wave_table[left + 1][wave_table_cursor]
    // );
    double value = wave_table[0][wave_table_cursor];
    dac_oneshot_output_voltage(
        (dac_oneshot_handle_t)user_data, (uint8_t)round(value)
    );
    wave_table_cursor ++;
    if (wave_table_cursor == WAVE_TABLE_N_SAMPLES) {
        wave_table_cursor = 0;
    }
    // use uint8_t overflow instead. 
    // static_assert(WAVE_TABLE_N_SAMPLES == 256);
    return false;
}

dac_oneshot_handle_t initDAC() {
    dac_oneshot_handle_t handle;
    dac_oneshot_config_t chan0_cfg = {
        .chan_id = DAC_CHAN_0,  // D25
    };
    ESP_ERROR_CHECK(dac_oneshot_new_channel(&chan0_cfg, &handle));
    return handle;
}

#define N_PARTIALS 1
#define TIMBRE_PITCH_BOTTOM 60
#define TIMBRE_LEN 5
const double TIMBRE[TIMBRE_LEN] = {
    1, 1, 1, 1, 1, 
};
double timbreAt(double freq) {
    double pitch = freq2pitch(freq);
    double index = pitch - TIMBRE_PITCH_BOTTOM;
    if (index <= 0)
        return TIMBRE[0];
    if (index >= TIMBRE_LEN - 1)
        return TIMBRE[TIMBRE_LEN - 1];
    int left = (int)floor(index);
    double w = index - left;
    return TIMBRE[left] * (1 - w) + TIMBRE[left + 1] * w;
}

double cosCached(int x) {
    x %= WAVE_TABLE_N_SAMPLES;
    static double cache[WAVE_TABLE_N_SAMPLES];
    static bool has_cache[WAVE_TABLE_N_SAMPLES];
    // weak todo: free the cache memory (1.3KB) after initWaveTable() is done. 
    if (! has_cache[x]) {
        double progress = x / (double) WAVE_TABLE_N_SAMPLES;
        cache[x] = cos(progress * M_TWOPI);
        has_cache[x] = true;
    }
    return cache[x];
}

void initWaveTable() {
    ESP_LOGI(PROJECT_TAG, "initWaveTable");
    for (int f0_i = 0; f0_i < WAVE_TABLE_N_F0S; f0_i ++) {
        if (f0_i % 8 == 0) {
            ESP_LOGI(PROJECT_TAG, "%d/%d", f0_i, WAVE_TABLE_N_F0S);
            vTaskDelay(1);  // avoid WATCHDOG
        }
        double f0 = WAVE_TABLE_F0_MIN + f0_i / WAVE_TABLE_INV_D_FREQ;
        double timbre[N_PARTIALS];
        for (int f_i = 0; f_i < N_PARTIALS; f_i ++) {
            double freq = f0 * (f_i + 1);
            timbre[f_i] = timbreAt(freq);
        }
        for (int sample_i = 0; sample_i < WAVE_TABLE_N_SAMPLES; sample_i ++) {
            double acc = 0;
            for (int f_i = 0; f_i < N_PARTIALS; f_i ++) {
                acc += timbre[f_i] * cosCached(
                    sample_i * (f_i + 1)
                );
            }
            wave_table[f0_i][sample_i] = (uint8_t)round(
                (acc + 1) * .5 * 255
            );
        }
    }
    ESP_LOGI(PROJECT_TAG, "ok");
}

void updateFrequency(double new_freq) {
    gptimer_alarm_config_t alarm_config1 = {
        .alarm_count = (int)round(CLOCK_FREQ / (new_freq * WAVE_TABLE_N_SAMPLES)),
        .flags.auto_reload_on_alarm = true, 
        .reload_count = 0, 
    };
    gptimer_set_alarm_action(synth_timer, &alarm_config1);
    synth_frequency = new_freq;
}

void initSynthTimer(dac_oneshot_handle_t dac_handle) {
    ESP_LOGI(PROJECT_TAG, "Create timer handle");
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = CLOCK_FREQ,
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &synth_timer));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = nextAudioSample,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(
        synth_timer, &cbs, dac_handle
    ));

    ESP_LOGI(PROJECT_TAG, "Enable timer");
    ESP_ERROR_CHECK(gptimer_enable(synth_timer));

    ESP_LOGI(PROJECT_TAG, "Start timer");
    updateFrequency(110);   // to set the alarm action
    ESP_ERROR_CHECK(gptimer_start(synth_timer));
}

void app_main(void)
{
    dac_oneshot_handle_t dac_handle = initDAC();
    initWaveTable();
    printArray(&(wave_table[0][0]), WAVE_TABLE_N_SAMPLES);
    printf("\n");
    initSynthTimer(dac_handle);

    ESP_LOGI(PROJECT_TAG, "ready");

    for (int p = 48; p < 72; p ++) {
        vTaskDelay(100);
        ESP_LOGI(PROJECT_TAG, "pitch=%d", p);
        updateFrequency(pitch2freq(p));
    }
}

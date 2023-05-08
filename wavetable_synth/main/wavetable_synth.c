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
#include "esp_timer.h"

#include "kit.c"
#include "music.c"

#define WAVE_TABLE_N_SAMPLES 64 // can reach 8000Hz partial, given WAVE_TABLE_F0_MIN=250
#define WAVE_TABLE_N_F0S 256
static const float WAVE_TABLE_MAX_F0_INDEX = WAVE_TABLE_N_F0S - 1.0001;
#define WAVE_TABLE_F0_MIN 250     // ~ C4
#define WAVE_TABLE_F0_MAX 4200    // ~ C8
static const float WAVE_TABLE_INV_D_FREQ = 1 / ((
    WAVE_TABLE_F0_MAX - WAVE_TABLE_F0_MIN
) / (float)(WAVE_TABLE_N_F0S - 1));

#define CLOCK_FREQ 8e6

static const char *PROJECT_TAG = "Music X Flute";

gptimer_handle_t synth_timer;

static uint8_t wave_table[WAVE_TABLE_N_F0S][WAVE_TABLE_N_SAMPLES];
static uint8_t wave_row[WAVE_TABLE_N_SAMPLES];
static uint8_t wave_row_cursor = 0;

static bool IRAM_ATTR nextAudioSample(
    gptimer_handle_t timer, 
    const gptimer_alarm_event_data_t *edata, 
    void *user_data
) {
    // ISR context
    dac_oneshot_output_voltage(
        (dac_oneshot_handle_t)user_data, 
        wave_row[wave_row_cursor]
    );
    wave_row_cursor ++;
    if (wave_row_cursor == WAVE_TABLE_N_SAMPLES) {
        wave_row_cursor = 0;
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

#define TIMBRE_PITCH_BOTTOM 60
#define TIMBRE_LEN 49
const float TIMBRE[TIMBRE_LEN] = {
    0.13676878662109376, 
    0.13676878662109376, 
    0.13676878662109376, 
    0.13676878662109376, 
    0.13676878662109376, 
    0.13676878662109376, 
    0.13676878662109376, 
    0.13676878662109376, 
    0.2327656005859375, 
    0.2327656005859375, 
    0.11663565673828126, 
    0.12271953125, 
    0.12271953125, 
    0.12271953125, 
    0.09569869384765625, 
    0.09569869384765625, 
    0.09569869384765625, 
    0.253130078125, 
    0.12710006103515625, 
    0.12710006103515625, 
    0.036081689453125, 
    0.036081689453125, 
    0.07039032592773438, 
    0.025634671020507815, 
    0.025634671020507815, 
    0.20156268310546877, 
    0.0039300498962402345, 
    0.29150974121093753, 
    0.027677261352539066, 
    0.07467901000976562, 
    0.032162060546875, 
    0.008509994506835938, 
    0.016688189697265626, 
    0.02972444763183594, 
    0.01769535675048828, 
    0.003682607650756836, 
    0.02248181304931641, 
    0.005784563827514648, 
    0.007685343170166016, 
    0.002239899444580078, 
    0, 
    0.0074455039978027346, 
    0.0015351497650146486, 
    0.0024925634384155276, 
    0.016617822265625002, 
    0.001835605239868164, 
    0.0025110095977783206, 
    0.011419316101074219, 
    0.00209945125579834, 
};
float timbreAt(float freq) {
    float pitch = freq2pitch(freq);
    float index = pitch - TIMBRE_PITCH_BOTTOM;
    if (index <= 0)
        return TIMBRE[0];
    if (index >= TIMBRE_LEN - 1)
        return 0;
    int left = (int)floorf(index);
    float w = index - left;
    return TIMBRE[left] * (1 - w) + TIMBRE[left + 1] * w;
}

float cosCached(int x) {
    x %= WAVE_TABLE_N_SAMPLES;
    static float cache[WAVE_TABLE_N_SAMPLES];
    static bool has_cache[WAVE_TABLE_N_SAMPLES];
    // weak todo: free the cache memory (1.3KB) after initWaveTable() is done. 
    if (! has_cache[x]) {
        float progress = x / (float) WAVE_TABLE_N_SAMPLES;
        cache[x] = cos(progress * M_TWOPI);
        has_cache[x] = true;
    }
    return cache[x];
}

void initWaveTable() {
    ESP_LOGI(PROJECT_TAG, "initWaveTable");
    int MAX_N_PARTIALS = WAVE_TABLE_N_SAMPLES / 2;
    float timbre_max_freq = pitch2freq(TIMBRE_PITCH_BOTTOM + TIMBRE_LEN + 1);
    // The timbre defines that any freq above this has mag 0. 
    ESP_LOGI(PROJECT_TAG, "timbre_max_freq = %f", timbre_max_freq);
    if (MAX_N_PARTIALS * WAVE_TABLE_F0_MIN < timbre_max_freq) {
        ESP_LOGW(PROJECT_TAG, "Some meaningful partials exceed Nyquist freq. ");
    }
    int next_wake = esp_timer_get_time() + 1e6;
    for (int f0_i = 0; f0_i < WAVE_TABLE_N_F0S; f0_i ++) {
        if (esp_timer_get_time() > next_wake) {
            next_wake += 1e6;
            ESP_LOGI(PROJECT_TAG, "%d/%d", f0_i, WAVE_TABLE_N_F0S);
            vTaskDelay(1);  // avoid WATCHDOG
        }
        float f0 = WAVE_TABLE_F0_MIN + f0_i / WAVE_TABLE_INV_D_FREQ;
        float timbre[MAX_N_PARTIALS];
        int n_partials = MAX_N_PARTIALS;
        for (int f_i = 0; f_i < MAX_N_PARTIALS; f_i ++) {
            float freq = f0 * (f_i + 1);
            if (freq > timbre_max_freq) {
                n_partials = f_i;
                break;
            }
            timbre[f_i] = timbreAt(freq);
        }
        for (int sample_i = 0; sample_i < WAVE_TABLE_N_SAMPLES; sample_i ++) {
            float acc = 0;
            for (int f_i = 0; f_i < n_partials; f_i ++) {
                acc += timbre[f_i] * cosCached(
                    sample_i * (f_i + 1)
                );
            }
            if (acc < -1 || acc > 1) {
                ESP_LOGE(PROJECT_TAG, "audio over-norm!");
            }
            wave_table[f0_i][sample_i] = (uint8_t)roundf(
                (acc + 1) * .5 * 255
            );
        }
    }
    ESP_LOGI(PROJECT_TAG, "ok");
}

void updateWaveRow(float freq, float amplitude) {
    gptimer_alarm_config_t alarm_config1 = {
        .alarm_count = (int)roundf(CLOCK_FREQ / (freq * WAVE_TABLE_N_SAMPLES)),
        .flags.auto_reload_on_alarm = true, 
        .reload_count = 0, 
    };
    gptimer_set_alarm_action(synth_timer, &alarm_config1);
    float index = (freq - WAVE_TABLE_F0_MIN) * WAVE_TABLE_INV_D_FREQ;
    index = (index < 0 ? 0 : index);
    index = (index > WAVE_TABLE_MAX_F0_INDEX ? WAVE_TABLE_MAX_F0_INDEX : index);
    int left = (int)floorf(index);
    float w = index - left;
    // weak todo: optim w/ SIMD (esp-dsp)
    for (int i = 0; i < WAVE_TABLE_N_SAMPLES; i ++) {
        wave_row[i] = (uint8_t) roundf(amplitude * (
            (1 - w) * wave_table[left    ][i]
            +     w * wave_table[left + 1][i]
            - 127.5
        ) + 127.5);
        // there is also race condition w/ ISR, but it's fine
    }
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
    updateWaveRow(110, 1);   // to set the alarm action
    ESP_ERROR_CHECK(gptimer_start(synth_timer));
}

void app_main(void)
{
    dac_oneshot_handle_t dac_handle = initDAC();
    initWaveTable();
    // ESP_LOGI(PROJECT_TAG, "Example waveform:");
    // printArray(&(wave_table[0][0]), WAVE_TABLE_N_SAMPLES);
    // printf("\n");

    initSynthTimer(dac_handle);

    ESP_LOGI(PROJECT_TAG, "ready");

    for (int p = 72; p >= 48; p --) {
        ESP_LOGI(PROJECT_TAG, "pitch=%d", p);
        for (float a = 0; a <= 1; a += .01) {
            updateWaveRow(pitch2freq(p), a);
            vTaskDelay(2);
        }
    }
}

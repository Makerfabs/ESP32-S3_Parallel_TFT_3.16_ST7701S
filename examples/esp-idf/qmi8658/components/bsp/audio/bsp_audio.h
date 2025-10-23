#pragma once

#include "bsp_board.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/i2s_std.h"
#include "driver/i2s_common.h"

#include "freertos/queue.h"



typedef struct audio_t* audio_handle_t;

#define AUDIO_QUEUE_SIZE            (20)
#define AUDIO_QUEUE_NODE_SIZE       (2048)

typedef enum {
    AUDIO_PLAY_FORMAT_WAV = 0,
    AUDIO_PLAY_FORMAT_PCM,
    AUDIO_PLAY_FORMAT_MP3,
} audio_play_format_t;

typedef struct {
    void *data;
    size_t size;
    bool is_busy;
}audio_queue_node_t;


struct audio_t {
    i2s_chan_handle_t _tx_handle;
    i2s_chan_handle_t _rx_handle;

    QueueHandle_t _queue;

    uint32_t _output_sample_rate;
    uint32_t _input_sample_rate;
    uint16_t _bits_per_sample;
    uint16_t _channel_count;

    esp_err_t (*read)(void *data, size_t size, size_t *bytes_read, TickType_t ticks_to_wait);
    esp_err_t (*write)(const void *data, size_t size);
    esp_err_t (*play)(const char *source, audio_play_format_t format);
    esp_err_t (*del)();
    esp_err_t (*set_output_format)(uint32_t sample_rate, uint16_t bits_per_sample, uint16_t channel_count);
};

esp_err_t audio_init(int input_sample_rate, int output_sample_rate, gpio_num_t bclk, gpio_num_t ws, gpio_num_t dout, gpio_num_t din);
audio_handle_t bsp_audio_handle_get(void);
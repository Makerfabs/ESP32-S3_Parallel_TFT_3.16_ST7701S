#pragma once

#include "bsp_board.h"




#define AUDIO_QUEUE_SIZE            (20)
#define AUDIO_QUEUE_NODE_SIZE       (2048)
#define AUDIO_SAMPLE_RATE           (16000)
#define AUDIO_BITS_PER_SAMPLE       (16)
#define AUDIO_DATA_BIT_WIDTH        (16)
#define AUDIO_SLOT_MODE             (1)

#define BSP_AUDIO_I2S_GPIO_CFG        \
    {                                \
        .mclk = I2S_GPIO_UNUSED,     \
        .bclk = AUDIO_I2S_GPIO_BCLK, \
        .ws = AUDIO_I2S_GPIO_WS,     \
        .dout = AUDIO_I2S_GPIO_DOUT, \
        .din = AUDIO_I2S_GPIO_DIN,   \
        .invert_flags = {            \
            .mclk_inv = false,       \
            .bclk_inv = false,       \
            .ws_inv = false,         \
        },                           \
    }

#define BSP_AUDIO_I2S_DUPLEX_MONO_CFG(_sample_rate)                                                   \
    {                                                                                                 \
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(_sample_rate),                                          \
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(AUDIO_DATA_BIT_WIDTH, AUDIO_SLOT_MODE),           \
        .gpio_cfg = BSP_AUDIO_I2S_GPIO_CFG,                                                           \
    }

typedef struct bsp_audio_t* bsp_audio_handle_t;


typedef enum {
    AUDIO_PLAY_FORMAT_WAV = 0,
    AUDIO_PLAY_FORMAT_PCM,
    AUDIO_PLAY_FORMAT_MP3,
} audio_play_format_t;

typedef struct {
    void *data;
    size_t size;
    bool is_busy;
} bsp_audio_queue_node_t;


struct bsp_audio_t {
    QueueHandle_t _audio_queue;
    uint32_t output_sample_rate;
    uint32_t input_sample_rate;
    uint16_t bits_per_sample;
    uint16_t channel_count;

    esp_err_t (*read)(void *data, size_t to_read, size_t *bytes_read, uint32_t ticks_to_wait);
    esp_err_t (*write)(void *data, size_t to_write, size_t *bytes_written, uint32_t ticks_to_wait);
    esp_err_t (*add_queue_node)(void *data, size_t size);
    esp_err_t (*play)(const char *source, audio_play_format_t format);
    esp_err_t (*mute)(AUDIO_PLAYER_MUTE_SETTING setting);
    esp_err_t (*del)();
    esp_err_t (*set_output_format)(uint32_t sample_rate, uint32_t bits_per_sample, i2s_slot_mode_t ch);
};

esp_err_t bsp_audio_init(bsp_audio_handle_t *handle, bool enable_audio_queue_task);
#include "bsp_audio.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/unistd.h>

#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "audio_wav";
/**
 * @brief Header structure for WAV file with only one data chunk
 *
 * @note See this for reference: http://soundfile.sapp.org/doc/WaveFormat/
 *
 * @note Assignment to variables in this struct directly is only possible for little endian architectures
 *       (including Xtensa & RISC-V)
 */
typedef struct {
    struct {
        char chunk_id[4]; /*!< Contains the letters "RIFF" in ASCII form */
        uint32_t chunk_size; /*!< This is the size of the rest of the chunk following this number */
        char chunk_format[4]; /*!< Contains the letters "WAVE" */
    } descriptor_chunk; /*!< Canonical WAVE format starts with the RIFF header */
    struct {
        char subchunk_id[4]; /*!< Contains the letters "fmt " */
        uint32_t subchunk_size; /*!< This is the size of the rest of the Subchunk which follows this number */
        uint16_t audio_format; /*!< PCM = 1, values other than 1 indicate some form of compression */
        uint16_t num_of_channels; /*!< Mono = 1, Stereo = 2, etc. */
        uint32_t sample_rate; /*!< 8000, 44100, etc. */
        uint32_t byte_rate; /*!< ==SampleRate * NumChannels * BitsPerSample s/ 8 */
        uint16_t block_align; /*!< ==NumChannels * BitsPerSample / 8 */
        uint16_t bits_per_sample; /*!< 8 bits = 8, 16 bits = 16, etc. */
    } fmt_chunk; /*!< The "fmt " subchunk describes the sound data's format */
    struct {
        char subchunk_id[4]; /*!< Contains the letters "data" */
        uint32_t subchunk_size; /*!< ==NumSamples * NumChannels * BitsPerSample / 8 */
        // int16_t data[0]; /*!< Holds raw audio data */
    } data_chunk; /*!< The "data" subchunk contains the size of the data and the actual sound */
} wav_header_t;

/**
 * @brief Default header for PCM format WAV files
 *
 */
#define WAV_HEADER_PCM_DEFAULT(wav_sample_size, wav_sample_bits, wav_sample_rate, wav_channel_num) { \
    .descriptor_chunk = { \
        .chunk_id = {'R', 'I', 'F', 'F'}, \
        .chunk_size = (wav_sample_size) + sizeof(wav_header_t) - 8, \
        .chunk_format = {'W', 'A', 'V', 'E'} \
    }, \
    .fmt_chunk = { \
        .subchunk_id = {'f', 'm', 't', ' '}, \
        .subchunk_size = 16, /* 16 for PCM */ \
        .audio_format = 1, /* 1 for PCM */ \
        .num_of_channels = (wav_channel_num), \
        .sample_rate = (wav_sample_rate), \
        .byte_rate = (wav_sample_bits) * (wav_sample_rate) * (wav_channel_num) / 8, \
        .block_align = (wav_sample_bits) * (wav_channel_num) / 8, \
        .bits_per_sample = (wav_sample_bits)\
    }, \
    .data_chunk = { \
        .subchunk_id = {'d', 'a', 't', 'a'}, \
        .subchunk_size = (wav_sample_size) \
    } \
}


typedef struct __attribute__((packed)) {
	uint16_t audio_format;    // PCM == 1, anything else is unsupported here
	uint16_t num_channels;    // 1 for mono, 2 for stereo
	uint32_t sample_rate;     // playback sample rate in Hz
	uint32_t byte_rate;       // bytes per second (not strictly required)
	uint16_t block_align;     // bytes per audio frame
	uint16_t bits_per_sample; // bits per channel sample (16/24/32)
} wav_fmt_chunk_t;

static uint32_t read_le32(const uint8_t *ptr)
{
	return (uint32_t)ptr[0] |
	       ((uint32_t)ptr[1] << 8) |
	       ((uint32_t)ptr[2] << 16) |
	       ((uint32_t)ptr[3] << 24);
}

static uint16_t read_le16(const uint8_t *ptr)
{
	return (uint16_t)ptr[0] | (uint16_t)((uint32_t)ptr[1] << 8);
}

esp_err_t wav_read_header(FILE *file, wav_fmt_chunk_t *fmt, uint32_t *data_size, long *data_offset)
{
	/* Parse the canonical 44-byte PCM WAV header; files with extra fmt or metadata chunks are rejected. */
	uint8_t header[44];
	if(fread(header, 1, sizeof(header), file) != sizeof(header)) {
		ESP_LOGE(TAG, "failed to read WAV header");
		return ESP_FAIL;
	}

	if(memcmp(header, "RIFF", 4) != 0 || memcmp(&header[8], "WAVE", 4) != 0) {
		ESP_LOGE(TAG, "invalid WAV file magic");
		return ESP_ERR_INVALID_STATE;
	}

	if(memcmp(&header[12], "fmt ", 4) != 0) {
		ESP_LOGE(TAG, "fmt chunk not found");
		return ESP_ERR_NOT_FOUND;
	}

	uint32_t fmt_size = read_le32(&header[16]);
	if(fmt_size != 16) {
		ESP_LOGE(TAG, "unsupported fmt chunk size: %u", (unsigned)fmt_size);
		return ESP_ERR_NOT_SUPPORTED;
	}

	fmt->audio_format = read_le16(&header[20]);
	fmt->num_channels = read_le16(&header[22]);
	fmt->sample_rate = read_le32(&header[24]);
	fmt->byte_rate = read_le32(&header[28]);
	fmt->block_align = read_le16(&header[32]);
	fmt->bits_per_sample = read_le16(&header[34]);

	if(memcmp(&header[36], "data", 4) != 0) {
		ESP_LOGE(TAG, "data chunk not found");
		return ESP_ERR_NOT_FOUND;
	}

	*data_size = read_le32(&header[40]);
	*data_offset = sizeof(header);

	if(fmt->audio_format != 1) {
		ESP_LOGE(TAG, "unsupported WAV format (0x%04x)", fmt->audio_format);
		return ESP_ERR_NOT_SUPPORTED;
	}

	/* Log parsed header info once here so callers don't need to duplicate the message. */
	ESP_LOGI(TAG, "WAV header: %u Hz, %u-bit, %u channel(s), data %u bytes", (unsigned)fmt->sample_rate, (unsigned)fmt->bits_per_sample, (unsigned)fmt->num_channels, (unsigned)*data_size);

	return ESP_OK;
}

esp_err_t wav_play(audio_handle_t handle, const char *path)
{
	if(handle == NULL || handle->write == NULL || path == NULL) {
		return ESP_ERR_INVALID_ARG;
	}

	FILE *file = fopen(path, "rb");
	if(file == NULL) {
		ESP_LOGE(TAG, "failed to open %s", path);
		return ESP_FAIL;
	}

	wav_fmt_chunk_t fmt = {0};
	uint32_t data_size = 0;
	long data_offset = 0;
	esp_err_t ret = wav_read_header(file, &fmt, &data_size, &data_offset);
	if(ret != ESP_OK) {
		fclose(file);
		return ret;
	}

	if(fseek(file, data_offset, SEEK_SET) != 0) {
		ESP_LOGE(TAG, "failed to seek to data chunk");
		fclose(file);
		return ESP_FAIL;
	}

	if(fmt.num_channels != 1 && fmt.num_channels != 2) {
		ESP_LOGE(TAG, "unsupported channel count %u", (unsigned)fmt.num_channels);
		fclose(file);
		return ESP_ERR_NOT_SUPPORTED;
	}

	if(fmt.bits_per_sample != 16 && fmt.bits_per_sample != 24 && fmt.bits_per_sample != 32) {
		ESP_LOGE(TAG, "unsupported bit depth %u", (unsigned)fmt.bits_per_sample);
		fclose(file);
		return ESP_ERR_NOT_SUPPORTED;
	}

	if(fmt.sample_rate < 8000 || fmt.sample_rate > 48000) {
		ESP_LOGE(TAG, "unsupported sample rate %u", (unsigned)fmt.sample_rate);
		fclose(file);
		return ESP_ERR_NOT_SUPPORTED;
	}

	if(handle->set_output_format) {
		ret = handle->set_output_format(fmt.sample_rate, fmt.bits_per_sample, fmt.num_channels);
		if(ret != ESP_OK) {
			ESP_LOGE(TAG, "set_output_format failed: %s", esp_err_to_name(ret));
			fclose(file);
			return ret;
		}
	} else {
		ESP_LOGW(TAG, "set_output_format not implemented, using existing I2S configuration");
	}

	size_t bytes_per_sample = (fmt.bits_per_sample + 7U) / 8U;
	size_t frame_bytes = bytes_per_sample * fmt.num_channels;
	size_t output_sample_bytes = (fmt.bits_per_sample == 24) ? sizeof(int32_t) : bytes_per_sample;
	size_t max_frames_per_chunk = AUDIO_QUEUE_NODE_SIZE / (output_sample_bytes * fmt.num_channels);
	if(max_frames_per_chunk == 0) {
		max_frames_per_chunk = 1;
	}
	size_t chunk_bytes = frame_bytes * max_frames_per_chunk;

	uint8_t *raw_buffer = (uint8_t *)malloc(chunk_bytes);
	if(raw_buffer == NULL) {
		ESP_LOGE(TAG, "failed to allocate raw buffer");
		fclose(file);
		return ESP_ERR_NO_MEM;
	}

	uint8_t *converted_buffer = NULL;
	if(fmt.bits_per_sample == 24) {
		/* Promote 24-bit packed samples to 32-bit aligned values for the I2S driver. */
		converted_buffer = (uint8_t *)malloc(max_frames_per_chunk * fmt.num_channels * sizeof(int32_t));
		if(converted_buffer == NULL) {
			ESP_LOGE(TAG, "failed to allocate conversion buffer");
			free(raw_buffer);
			fclose(file);
			return ESP_ERR_NO_MEM;
		}
	}

	/* Playback logging handled in wav_read_header; avoid duplicate format logs here. */

	uint32_t remaining = data_size;
	ret = ESP_OK;

	while(remaining >= frame_bytes) {
		size_t frames_this_round = remaining / frame_bytes;
		if(frames_this_round > max_frames_per_chunk) {
			frames_this_round = max_frames_per_chunk;
		}

		size_t bytes_to_read = frames_this_round * frame_bytes;
		size_t read_bytes = fread(raw_buffer, 1, bytes_to_read, file);
		if(read_bytes != bytes_to_read) {
			ESP_LOGE(TAG, "failed to read audio data, expected %u got %u", (unsigned)bytes_to_read, (unsigned)read_bytes);
			ret = ESP_FAIL;
			break;
		}

		const uint8_t *payload = raw_buffer;
		size_t payload_bytes = read_bytes;

		if(fmt.bits_per_sample == 24) {
			int32_t *dst = (int32_t *)converted_buffer;
			const uint8_t *src = raw_buffer;
			size_t samples = frames_this_round * fmt.num_channels;
			for(size_t i = 0; i < samples; ++i) {
				int32_t value = (int32_t)((src[0]) | (src[1] << 8) | (src[2] << 16));
				value = (value << 8) >> 8; // sign extend 24-bit
				dst[i] = value;
				src += 3;
			}
			payload = converted_buffer;
			payload_bytes = samples * sizeof(int32_t);
		}

		bool sent = false;
		while(!sent) {
			ret = handle->write(payload, payload_bytes);
			if(ret == ESP_OK) {
				sent = true;
			} else if(ret == ESP_ERR_NO_MEM) {
				/* Allow the playback task to drain its queue before retrying. */
				vTaskDelay(pdMS_TO_TICKS(10));
			} else {
				ESP_LOGE(TAG, "audio write failed: %s", esp_err_to_name(ret));
				break;
			}
		}

		if(ret != ESP_OK) {
			break;
		}

		remaining -= read_bytes;
	}

	if(remaining > 0 && remaining < frame_bytes) {
		ESP_LOGW(TAG, "skipping %u trailing bytes", (unsigned)remaining);
	}

	free(converted_buffer);
	free(raw_buffer);
	fclose(file);

	return ret;
}


void wav_save(void)
{
	audio_init(16000, 16000, AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN);

	int flash_wr_size = 0;
	/* target bytes: sample_rate * bits_per_sample/8 * channels * seconds */
	uint32_t flash_rec_time = 16000 * 16 / 8 * 1 * 5; /* 5 seconds */

	/* write placeholder header (data size will be updated after recording) */
	wav_header_t wav_header = WAV_HEADER_PCM_DEFAULT(0, 16, 16000, 1);

	struct stat st;
	if (stat("/sdcard/record.wav", &st) == 0) {
		unlink("/sdcard/record.wav");
	}

	FILE *f = fopen("/sdcard/record.wav", "wb");
	if (f == NULL) {
		ESP_LOGE(TAG, "Failed to open file for writing");
		return;
	}

	/* write a placeholder header; we'll update sizes later */
	if (fwrite(&wav_header, sizeof(wav_header), 1, f) != 1) {
		ESP_LOGE(TAG, "Failed to write WAV header placeholder");
		fclose(f);
		return;
	}

	audio_handle_t _audio_handle = bsp_audio_handle_get();
	if (_audio_handle == NULL) {
		ESP_LOGE(TAG, "audio handle NULL, abort recording");
		fclose(f);
		return;
	}
	if (_audio_handle->_rx_handle == NULL) {
		ESP_LOGE(TAG, "audio rx handle NULL, abort recording");
		fclose(f);
		return;
	}

	const size_t CHUNK = 4 * 1024;
	uint8_t *i2s_readraw_buff = (uint8_t *)malloc(CHUNK);
	if (i2s_readraw_buff == NULL) {
		ESP_LOGE(TAG, "failed to allocate read buffer");
		fclose(f);
		return;
	}

	size_t bytes_read = 0;

	/* Start recording */
	while (flash_wr_size < (int)flash_rec_time) {
		esp_err_t r = i2s_channel_read(_audio_handle->_rx_handle, i2s_readraw_buff, CHUNK, &bytes_read, portMAX_DELAY);
		if (r == ESP_OK && bytes_read > 0) {
			fwrite(i2s_readraw_buff, 1, bytes_read, f);
			flash_wr_size += bytes_read;
		} else if (r != ESP_OK) {
			ESP_LOGE(TAG, "i2s read failed: %s", esp_err_to_name(r));
			break;
		} else {
			/* zero bytes read? avoid busy loop */
			vTaskDelay(pdMS_TO_TICKS(10));
		}
	}

	ESP_LOGI(TAG, "Recording done! bytes=%d", flash_wr_size);

	/* update header sizes */
	uint32_t data_size = (uint32_t)flash_wr_size;
	uint32_t riff_size = data_size + sizeof(wav_header_t) - 8;
	fseek(f, 4, SEEK_SET);
	fwrite(&riff_size, sizeof(riff_size), 1, f);
	fseek(f, 40, SEEK_SET);
	fwrite(&data_size, sizeof(data_size), 1, f);
	fflush(f);
	fclose(f);

	free(i2s_readraw_buff);

	vTaskDelay(pdMS_TO_TICKS(2000));

	/* attempt playback */
	esp_err_t play_ret = wav_play(_audio_handle, "/sdcard/record.wav");
	if (play_ret != ESP_OK) {
		ESP_LOGE(TAG, "wav_play failed: %s", esp_err_to_name(play_ret));
	}

	vTaskDelete(NULL);

}

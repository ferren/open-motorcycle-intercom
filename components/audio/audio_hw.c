/**
 * @file audio_hw.c
 * @brief I2S and Opus codec resource management.
 */

#include "audio_internal.h"

#include "esp_log.h"
static const char *TAG = "audio";

esp_err_t audio_hw_i2s_init(const audio_config_t *config)
{
    i2s_chan_config_t channel_config =
        I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    channel_config.dma_desc_num = I2S_DMA_BUFFER_COUNT;
    channel_config.dma_frame_num = I2S_DMA_BUFFER_SIZE;
    channel_config.auto_clear_after_cb = true;

    esp_err_t ret = i2s_new_channel(&channel_config, &g_audio.tx_chan, &g_audio.rx_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2S channel: %s", esp_err_to_name(ret));
        return ret;
    }

    i2s_std_config_t standard_config = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(config->sample_rate),
        .slot_cfg =
            I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = (gpio_num_t)config->i2s_pins.bclk_gpio,
            .ws = (gpio_num_t)config->i2s_pins.ws_gpio,
            .dout = (gpio_num_t)config->i2s_pins.dout_gpio,
            .din = (gpio_num_t)config->i2s_pins.din_gpio,
            .invert_flags = {0},
        },
    };
    ret = i2s_channel_init_std_mode(g_audio.tx_chan, &standard_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2S TX: %s", esp_err_to_name(ret));
        i2s_del_channel(g_audio.rx_chan);
        g_audio.rx_chan = NULL;
        i2s_del_channel(g_audio.tx_chan);
        g_audio.tx_chan = NULL;
        return ret;
    }
    ret = i2s_channel_init_std_mode(g_audio.rx_chan, &standard_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2S RX: %s", esp_err_to_name(ret));
        i2s_del_channel(g_audio.rx_chan);
        g_audio.rx_chan = NULL;
        i2s_del_channel(g_audio.tx_chan);
        g_audio.tx_chan = NULL;
        return ret;
    }
    ESP_LOGI(TAG, "I2S full duplex initialized on BCLK=%d WS=%d DIN=%d DOUT=%d",
             config->i2s_pins.bclk_gpio, config->i2s_pins.ws_gpio,
             config->i2s_pins.din_gpio, config->i2s_pins.dout_gpio);
    return ESP_OK;
}

void audio_hw_i2s_deinit(void)
{
    if (g_audio.rx_chan != NULL) {
        i2s_del_channel(g_audio.rx_chan);
        g_audio.rx_chan = NULL;
    }
    if (g_audio.tx_chan != NULL) {
        i2s_del_channel(g_audio.tx_chan);
        g_audio.tx_chan = NULL;
    }
}

esp_err_t audio_hw_opus_init(const audio_config_t *config)
{
    int error = OPUS_OK;
    g_audio.opus_encoder =
        opus_encoder_create(config->sample_rate, config->channels, OPUS_APPLICATION_VOIP, &error);
    if (error != OPUS_OK || g_audio.opus_encoder == NULL) {
        ESP_LOGE(TAG, "Failed to create Opus encoder: %s", opus_strerror(error));
        return ESP_FAIL;
    }
    opus_encoder_ctl(g_audio.opus_encoder, OPUS_SET_BITRATE(config->opus_bitrate));
    opus_encoder_ctl(g_audio.opus_encoder, OPUS_SET_VBR(1));
    opus_encoder_ctl(g_audio.opus_encoder, OPUS_SET_INBAND_FEC(1));
    opus_encoder_ctl(g_audio.opus_encoder, OPUS_SET_PACKET_LOSS_PERC(OPUS_EXPECTED_LOSS_PERC));
    opus_encoder_ctl(g_audio.opus_encoder, OPUS_SET_COMPLEXITY(5));
    opus_encoder_ctl(g_audio.opus_encoder, OPUS_SET_DTX(1));

    g_audio.loopback_decoder =
        opus_decoder_create(config->sample_rate, config->channels, &error);
    if (error != OPUS_OK || g_audio.loopback_decoder == NULL) {
        ESP_LOGE(TAG, "Failed to create loopback decoder: %s", opus_strerror(error));
        opus_encoder_destroy(g_audio.opus_encoder);
        g_audio.opus_encoder = NULL;
        return ESP_FAIL;
    }

    for (size_t i = 0; i < AUDIO_MAX_RX_SOURCES; ++i) {
        g_audio.rx_sources[i].decoder =
            opus_decoder_create(config->sample_rate, config->channels, &error);
        if (error != OPUS_OK || g_audio.rx_sources[i].decoder == NULL) {
            ESP_LOGE(TAG, "Failed to create source decoder %zu: %s", i, opus_strerror(error));
            return ESP_FAIL;
        }
    }
    ESP_LOGI(TAG, "Opus encoder and %u independent decoders initialized",
             AUDIO_MAX_RX_SOURCES + 1);
    return ESP_OK;
}

void audio_hw_opus_deinit(void)
{
    if (g_audio.opus_encoder != NULL) {
        opus_encoder_destroy(g_audio.opus_encoder);
        g_audio.opus_encoder = NULL;
    }
    if (g_audio.loopback_decoder != NULL) {
        opus_decoder_destroy(g_audio.loopback_decoder);
        g_audio.loopback_decoder = NULL;
    }
    for (size_t i = 0; i < AUDIO_MAX_RX_SOURCES; ++i) {
        if (g_audio.rx_sources[i].decoder != NULL) {
            opus_decoder_destroy(g_audio.rx_sources[i].decoder);
            g_audio.rx_sources[i].decoder = NULL;
        }
    }
}

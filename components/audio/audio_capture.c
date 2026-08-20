/**
 * @file audio_capture.c
 * @brief Capture task: I2S read, DSP chain, VOX, Opus encode, TX handoff.
 */

#include "audio_internal.h"

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
static const char *TAG = "audio";

static void hpf_init(audio_hpf_state_t *state, float cutoff_hz, float sample_rate)
{
    memset(state, 0, sizeof(*state));
    float omega = 2.0f * M_PI * cutoff_hz / sample_rate;
    float alpha = sinf(omega) / (2.0f * 0.7071f);
    float cosine = cosf(omega);
    float a0 = 1.0f + alpha;
    state->b0 = (1.0f + cosine) / (2.0f * a0);
    state->b1 = -(1.0f + cosine) / a0;
    state->b2 = state->b0;
    state->a1 = (-2.0f * cosine) / a0;
    state->a2 = (1.0f - alpha) / a0;
}

static void hpf_process(audio_hpf_state_t *state, int16_t *samples, size_t count)
{
    for (size_t i = 0; i < count; ++i) {
        float input = (float)samples[i] / 32768.0f;
        float output = state->b0 * input + state->b1 * state->x1 +
                       state->b2 * state->x2 - state->a1 * state->y1 -
                       state->a2 * state->y2;
        state->x2 = state->x1;
        state->x1 = input;
        state->y2 = state->y1;
        state->y1 = output;
        float scaled = output * 32768.0f;
        if (scaled > INT16_MAX) {
            scaled = INT16_MAX;
        } else if (scaled < INT16_MIN) {
            scaled = INT16_MIN;
        }
        samples[i] = (int16_t)scaled;
    }
}

void audio_capture_init_dsp(void)
{
    hpf_init(&g_audio.hpf, g_audio.config.hpf_cutoff_hz, g_audio.config.sample_rate);
    vox_init(&g_audio.vox, &g_audio.config.vox_config);
    voice_cleanup_init(&g_audio.voice_cleanup);
}

/* The INMP441's 24-bit left-channel I2S words become signed 16-bit PCM. */
static void convert_i2s_frame(const int32_t *i2s_buffer, size_t frame_samples)
{
    for (size_t i = 0; i < AUDIO_FRAME_SAMPLES; ++i) {
        if (i >= frame_samples) {
            g_audio.pcm_input[i] = 0;
            continue;
        }
        g_audio.pcm_input[i] = (int16_t)(i2s_buffer[i * I2S_CAPTURE_CHANNELS] >> 16);
    }
}

static void apply_voice_cleanup(void)
{
#if AUDIO_ENABLE_AEC_NS
    if (g_audio.config.mode == AUDIO_MODE_MESH) {
        int16_t far_reference[AUDIO_FRAME_SAMPLES];
        portENTER_CRITICAL(&g_audio_far_ref_lock);
        memcpy(far_reference, g_audio.far_ref_frame, sizeof(far_reference));
        portEXIT_CRITICAL(&g_audio_far_ref_lock);
        voice_cleanup_process(&g_audio.voice_cleanup, g_audio.pcm_input, far_reference,
                              AUDIO_FRAME_SAMPLES);
    }
#endif
}

static bool detect_voice_activity(void)
{
    bool was_active = g_audio.vox.active;
    bool vox_active = vox_process(&g_audio.vox, g_audio.pcm_input, AUDIO_FRAME_SAMPLES);
    portENTER_CRITICAL(&g_audio_task_lock);
    audio_activity_cb_t activity_callback = g_audio.activity_callback;
    portEXIT_CRITICAL(&g_audio_task_lock);
    if (vox_active != was_active && activity_callback != NULL) {
        activity_callback(vox_active);
    }
    AUDIO_STATS_LOCK();
    g_audio.stats.vox_active = vox_active;
    g_audio.stats.vox_activations = g_audio.vox.activation_count;
    AUDIO_STATS_UNLOCK();
    return vox_active;
}

static int encode_frame(const int16_t *input, int64_t *encode_time_sum,
                        uint32_t *encoded_frames)
{
    int64_t encode_start = esp_timer_get_time();
    int encoded = opus_encode(g_audio.opus_encoder, input, AUDIO_FRAME_SAMPLES,
                              g_audio.opus_buffer, sizeof(g_audio.opus_buffer));
    int64_t encode_time = esp_timer_get_time() - encode_start;
    if (encoded <= 0) {
        AUDIO_STATS_LOCK();
        g_audio.stats.encode_errors++;
        AUDIO_STATS_UNLOCK();
        ESP_LOGW(TAG, "Opus encode failed: %s", opus_strerror(encoded));
        return encoded;
    }
    *encode_time_sum += encode_time;
    AUDIO_STATS_LOCK();
    g_audio.stats.frames_encoded++;
    g_audio.stats.encode_time_us_avg =
        (uint32_t)(*encode_time_sum / g_audio.stats.frames_encoded);
    if ((uint32_t)encode_time > g_audio.stats.encode_time_us_max) {
        g_audio.stats.encode_time_us_max = (uint32_t)encode_time;
    }
    *encoded_frames = g_audio.stats.frames_encoded;
    AUDIO_STATS_UNLOCK();
    return encoded;
}

static void deliver_encoded_frame(int encoded, bool tx_active, int64_t frame_start_us)
{
    bool comfort_update = encoded > OPUS_DTX_FRAME_MAX_BYTES;
    if (g_audio.config.mode == AUDIO_MODE_MESH) {
        if (tx_active || comfort_update) {
            portENTER_CRITICAL(&g_audio_task_lock);
            audio_tx_cb_t tx_callback = g_audio.tx_callback;
            portEXIT_CRITICAL(&g_audio_task_lock);
            if (tx_callback != NULL) {
                tx_callback(g_audio.opus_buffer, (uint16_t)encoded, tx_active,
                            frame_start_us);
            }
        } else {
            AUDIO_STATS_LOCK();
            g_audio.stats.tx_dtx_suppressed++;
            AUDIO_STATS_UNLOCK();
        }
        return;
    }
    audio_loopback_item_t item = {
        .length = (uint16_t)encoded,
        .timestamp_us = frame_start_us,
    };
    memcpy(item.data, g_audio.opus_buffer, (size_t)encoded);
    if (xQueueSend(g_audio.loopback_queue, &item, 0) != pdTRUE) {
        AUDIO_STATS_LOCK();
        g_audio.stats.frames_dropped++;
        AUDIO_STATS_UNLOCK();
    }
}

static void record_frame_latency(int64_t frame_start_us, int64_t *latency_sum,
                                 uint32_t encoded_frames)
{
    int64_t processing_time_us = esp_timer_get_time() - frame_start_us;
    /* NOTE: latency stats cover local processing plus this DMA estimate;
     * they do not measure mouth-to-ear latency over the radio. Two
     * descriptors bound queued speaker data to roughly one or two frames. */
    int64_t dma_latency_us = ((int64_t)I2S_DMA_BUFFER_COUNT / 2) * 20000;
    int64_t total_latency_us = processing_time_us + dma_latency_us;
    *latency_sum += total_latency_us;
    AUDIO_STATS_LOCK();
    g_audio.stats.latency_ms_avg =
        (uint32_t)((*latency_sum / (int64_t)encoded_frames) / 1000);
    uint32_t latency_ms = (uint32_t)(total_latency_us / 1000);
    if (latency_ms > g_audio.stats.latency_ms_max) {
        g_audio.stats.latency_ms_max = latency_ms;
    }
    AUDIO_STATS_UNLOCK();
}

static void capture_task_finish(void)
{
    portENTER_CRITICAL(&g_audio_task_lock);
    g_audio.capture_task = NULL;
    portEXIT_CRITICAL(&g_audio_task_lock);
    xSemaphoreGive(g_audio.capture_done);
    vTaskDelete(NULL);
}

/* Returns the number of bytes read, or 0 when this loop iteration has no frame. */
static size_t read_i2s_frame(int32_t *i2s_buffer, size_t buffer_size)
{
    size_t bytes_read = 0;
    esp_err_t ret = i2s_channel_read(g_audio.rx_chan, i2s_buffer, buffer_size,
                                     &bytes_read, I2S_READ_TIMEOUT_MS);
    if (ret == ESP_ERR_TIMEOUT) {
        AUDIO_STATS_LOCK();
        g_audio.stats.capture_timeouts++;
        AUDIO_STATS_UNLOCK();
        return 0;
    }
    if (ret != ESP_OK || bytes_read == 0) {
        AUDIO_STATS_LOCK();
        g_audio.stats.i2s_read_errors++;
        AUDIO_STATS_UNLOCK();
        ESP_LOGW(TAG, "I2S microphone read error: %s", esp_err_to_name(ret));
        return 0;
    }
    AUDIO_STATS_LOCK();
    if (bytes_read == I2S_CAPTURE_FRAME_BYTES) {
        g_audio.stats.capture_frames_ok++;
    } else {
        g_audio.stats.capture_short_reads++;
    }
    AUDIO_STATS_UNLOCK();
    return bytes_read;
}

void audio_capture_task(void *arg)
{
    (void)arg;
    static int32_t i2s_buffer[AUDIO_FRAME_SAMPLES * I2S_CAPTURE_CHANNELS];
    static int16_t silence_frame[AUDIO_FRAME_SAMPLES];
    int64_t encode_time_sum = 0;
    int64_t latency_sum = 0;

    opus_encoder_ctl(g_audio.opus_encoder, OPUS_RESET_STATE);
    esp_err_t ret = i2s_channel_enable(g_audio.rx_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable I2S RX: %s", esp_err_to_name(ret));
        atomic_store_explicit(&g_audio.running, false, memory_order_release);
        xSemaphoreGive(g_audio.capture_started);
        capture_task_finish();
    }
    atomic_store_explicit(&g_audio.capture_ready, true, memory_order_release);
    xSemaphoreGive(g_audio.capture_started);
    ESP_LOGI(TAG, "Capture task started on core %d", xPortGetCoreID());

    while (atomic_load_explicit(&g_audio.running, memory_order_acquire)) {
        int64_t frame_start_us = esp_timer_get_time();
        AUDIO_STATS_LOCK();
        g_audio.stats.task_loops++;
        AUDIO_STATS_UNLOCK();

        size_t bytes_read = read_i2s_frame(i2s_buffer, sizeof(i2s_buffer));
        if (bytes_read == 0) {
            continue;
        }

        convert_i2s_frame(i2s_buffer,
                          bytes_read / (I2S_CAPTURE_CHANNELS * sizeof(i2s_buffer[0])));
        if (g_audio.config.enable_hpf) {
            hpf_process(&g_audio.hpf, g_audio.pcm_input, AUDIO_FRAME_SAMPLES);
        }
        apply_voice_cleanup();

        bool vox_active = detect_voice_activity();
        bool tx_active = vox_active || g_audio.config.force_tx_always;
        const int16_t *encode_input = tx_active ? g_audio.pcm_input : silence_frame;
        uint32_t encoded_frames = 0;
        int encoded = encode_frame(encode_input, &encode_time_sum, &encoded_frames);
        if (encoded <= 0) {
            continue;
        }
        deliver_encoded_frame(encoded, tx_active, frame_start_us);
        record_frame_latency(frame_start_us, &latency_sum, encoded_frames);
    }

    i2s_channel_disable(g_audio.rx_chan);
    ESP_LOGI(TAG, "Capture task stopped");
    capture_task_finish();
}

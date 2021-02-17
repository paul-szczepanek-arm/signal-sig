#include "mbed.h"
#include "stm32l475e_iot01_audio.h"
#include <cmath>

uint32_t isqrt32(uint32_t n)
{
    uint32_t root, remainder, place;

    root = 0;
    remainder = n;
    place = 0x40000000;

    while (place > remainder) {
        place = place >> 2;
    }
    while (place)  {
        if (remainder >= root + place) {
            remainder = remainder - root - place;
            root = root + (place << 1);
        }
        root = root >> 1;
        place = place >> 2;
    }
    return root;
}

class Recorder
{
public:
    /* buffer sizes */
    static constexpr size_t DRIVER_BUFFER_SIZE_IN_MS = 100;
    static constexpr size_t APP_BUFFER_SIZE_IN_MS = 2000;
    static constexpr size_t CHUNK_SIZE_IN_MS = 500;

    static constexpr size_t BYTES_PER_SAMPLE = 2;

    static constexpr size_t DRIVER_SAMPLES = (AUDIO_SAMPLING_FREQUENCY * DRIVER_BUFFER_SIZE_IN_MS) / 1000;
    static constexpr size_t DRIVER_BYTES = DRIVER_SAMPLES * BYTES_PER_SAMPLE;
    static constexpr size_t APP_SAMPLES = (AUDIO_SAMPLING_FREQUENCY * APP_BUFFER_SIZE_IN_MS) / 1000;
    static constexpr size_t APP_BYTES = APP_SAMPLES * BYTES_PER_SAMPLE;
    static constexpr size_t CHUNK_SAMPLES = (AUDIO_SAMPLING_FREQUENCY * CHUNK_SIZE_IN_MS) / 1000;
    static constexpr size_t CHUNK_BYTES = CHUNK_SAMPLES * BYTES_PER_SAMPLE;

    Recorder(EventQueue& event_queue) : queue(event_queue) {};
    int init() {
        driver_buffer = (uint8_t *) malloc(DRIVER_BYTES);
        app_buffer = (uint16_t *) malloc(APP_BYTES);
        
        if (!driver_buffer || !app_buffer) {
            printf("Failed to allocate app_buffer buffer\r\n");
            return 1;
        }
        
        BSP_AUDIO_Init_t MicParams;
        MicParams.BitsPerSample = BYTES_PER_SAMPLE * 8;
        MicParams.ChannelsNbr = AUDIO_CHANNELS;
        MicParams.Device = AUDIO_IN_DIGITAL_MIC1;
        MicParams.SampleRate = AUDIO_SAMPLING_FREQUENCY; //16Khz
        MicParams.Volume = 32;

        int32_t ret = BSP_AUDIO_IN_Init(AUDIO_INSTANCE, &MicParams);

        if (ret != BSP_ERROR_NONE) {
            printf("Error Audio Init (%ld)\r\r\n", ret);
            return 1;
        } else {
            printf("OK Audio Init\t(Audio Freq=%ld)\r\r\n", AUDIO_SAMPLING_FREQUENCY);
        }

        return 0;
    }

    void record() {
        int32_t ret;
        uint32_t state;

        ret = BSP_AUDIO_IN_GetState(AUDIO_INSTANCE, &state);
        if (ret != BSP_ERROR_NONE) {
            printf("Cannot start recording: Error getting audio state (%d)\r\n", ret);
            return;
        }
        if (state == AUDIO_IN_STATE_RECORDING) {
            printf("Cannot start recording: Already recording\r\n");
            return;
        }

        app_buffer_index = 0;

        ret = BSP_AUDIO_IN_Record(AUDIO_INSTANCE, driver_buffer, DRIVER_BYTES);
        if (ret != BSP_ERROR_NONE) {
            printf("Error Audio Record (%ld)\r\n", ret);
            return;
        } else {
            printf("OK Audio Record\r\n");
        }
    }

    void stop() {
        int32_t ret = BSP_AUDIO_IN_Stop(AUDIO_INSTANCE);
        if (ret != BSP_ERROR_NONE) {
            printf("Error Audio Stop (%d)\r\n", ret);
        }
    }

    /* every time we copy half the driver buffer, when full is true we copy the second half */
    void audio_in(bool full) {
        if (skip_events) {
            skip_events--;
            return;
        }

        const uint8_t *source = full ? driver_buffer + DRIVER_BYTES / 2 : driver_buffer;
        memcpy(app_buffer + app_buffer_index, source, DRIVER_BYTES / 2);

        accumulated_chunk += DRIVER_SAMPLES / 2;
        app_buffer_index += DRIVER_SAMPLES / 2;

        /* TODO: measure how much analysis is lagging behind capture to skip frames */

        app_buffer_index %= APP_SAMPLES;

        if (accumulated_chunk >= CHUNK_SAMPLES) {
            accumulated_chunk = 0;
            chunk_index = (app_buffer_index - CHUNK_SAMPLES);
            chunk_index %= APP_SAMPLES;
            queue.call(this, &Recorder::process_chunk);
            return;
        }
    }

private:
    void process_chunk() {
        printf("Processing %dms\r\n", CHUNK_SIZE_IN_MS);
        analyse_chunk();
        filter_chunk();
        analyse_chunk();
    }

    void filter_chunk() {
        uint16_t *source = app_buffer + chunk_index;

        /* our sampling freq 16khz for 2khz low pass alpha = 1/8 we shift by 3 */
        const uint32_t alpha = 1;
        const uint32_t alpha_scaling = 3; /* 1/8 << 3 = 1*/
        const uint32_t added_precision_bits = 4;

        static uint32_t scaled_value = 0;

        for (size_t n = 0; n < CHUNK_SAMPLES; n++) {
            const uint32_t value = source[n];
            uint32_t scaled_change = (alpha * ((value << added_precision_bits) - scaled_value)) >> alpha_scaling;
            scaled_value += scaled_change;
            source[n] = (uint16_t)(scaled_value >> added_precision_bits);
        }
    }

    void analyse_chunk() {
        const uint16_t *source = app_buffer + chunk_index;

        /* average */
        uint32_t sum = 0;
        uint16_t sum_avg = source[0];

        /* standard deviation */
        int32_t M2 = 0;
        int32_t delta = 0;

        /* histogram buckets */
        const size_t n_bucket = 8;
        const size_t bucket_size = 2000;
        size_t samples_in_bucket[n_bucket] = { 0 };
        
        for (size_t n = 0; n < CHUNK_SAMPLES; n++) {
            const uint16_t value = source[n];

            /* average */
            sum += value;
            uint16_t sum_avg = sum / (n + 1);

            /* standard deviation */
            const int32_t new_delta = (int32_t)value - (int32_t)sum_avg;
            M2 += delta * new_delta;
            delta = new_delta;

            /* histogram */
            size_t bucket_idx = (value / bucket_size);
            if (bucket_idx >= n_bucket) {
                bucket_idx = n_bucket - 1;
            }
            samples_in_bucket[bucket_idx]++;
        }

        const uint32_t variance = std::abs((int32_t)(M2 / CHUNK_SAMPLES));
        const uint32_t dev = isqrt32(variance);

        printf("sum %016lu sum_avg %08lu dev %08d\r\n", sum, sum_avg, dev);

        // histogram
        printf("%04lu %04lu %04lu %04lu %04lu %04lu %04lu %04lu\r\n",
               samples_in_bucket[0], samples_in_bucket[1], samples_in_bucket[2], samples_in_bucket[3],
               samples_in_bucket[4], samples_in_bucket[5], samples_in_bucket[6], samples_in_bucket[7]
        );
    }

private:
    EventQueue& queue;
    uint8_t *driver_buffer = nullptr;

    /* TODO: replace with circular buffer */
    uint16_t *app_buffer = nullptr;
    size_t app_buffer_index = 0;
    size_t chunk_index = 0;
    size_t accumulated_chunk = 0;

    /* first recorder full driver buffer will be skipped */
    size_t skip_events = 2;
};

static EventQueue event_queue;
static Recorder recorder(event_queue);

/* these are hardcoded in the driver to handle interrupts */
void BSP_AUDIO_IN_HalfTransfer_CallBack(uint32_t Instance) {
    recorder.audio_in(false);
}

void BSP_AUDIO_IN_TransferComplete_CallBack(uint32_t Instance) {
    recorder.audio_in(true);
}

void BSP_AUDIO_IN_Error_CallBack(uint32_t Instance) {
    printf("BSP_AUDIO_IN_Error_CallBack\r\n");
}

int main() {
    int ret = recorder.init();
    if (ret) {
        return ret;
    }

    recorder.record();

    event_queue.dispatch_forever();
}

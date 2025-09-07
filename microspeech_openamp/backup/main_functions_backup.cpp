#include "main_functions.hpp"

#include <zephyr/logging/log.h>

#include "tensorflow/lite/core/c/common.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_log.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"

/* tflite model */
#include "../tflite/micro_model_settings.h"
#include "../tflite/audio_preprocessor_int8_model_data.h"
#include "../tflite/micro_speech_quantized_model_data.h"

/* test input */
#include "no_1000ms_audio_data.h"
#include "yes_1000ms_audio_data.h"
#include "silence_1000ms_audio_data.h"
#include "noise_1000ms_audio_data.h"

namespace {

/* Arena size for model operations */
constexpr size_t kArenaSize = 28584;  /* xtensa p6 */
alignas(16) uint8_t g_arena[kArenaSize];

/* Features type for audio processing */
using Features = int8_t[kFeatureCount][kFeatureSize];
Features g_features;

/* Audio sample constants */
constexpr int kAudioSampleDurationCount =
    kFeatureDurationMs * kAudioSampleFrequency / 1000;
constexpr int kAudioSampleStrideCount =
    kFeatureStrideMs * kAudioSampleFrequency / 1000;

/* Operation resolver types */
using MicroSpeechOpResolver = tflite::MicroMutableOpResolver<4>;
using AudioPreprocessorOpResolver = tflite::MicroMutableOpResolver<18>;

/**
 * @brief Register operations for micro speech model
 *
 * @param op_resolver Operation resolver to register operations with
 * @return TfLiteStatus indicating success or failure
 */
TfLiteStatus register_micro_speech_ops(MicroSpeechOpResolver& op_resolver)
{
    TF_LITE_ENSURE_STATUS(op_resolver.AddReshape());
    TF_LITE_ENSURE_STATUS(op_resolver.AddFullyConnected());
    TF_LITE_ENSURE_STATUS(op_resolver.AddDepthwiseConv2D());
    TF_LITE_ENSURE_STATUS(op_resolver.AddSoftmax());
    return kTfLiteOk;
}

/**
 * @brief Register operations for audio preprocessor model
 *
 * @param op_resolver Operation resolver to register operations with
 * @return TfLiteStatus indicating success or failure
 */
TfLiteStatus register_audio_preprocessor_ops(AudioPreprocessorOpResolver& op_resolver)
{
    TF_LITE_ENSURE_STATUS(op_resolver.AddReshape());
    TF_LITE_ENSURE_STATUS(op_resolver.AddCast());
    TF_LITE_ENSURE_STATUS(op_resolver.AddStridedSlice());
    TF_LITE_ENSURE_STATUS(op_resolver.AddConcatenation());
    TF_LITE_ENSURE_STATUS(op_resolver.AddMul());
    TF_LITE_ENSURE_STATUS(op_resolver.AddAdd());
    TF_LITE_ENSURE_STATUS(op_resolver.AddDiv());
    TF_LITE_ENSURE_STATUS(op_resolver.AddMinimum());
    TF_LITE_ENSURE_STATUS(op_resolver.AddMaximum());
    TF_LITE_ENSURE_STATUS(op_resolver.AddWindow());
    TF_LITE_ENSURE_STATUS(op_resolver.AddFftAutoScale());
    TF_LITE_ENSURE_STATUS(op_resolver.AddRfft());
    TF_LITE_ENSURE_STATUS(op_resolver.AddEnergy());
    TF_LITE_ENSURE_STATUS(op_resolver.AddFilterBank());
    TF_LITE_ENSURE_STATUS(op_resolver.AddFilterBankSquareRoot());
    TF_LITE_ENSURE_STATUS(op_resolver.AddFilterBankSpectralSubtraction());
    TF_LITE_ENSURE_STATUS(op_resolver.AddPCAN());
    TF_LITE_ENSURE_STATUS(op_resolver.AddFilterBankLog());
    return kTfLiteOk;
}

/**
 * @brief Generate a single feature from audio data
 *
 * @param audio_data Input audio data
 * @param audio_data_size Size of audio data
 * @param feature_output Output buffer for the generated feature
 * @param interpreter TensorFlow Lite interpreter
 * @return TfLiteStatus indicating success or failure
 */
TfLiteStatus generate_single_feature(const int16_t* audio_data,
                                    const int audio_data_size,
                                    int8_t* feature_output,
                                    tflite::MicroInterpreter* interpreter)
{
    TfLiteTensor* input = interpreter->input(0);
    if (!input) {
        MicroPrintf("Failed to get input tensor");
        return kTfLiteError;
    }

    /* Check input shape is compatible with our audio sample size */
    if (audio_data_size != kAudioSampleDurationCount) {
        MicroPrintf("Audio data size mismatch: %d vs %d", audio_data_size, kAudioSampleDurationCount);
        return kTfLiteError;
    }

    TfLiteTensor* output = interpreter->output(0);
    if (!output) {
        MicroPrintf("Failed to get output tensor");
        return kTfLiteError;
    }

    /* Copy audio data to input tensor */
    std::copy_n(audio_data, audio_data_size, tflite::GetTensorData<int16_t>(input));

    /* Run inference */
    if (interpreter->Invoke() != kTfLiteOk) {
        MicroPrintf("Invoke failed");
        return kTfLiteError;
    }

    /* Copy output to feature buffer */
    std::copy_n(tflite::GetTensorData<int8_t>(output), kFeatureSize, feature_output);

    return kTfLiteOk;
}

/**
 * @brief Generate features from audio data
 *
 * @param audio_data Input audio data
 * @param audio_data_size Size of audio data
 * @param features_output Output buffer for the generated features
 * @return TfLiteStatus indicating success or failure
 */
TfLiteStatus generate_features(const int16_t* audio_data,
                            const size_t audio_data_size,
                            Features* features_output)
{
    /* Load the audio preprocessing model */
    const tflite::Model* model = tflite::GetModel(audio_preprocessor_int8_tflite);
    if (model->version() != TFLITE_SCHEMA_VERSION) {
        MicroPrintf("Audio preprocessor model version mismatch: %d vs %d",
            model->version(), TFLITE_SCHEMA_VERSION);
        return kTfLiteError;
    }

    /* Set up operations */
    AudioPreprocessorOpResolver op_resolver;
    if (register_audio_preprocessor_ops(op_resolver) != kTfLiteOk) {
        MicroPrintf("Failed to register audio preprocessor ops");
        return kTfLiteError;
    }

    /* Create interpreter */
    tflite::MicroInterpreter interpreter(model, op_resolver, g_arena, kArenaSize);

    /* Allocate tensors */
    if (interpreter.AllocateTensors() != kTfLiteOk) {
        MicroPrintf("Failed to allocate tensors for audio preprocessor");
        return kTfLiteError;
    }

    MicroPrintf("AudioPreprocessor model arena size = %u", interpreter.arena_used_bytes());

    /* Process audio in stride windows */
    size_t remaining_samples = audio_data_size;
    size_t feature_index = 0;
    const int16_t* current_audio = audio_data;

    while (remaining_samples >= kAudioSampleDurationCount && feature_index < kFeatureCount) {
        if (generate_single_feature(current_audio, kAudioSampleDurationCount,
                    (*features_output)[feature_index], &interpreter) != kTfLiteOk) {
            MicroPrintf("Failed to generate feature %d", feature_index);
            return kTfLiteError;
        }

        feature_index++;
        current_audio += kAudioSampleStrideCount;
        remaining_samples -= kAudioSampleStrideCount;
    }

    return kTfLiteOk;
}

/**
 * @brief Run speech recognition on generated features
 *
 * @param features Input features
 * @param expected_label Expected recognition result
 * @return TfLiteStatus indicating success or failure
 */
TfLiteStatus run_micro_speech_inference(const Features& features, const char* expected_label)
{
    /* Load the speech recognition model */
    const tflite::Model* model = tflite::GetModel(micro_speech_quantized_tflite);
    if (model->version() != TFLITE_SCHEMA_VERSION) {
        MicroPrintf("MicroSpeech model version mismatch: %d vs %d",
            model->version(), TFLITE_SCHEMA_VERSION);
        return kTfLiteError;
    }

    /* Set up operations */
    MicroSpeechOpResolver op_resolver;
    if (register_micro_speech_ops(op_resolver) != kTfLiteOk) {
        MicroPrintf("Failed to register MicroSpeech ops");
        return kTfLiteError;
    }

    /* Create interpreter */
    tflite::MicroInterpreter interpreter(model, op_resolver, g_arena, kArenaSize);

    /* Allocate tensors */
    if (interpreter.AllocateTensors() != kTfLiteOk) {
        MicroPrintf("Failed to allocate tensors for MicroSpeech");
        return kTfLiteError;
    }

    MicroPrintf("MicroSpeech model arena size = %u", interpreter.arena_used_bytes());

    /* Get input tensor */
    TfLiteTensor* input = interpreter.input(0);
    if (!input) {
        MicroPrintf("Failed to get input tensor");
        return kTfLiteError;
    }

    /* Check input shape is compatible with our feature data size */
    if (input->dims->data[input->dims->size - 1] != kFeatureElementCount) {
        MicroPrintf("Input tensor size mismatch");
        return kTfLiteError;
    }

    TfLiteTensor* output = interpreter.output(0);
    if (!output) {
        MicroPrintf("Failed to get output tensor");
        return kTfLiteError;
    }

    /* Check output shape is compatible with our number of categories */
    if (output->dims->data[output->dims->size - 1] != kCategoryCount) {
        MicroPrintf("Output tensor size mismatch");
        return kTfLiteError;
    }

    float output_scale = output->params.scale;
    int output_zero_point = output->params.zero_point;

    /* Copy feature data to input tensor */
    std::copy_n(&features[0][0], kFeatureElementCount, tflite::GetTensorData<int8_t>(input));

    /* Run inference */
    if (interpreter.Invoke() != kTfLiteOk) {
        MicroPrintf("Invoke failed");
        return kTfLiteError;
    }

    /* Dequantize and print results */
    float category_predictions[kCategoryCount];
    MicroPrintf("MicroSpeech category predictions for <%s>", expected_label);
    
    for (int i = 0; i < kCategoryCount; i++) {
        category_predictions[i] = (tflite::GetTensorData<int8_t>(output)[i] - output_zero_point) * output_scale;
        MicroPrintf("  %.4f %s", (double)category_predictions[i], kCategoryLabels[i]);
    }

    /* Find the prediction with highest confidence */
    int prediction_index = std::distance(
        std::begin(category_predictions),
        std::max_element(std::begin(category_predictions), std::end(category_predictions))
    );

    MicroPrintf("Detected: %s (Expected: %s)", kCategoryLabels[prediction_index], expected_label);

    /* Check if prediction matches expected label */
    if (strcmp(kCategoryLabels[prediction_index], expected_label) != 0) {
        MicroPrintf("Recognition mismatch!");
    }

    return kTfLiteOk;
}

} /* namespace */

/* C API implementation */

extern "C" {

int micro_speech_process_audio(const char *expected_label, const int16_t *audio_data,
                        size_t audio_data_size) {
    /* Generate features */
    if (generate_features(audio_data, audio_data_size, &g_features) != kTfLiteOk) {
        MicroPrintf("Failed to generate features");
        return -1;
    }

    /* Run inference */
    if (run_micro_speech_inference(g_features, expected_label) != kTfLiteOk) {
        MicroPrintf("Inference failed");
        return -2;
    }
    
    return 0;
}

void setup() {
    printf("MicroSpeech porting to qemu_xtensa\n");
}

void loop() {
    int ret;
    /* Process test audio samples */
    MicroPrintf("Processing 'yes' audio sample...");
    ret = micro_speech_process_audio("yes", g_yes_1000ms_audio_data, g_yes_1000ms_audio_data_size);
    if (ret != 0) {
        MicroPrintf("Failed to process 'yes' sample: %d", ret);
    }

    MicroPrintf("Processing 'no' audio sample...");
    ret = micro_speech_process_audio("no", g_no_1000ms_audio_data, g_no_1000ms_audio_data_size);
    if (ret != 0) {
        MicroPrintf("Failed to process 'no' sample: %d", ret);
    }

    MicroPrintf("Processing 'silence' audio sample...");
    ret = micro_speech_process_audio("silence", g_silence_1000ms_audio_data, g_silence_1000ms_audio_data_size);
    if (ret != 0) {
        MicroPrintf("Failed to process 'silence' sample: %d", ret);
    }

    MicroPrintf("Processing 'noise' audio sample...");
    ret = micro_speech_process_audio("silence", g_noise_1000ms_audio_data, g_noise_1000ms_audio_data_size);
    if (ret != 0) {
        MicroPrintf("Failed to process 'noise' sample: %d", ret);
    }
}
} /* extern "C" */
/*
 * Copyright 2024 The TensorFlow Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

 #include "micro_speech.h"

 #include <zephyr/logging/log.h>
 #include <algorithm>
 #include <cstdint>
 #include <iterator>
 
 #include "tensorflow/lite/core/c/common.h"
 #include "tensorflow/lite/micro/examples/micro_speech/micro_model_settings.h"
 #include "tensorflow/lite/micro/examples/micro_speech/models/audio_preprocessor_int8_model_data.h"
 #include "tensorflow/lite/micro/examples/micro_speech/models/micro_speech_quantized_model_data.h"
 #include "tensorflow/lite/micro/examples/micro_speech/testdata/no_1000ms_audio_data.h"
 #include "tensorflow/lite/micro/examples/micro_speech/testdata/yes_1000ms_audio_data.h"
 #include "tensorflow/lite/micro/examples/micro_speech/testdata/silence_1000ms_audio_data.h"
 #include "tensorflow/lite/micro/examples/micro_speech/testdata/noise_1000ms_audio_data.h"
 #include "tensorflow/lite/micro/micro_interpreter.h"
 #include "tensorflow/lite/micro/micro_log.h"
 #include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
 
 LOG_MODULE_REGISTER(micro_speech, CONFIG_LOG_DEFAULT_LEVEL);
 
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
         LOG_ERR("Failed to get input tensor");
         return kTfLiteError;
     }
 
     /* Check input shape is compatible with our audio sample size */
     if (audio_data_size != kAudioSampleDurationCount) {
         LOG_ERR("Audio data size mismatch: %d vs %d", audio_data_size, kAudioSampleDurationCount);
         return kTfLiteError;
     }
 
     TfLiteTensor* output = interpreter->output(0);
     if (!output) {
         LOG_ERR("Failed to get output tensor");
         return kTfLiteError;
     }
 
     /* Copy audio data to input tensor */
     std::copy_n(audio_data, audio_data_size, tflite::GetTensorData<int16_t>(input));
 
     /* Run inference */
     if (interpreter->Invoke() != kTfLiteOk) {
         LOG_ERR("Invoke failed");
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
     const tflite::Model* model = tflite::GetModel(g_audio_preprocessor_int8_model_data);
     if (model->version() != TFLITE_SCHEMA_VERSION) {
         LOG_ERR("Audio preprocessor model version mismatch: %d vs %d",
             model->version(), TFLITE_SCHEMA_VERSION);
         return kTfLiteError;
     }
 
     /* Set up operations */
     AudioPreprocessorOpResolver op_resolver;
     if (register_audio_preprocessor_ops(op_resolver) != kTfLiteOk) {
         LOG_ERR("Failed to register audio preprocessor ops");
         return kTfLiteError;
     }
 
     /* Create interpreter */
     tflite::MicroInterpreter interpreter(model, op_resolver, g_arena, kArenaSize);
 
     /* Allocate tensors */
     if (interpreter.AllocateTensors() != kTfLiteOk) {
         LOG_ERR("Failed to allocate tensors for audio preprocessor");
         return kTfLiteError;
     }
 
     LOG_INF("AudioPreprocessor model arena size = %u", interpreter.arena_used_bytes());
 
     /* Process audio in stride windows */
     size_t remaining_samples = audio_data_size;
     size_t feature_index = 0;
     const int16_t* current_audio = audio_data;
 
     while (remaining_samples >= kAudioSampleDurationCount && feature_index < kFeatureCount) {
         if (generate_single_feature(current_audio, kAudioSampleDurationCount,
                        (*features_output)[feature_index], &interpreter) != kTfLiteOk) {
             LOG_ERR("Failed to generate feature %d", feature_index);
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
     const tflite::Model* model = tflite::GetModel(g_micro_speech_quantized_model_data);
     if (model->version() != TFLITE_SCHEMA_VERSION) {
         LOG_ERR("MicroSpeech model version mismatch: %d vs %d",
             model->version(), TFLITE_SCHEMA_VERSION);
         return kTfLiteError;
     }
 
     /* Set up operations */
     MicroSpeechOpResolver op_resolver;
     if (register_micro_speech_ops(op_resolver) != kTfLiteOk) {
         LOG_ERR("Failed to register MicroSpeech ops");
         return kTfLiteError;
     }
 
     /* Create interpreter */
     tflite::MicroInterpreter interpreter(model, op_resolver, g_arena, kArenaSize);
 
     /* Allocate tensors */
     if (interpreter.AllocateTensors() != kTfLiteOk) {
         LOG_ERR("Failed to allocate tensors for MicroSpeech");
         return kTfLiteError;
     }
 
     LOG_INF("MicroSpeech model arena size = %u", interpreter.arena_used_bytes());
 
     /* Get input tensor */
     TfLiteTensor* input = interpreter.input(0);
     if (!input) {
         LOG_ERR("Failed to get input tensor");
         return kTfLiteError;
     }
 
     /* Check input shape is compatible with our feature data size */
     if (input->dims->data[input->dims->size - 1] != kFeatureElementCount) {
         LOG_ERR("Input tensor size mismatch");
         return kTfLiteError;
     }
 
     TfLiteTensor* output = interpreter.output(0);
     if (!output) {
         LOG_ERR("Failed to get output tensor");
         return kTfLiteError;
     }
 
     /* Check output shape is compatible with our number of categories */
     if (output->dims->data[output->dims->size - 1] != kCategoryCount) {
         LOG_ERR("Output tensor size mismatch");
         return kTfLiteError;
     }
 
     float output_scale = output->params.scale;
     int output_zero_point = output->params.zero_point;
 
     /* Copy feature data to input tensor */
     std::copy_n(&features[0][0], kFeatureElementCount, tflite::GetTensorData<int8_t>(input));
 
     /* Run inference */
     if (interpreter.Invoke() != kTfLiteOk) {
         LOG_ERR("Invoke failed");
         return kTfLiteError;
     }
 
     /* Dequantize and print results */
     float category_predictions[kCategoryCount];
     LOG_INF("MicroSpeech category predictions for <%s>", expected_label);
     
     for (int i = 0; i < kCategoryCount; i++) {
         category_predictions[i] = (tflite::GetTensorData<int8_t>(output)[i] - output_zero_point) * output_scale;
         LOG_INF("  %.4f %s", (double)category_predictions[i], kCategoryLabels[i]);
     }
 
     /* Find the prediction with highest confidence */
     int prediction_index = std::distance(
         std::begin(category_predictions),
         std::max_element(std::begin(category_predictions), std::end(category_predictions))
     );
 
     LOG_INF("Detected: %s (Expected: %s)", kCategoryLabels[prediction_index], expected_label);
 
     /* Check if prediction matches expected label */
     if (strcmp(kCategoryLabels[prediction_index], expected_label) != 0) {
         LOG_WRN("Recognition mismatch!");
     }
 
     return kTfLiteOk;
 }
 
 } /* namespace */
 
 /* C API implementation */
 
 extern "C" {
 
 int micro_speech_initialize(void)
 {
     /* Nothing to initialize here */
     return 0;
 }
 
 int micro_speech_process_audio(const char *expected_label, const int16_t *audio_data,
                               size_t audio_data_size)
 {
     /* Generate features */
     if (generate_features(audio_data, audio_data_size, &g_features) != kTfLiteOk) {
         LOG_ERR("Failed to generate features");
         return -1;
     }
 
     /* Run inference */
     if (run_micro_speech_inference(g_features, expected_label) != kTfLiteOk) {
         LOG_ERR("Inference failed");
         return -2;
     }
 
     return 0;
 }
 
 } /* extern "C" */
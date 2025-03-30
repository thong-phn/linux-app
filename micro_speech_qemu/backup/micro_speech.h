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

 #ifndef MICRO_SPEECH_H_
 #define MICRO_SPEECH_H_
 
 #include <stdint.h>
 #include <stddef.h>
 
 #ifdef __cplusplus
 extern "C" {
 #endif
 
 /**
  * @brief Initialize the micro speech recognition system
  *
  * @return 0 on success, negative errno code on failure
  */
 int micro_speech_initialize(void);
 
 /**
  * @brief Process an audio sample and perform speech recognition
  *
  * @param expected_label Expected recognition label (for testing)
  * @param audio_data Pointer to audio sample data
  * @param audio_data_size Size of audio data in bytes
  * 
  * @return 0 on success, negative errno code on failure
  */
 int micro_speech_process_audio(const char *expected_label, const int16_t *audio_data, 
                                size_t audio_data_size);
 
 /* Exposed for direct reference in main.c */
 extern const int16_t g_yes_1000ms_audio_data[];
 extern const int g_yes_1000ms_audio_data_size;
 extern const int16_t g_no_1000ms_audio_data[];
 extern const int g_no_1000ms_audio_data_size;
 extern const int16_t g_silence_1000ms_audio_data[];
 extern const int g_silence_1000ms_audio_data_size;
 extern const int16_t g_noise_1000ms_audio_data[];
 extern const int g_noise_1000ms_audio_data_size;
 
 #ifdef __cplusplus
 }
 #endif
 
 #endif /* MICRO_SPEECH_H_ */
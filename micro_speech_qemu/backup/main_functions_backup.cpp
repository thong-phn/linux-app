/*
 * Copyright 2020 The TensorFlow Authors. All Rights Reserved.
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

#include "main_functions.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

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

/* Globals, used for compatibility with Arduino-style sketches. */
namespace {
	tflite::ErrorReporter* 		error_reporter = nullptr;
	const tflite::Model* 		model = nullptr;
	tflite::MicroInterpreter* 	interpreter = nullptr;
	TfLiteTensor* 				model_input = nullptr;
	FeatureProvider* 			feature_provider = nullptr;
	RecognizeCommands* 			recognizer = nullptr;
	int32_t 					previous_time = 0;
	
	// tflite related variables
	constexpr int kTensorArenaSize = 28584; // xtensa P6
	uint8_t tensor_arena[kTensorArenaSize];
	int8_t feature_buffer[kFeatureElementCount];
	int8_t* model_input_buffer = nullptr;
}  /* namespace */

using Features = int8_t[kFeatureCount][kFeatureSize];

TfLiteStatus GenerateSingleFeature(const int16_t* audio_data,
									const int audio_data_size,
									int8_t* feature_output,
									tflite::MicroInterpreter* interpreter) {
	// Set up model input
	TfLiteTensor* input = interpreter->input(0);
	if (!input) {
		printk("Failed to get input tensor\n");
		return kTfLiteError;
	}
	
	// Check input dimensions
	if (audio_data_size != kAudioSampleDurationCount) {
		printk("Audio data size mismatch: %d vs %d\n", audio_data_size, kAudioSampleDurationCount);
		return kTfLiteError;
	}
	
	// Copy audio data to input tensor
	std::copy_n(audio_data, audio_data_size, tflite::GetTensorData<int16_t>(input));
	
	// Run inference
	if (interpreter->Invoke() != kTfLiteOk) {
		printk("Invoke failed\n");
		return kTfLiteError;
	}
	
	// Get output
	TfLiteTensor* output = interpreter->output(0);
	if (!output) {
		printk("Failed to get output tensor\n");
		return kTfLiteError;
	}
	
	// Copy output to feature buffer
	std::copy_n(tflite::GetTensorData<int8_t>(output), kFeatureSize, feature_output);
	
	return kTfLiteOk;
}

TfLiteStatus GenerateFeatures(const int16_t* audio_data,
							  const size_t audio_data_size,
						      Features* features_output) {
	// Load the audio preprocessing model
    const tflite::Model* model = tflite::GetModel(g_audio_preprocessor_int8_model_data);
    if (model->version() != TFLITE_SCHEMA_VERSION) {
        printk("Audio preprocessor model version mismatch\n");
        return kTfLiteError;
    }
    
    // Set up operations
    static tflite::MicroMutableOpResolver <18> AudioPreprocessorResolver;
	AudioPreprocessorResolver.AddReshape();
	AudioPreprocessorResolver.AddCast();
	AudioPreprocessorResolver.AddStridedSlice();
	AudioPreprocessorResolver.AddConcatenation();
	AudioPreprocessorResolver.AddMul();
	AudioPreprocessorResolver.AddAdd();
	AudioPreprocessorResolver.AddDiv();
	AudioPreprocessorResolver.AddMinimum();
	AudioPreprocessorResolver.AddMaximum();
	AudioPreprocessorResolver.AddWindow();
	AudioPreprocessorResolver.AddFftAutoScale();
	AudioPreprocessorResolver.AddRfft();
	AudioPreprocessorResolver.AddEnergy();
	AudioPreprocessorResolver.AddFilterBank();
	AudioPreprocessorResolver.AddFilterBankSquareRoot();
	AudioPreprocessorResolver.AddFilterBankSpectralSubtraction();
	AudioPreprocessorResolver.AddPCAN();
	AudioPreprocessorResolver.AddFilterBankLog();
    if ((AudioPreprocessorResolver) != kTfLiteOk) {
        printk("Failed to register audio preprocessor ops\n");
        return kTfLiteError;
    }
    
    // Create interpreter
    tflite::MicroInterpreter preprocessing_interpreter(model, AudioPreprocessorResolver, tensor_arena, kTensorArenaSize);
	interpreter = &preprocessing_interpreter;
    
    // Allocate tensors
    if (interpreter.AllocateTensors() != kTfLiteOk) {
        printk("Failed to allocate tensors for audio preprocessor\n");
        return kTfLiteError;
    }
    
    printk("AudioPreprocessor model arena size = %u\n", interpreter.arena_used_bytes());
    
    // Process audio in stride windows
    size_t remaining_samples = audio_data_size;
    size_t feature_index = 0;
    const int16_t* current_audio = audio_data;
    
    while (remaining_samples >= kAudioSampleDurationCount && feature_index < kFeatureCount) {
        if (GenerateSingleFeature(current_audio, kAudioSampleDurationCount, 
                                  (*features_output)[feature_index], &interpreter) != kTfLiteOk) {
            printk("Failed to generate feature %d\n", feature_index);
            return kTfLiteError;
        }
        
        feature_index++;
        current_audio += kAudioSampleStrideCount;
        remaining_samples -= kAudioSampleStrideCount;
    }
    
    return kTfLiteOk;
}

/* The name of this function is important for Arduino compatibility */
void setup(void)
{
	/* Map the model to a usable data structrure */
	model = tflite::GetModel(g_model);
	if (model->version() != TFLITE_SCHEMA_VERSION) {
		MicroPrintf("Model provided is schema version %d not equal "
					"to supported version %d.",
					model->version(), TFLITE_SCHEMA_VERSION);
		return;
	}

	/* This pulls 4 + 18 operation implementations */
	static tflite::MicroMutableOpResolver <4> MicroSpeechResolver;
	MicroSpeechResolver.AddReshape();
	MicroSpeechResolver.AddFullyConnected();
	MicroSpeechResolver.AddDepthwiseConv2D();
	MicroSpeechResolver.AddSoftmax();
	
	


	/* Build an interpreter to run the model with. */
	static tflite::MicroInterpreter static_interpreter(
		model, resolver, tensor_arena, kTensorArenaSize);
	interpreter = &static_interpreter;

	/* Allocate memory from the tensor_arena for the model's tensors. */
	TfLiteStatus allocate_status = interpreter->AllocateTensors();
	if (allocate_status != kTfLiteOk) {
		MicroPrintf("AllocateTensors() failed");
		return;
	}
}

/* The name of this function is important for Arduino compatibility. */
void loop(void)
{
	/* Calculate an x value to feed into the model. We compare the current
	 * inference_count to the number of inferences per cycle to determine
	 * our position within the range of possible x values the model was
	 * trained on, and use this to calculate a value.
	 */
	float position = static_cast < float > (inference_count) /
			 static_cast < float > (kInferencesPerCycle);
	float x = position * kXrange;

	/* Quantize the input from floating-point to integer */
	int8_t x_quantized = x / input->params.scale + input->params.zero_point;
	/* Place the quantized input in the model's input tensor */
	input->data.int8[0] = x_quantized;

	/* Run inference, and report any error */
	TfLiteStatus invoke_status = interpreter->Invoke();
	if (invoke_status != kTfLiteOk) {
		MicroPrintf("Invoke failed on x: %f\n", static_cast < double > (x));
		return;
	}

	/* Obtain the quantized output from model's output tensor */
	int8_t y_quantized = output->data.int8[0];
	/* Dequantize the output from integer to floating-point */
	float y = (y_quantized - output->params.zero_point) * output->params.scale;

	/* Output the results. A custom HandleOutput function can be implemented
	 * for each supported hardware target.
	 */
	HandleOutput(x, y);

	/* Increment the inference_counter, and reset it if we have reached
	 * the total number per cycle
	 */
	inference_count += 1;
	if (inference_count >= kInferencesPerCycle) inference_count = 0;
}

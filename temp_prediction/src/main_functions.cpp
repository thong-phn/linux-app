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

#include <tensorflow/lite/micro/micro_mutable_op_resolver.h>
#include "model.hpp"
#include <tensorflow/lite/micro/micro_log.h>
#include <tensorflow/lite/micro/micro_interpreter.h>
#include <tensorflow/lite/micro/system_setup.h>
#include <tensorflow/lite/schema/schema_generated.h>

/* Globals, used for compatibility with Arduino-style sketches. */
namespace {
	const tflite::Model *model = nullptr;
	tflite::MicroInterpreter *interpreter = nullptr;
	TfLiteTensor *input = nullptr;
	TfLiteTensor *output = nullptr;

	constexpr int kTensorArenaSize = 2000;
	uint8_t tensor_arena[kTensorArenaSize];
}  /* namespace */

/* The name of this function is important for Arduino compatibility. */
void setup(void)
{
	/* Map the model into a usable data structure. */
	model = tflite::GetModel(g_model);
	if (model->version() != TFLITE_SCHEMA_VERSION) {
		MicroPrintf("Model provided is schema version %d not equal "
					"to supported version %d.",
					model->version(), TFLITE_SCHEMA_VERSION);
		return;
	}

	/* Pull operations. */
	static tflite::MicroMutableOpResolver <1> resolver;
	resolver.AddFullyConnected();

	/* Build an interpreter to run the model with. */
	static tflite::MicroInterpreter static_interpreter(
		model, resolver, tensor_arena, kTensorArenaSize);
	interpreter = &static_interpreter;

	/* Allocate memory from the tensor_arena for the model's tensors. */
	TfLiteStatus allocate_status = interpreter->AllocateTensors();
	if (allocate_status != kTfLiteOk) {
		MicroPrintf("AllocateTensors() failed. Adjust the tensor_arena\n");
		return;
	}

	/* Obtain pointers to the model's input and output tensors. */
	input = interpreter->input(0);
	output = interpreter->output(0);
}

/* The name of this function is important for Arduino compatibility. */
void loop(void)
{
	/* Simulate the input */
	float hours[] = {10.0, 11.0, 12.0, 13.0, 14.0, 15.0};
	int num_inputs = sizeof(hours) / sizeof(hours[0]);
	
	for (int i = 0; i < num_inputs; i++) {
		input->data.f[0] = (double) hours[i] / 23.0; // Normalize
		MicroPrintf("Hour: %f\n", (double) input->data.f[0] * 23.0);
		
		/* Run inference */
		TfLiteStatus invoke_status = interpreter->Invoke(); 
		if (invoke_status != kTfLiteOk) {
			MicroPrintf("Invoke failed\n");
			return;
		}

		/* Print the predicted output */
		float predicted_output = output->data.f[0];
		MicroPrintf("Predicted output: %f\n", (double) predicted_output);
	}
}

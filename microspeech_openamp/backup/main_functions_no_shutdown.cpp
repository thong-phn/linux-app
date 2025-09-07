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

/* rpmsg start */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/drivers/ipm.h>

#include <openamp/open_amp.h>
#include <metal/sys.h>
#include <metal/io.h>
#include <resource_table.h>

#ifdef CONFIG_SHELL_BACKEND_RPMSG
#include <zephyr/shell/shell_rpmsg.h>
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(openamp_rsc_table, LOG_LEVEL_DBG);

#define SHM_DEVICE_NAME	"shm"

#if !DT_HAS_CHOSEN(zephyr_ipc_shm)
#error "Sample requires definition of shared memory for rpmsg"
#endif

/* Constants derived from device tree */
#define SHM_NODE		DT_CHOSEN(zephyr_ipc_shm)
#define SHM_START_ADDR	DT_REG_ADDR(SHM_NODE)
#define SHM_SIZE		DT_REG_SIZE(SHM_NODE)

#define APP_TASK_STACK_SIZE (1024)

/* Add 1024 extra bytes for the TTY task stack for the "tx_buff" buffer. */
#define APP_TTY_TASK_STACK_SIZE (1536)

K_THREAD_STACK_DEFINE(thread_mng_stack, APP_TASK_STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_tty_stack, APP_TTY_TASK_STACK_SIZE);

static struct k_thread thread_mng_data;
static struct k_thread thread_tty_data;

static const struct device *const ipm_handle =
	DEVICE_DT_GET(DT_CHOSEN(zephyr_ipc));

static metal_phys_addr_t shm_physmap = SHM_START_ADDR;
static metal_phys_addr_t rsc_tab_physmap;

static struct metal_io_region shm_io_data; /* shared memory */
static struct metal_io_region rsc_io_data; /* rsc_table memory */

struct rpmsg_rcv_msg {
	void *data;
	size_t len;
};

static struct metal_io_region *shm_io = &shm_io_data;

static struct metal_io_region *rsc_io = &rsc_io_data;
static struct rpmsg_virtio_device rvdev;

static struct fw_resource_table *rsc_table;
static struct rpmsg_device *rpdev;

static struct rpmsg_endpoint tty_ept;
static struct rpmsg_rcv_msg tty_msg;

static K_SEM_DEFINE(data_sem, 0, 1);
static K_SEM_DEFINE(data_tty_sem, 0, 1);

static int16_t file_buffer[1004];  // Buffer to store reconstructed file
static uint32_t file_size = 0;     // Current file size
static uint32_t chunk_count = 0;   // Number of chunks received

/* rpmsg end */

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

    // Send prediction result through rpmsg
    char prediction_buff[256];
    snprintf(prediction_buff, sizeof(prediction_buff), 
             "[Z] Detected: %s (Expected: %s)\n", 
             kCategoryLabels[prediction_index], expected_label);
    
    // Send through rpmsg if TTY endpoint is available
    if (tty_ept.addr != RPMSG_ADDR_ANY) {
        rpmsg_send(&tty_ept, prediction_buff, strlen(prediction_buff));
    }
    
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

static void platform_ipm_callback(const struct device *dev, void *context,
				  uint32_t id, volatile void *data)
{
	LOG_DBG("%s: msg received from mb %d", __func__, id);
	k_sem_give(&data_sem);
}

static int rpmsg_recv_tty_callback(struct rpmsg_endpoint *ept, void *data,
				   size_t len, uint32_t src, void *priv)
{
	struct rpmsg_rcv_msg *msg = static_cast<struct rpmsg_rcv_msg *>(priv);

	rpmsg_hold_rx_buffer(ept, data);
	msg->data = data;
	msg->len = len;
	k_sem_give(&data_tty_sem);

	return RPMSG_SUCCESS;
}

static void receive_message(unsigned char **msg, unsigned int *len)
{
	int status = k_sem_take(&data_sem, K_FOREVER);

	if (status == 0) {
		rproc_virtio_notified(rvdev.vdev, VRING1_ID);
	}
}

static void new_service_cb(struct rpmsg_device *rdev, const char *name,
			   uint32_t src)
{
	LOG_ERR("%s: unexpected ns service receive for name %s",
		__func__, name);
}

int mailbox_notify(void *priv, uint32_t id)
{
	ARG_UNUSED(priv);

	LOG_DBG("%s: msg received", __func__);
	ipm_send(ipm_handle, 0, id, NULL, 0);

	return 0;
}

int platform_init(void)
{
	int rsc_size;
	struct metal_init_params metal_params = METAL_INIT_DEFAULTS;
	int status;

	status = metal_init(&metal_params);
	if (status) {
		LOG_ERR("metal_init: failed: %d", status);
		return -1;
	}

	/* declare shared memory region */
	metal_io_init(shm_io, (void *)SHM_START_ADDR, &shm_physmap,
		      SHM_SIZE, -1, 0, NULL);

	/* declare resource table region */
	rsc_table_get(&rsc_table, &rsc_size);
	rsc_tab_physmap = (uintptr_t)rsc_table;

	metal_io_init(rsc_io, rsc_table,
		      &rsc_tab_physmap, rsc_size, -1, 0, NULL);

	/* setup IPM */
	if (!device_is_ready(ipm_handle)) {
		LOG_ERR("IPM device is not ready");
		return -1;
	}

	ipm_register_callback(ipm_handle, platform_ipm_callback, NULL);

	status = ipm_set_enabled(ipm_handle, 1);
	if (status) {
		LOG_ERR("ipm_set_enabled failed");
		return -1;
	}

	return 0;
}

static void cleanup_system(void)
{
	ipm_set_enabled(ipm_handle, 0);
	rpmsg_deinit_vdev(&rvdev);
	metal_finish();
}

struct  rpmsg_device *
platform_create_rpmsg_vdev(unsigned int vdev_index,
			   unsigned int role,
			   void (*rst_cb)(struct virtio_device *vdev),
			   rpmsg_ns_bind_cb ns_cb)
{
	struct fw_rsc_vdev_vring *vring_rsc;
	struct virtio_device *vdev;
	int ret;

	vdev = rproc_virtio_create_vdev(VIRTIO_DEV_DEVICE, VDEV_ID,
					rsc_table_to_vdev(rsc_table),
					rsc_io, NULL, mailbox_notify, NULL);

	if (!vdev) {
		LOG_ERR("failed to create vdev");
		return NULL;
	}

	/* wait master rpmsg init completion */
	rproc_virtio_wait_remote_ready(vdev);

	vring_rsc = rsc_table_get_vring0(rsc_table);
	ret = rproc_virtio_init_vring(vdev, 0, vring_rsc->notifyid,
				      (void *)vring_rsc->da, rsc_io,
				      vring_rsc->num, vring_rsc->align);
	if (ret) {
		LOG_ERR("failed to init vring 0");
		goto failed;
	}

	vring_rsc = rsc_table_get_vring1(rsc_table);
	ret = rproc_virtio_init_vring(vdev, 1, vring_rsc->notifyid,
				      (void *)vring_rsc->da, rsc_io,
				      vring_rsc->num, vring_rsc->align);
	if (ret) {
		LOG_ERR("failed to init vring 1");
		goto failed;
	}

	ret = rpmsg_init_vdev(&rvdev, vdev, ns_cb, shm_io, NULL);
	if (ret) {
		LOG_ERR("failed rpmsg_init_vdev");
		goto failed;
	}

	return rpmsg_virtio_get_rpmsg_device(&rvdev);

failed:
	rproc_virtio_remove_vdev(vdev);

	return NULL;
}

void app_rpmsg_tty(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	char tx_buff[512];
	int ret = 0;

	k_sem_take(&data_tty_sem,  K_FOREVER);

	tty_ept.priv = &tty_msg;
	ret = rpmsg_create_ept(&tty_ept, rpdev, "rpmsg-tty",
			       RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
			       rpmsg_recv_tty_callback, NULL);
	if (ret) {
		LOG_ERR("[Linux TTY] Could not create endpoint: %d", ret);
		goto task_end;
	}

	// Send ready message
	snprintf(tx_buff, sizeof(tx_buff), "[Z] Ready to receive\n");
    rpmsg_send(&tty_ept, tx_buff, strlen(tx_buff));
	
	// Counter
	file_size = 0;
	chunk_count = 0;

    while (tty_ept.addr != RPMSG_ADDR_ANY) {
        k_sem_take(&data_tty_sem, K_FOREVER);
        if (tty_msg.len) {
            // Check for end marker
            if (tty_msg.len == 3 && memcmp(tty_msg.data, "EOF", 3) == 0) {
                // Process the file
                snprintf(tx_buff, sizeof(tx_buff), 
                        "[Z] feed audio data to micro-speech\n");
                rpmsg_send(&tty_ept, tx_buff, strlen(tx_buff));

                ret = micro_speech_process_audio("yes", file_buffer, 1005);
                if (ret != 0) {
                    MicroPrintf("Failed to process 'yes' sample: %d", ret);
                    // Debug print
                    snprintf(tx_buff, sizeof(tx_buff), 
                            "[Z] Failed to process 'yes' sample: %d\n", ret);
                    rpmsg_send(&tty_ept, tx_buff, strlen(tx_buff));
                }
                // Debug print
                snprintf(tx_buff, sizeof(tx_buff), 
                        "[Z] Transfer complete: %u bytes in %u chunks\n", file_size, chunk_count);
                rpmsg_send(&tty_ept, tx_buff, strlen(tx_buff));
                
                // Reset for next transfer
                file_size = 0;
                chunk_count = 0;
            } else {
                chunk_count++;
                
                // Copy chunk to file buffer
                if (file_size + tty_msg.len <= sizeof(file_buffer)) {
                    memcpy(&file_buffer[file_size], tty_msg.data, tty_msg.len);
                    file_size += tty_msg.len;
                    
                    // Debug print
                    snprintf(tx_buff, sizeof(tx_buff),
                        "[Z] Chunk %u: %d bytes (total: %u)\n",
                        chunk_count, (int)tty_msg.len, file_size);
                    rpmsg_send(&tty_ept, tx_buff, strlen(tx_buff));
                }
            }
            
            rpmsg_release_rx_buffer(&tty_ept, tty_msg.data);
        }
        tty_msg.len = 0;
        tty_msg.data = NULL;
    }
	rpmsg_destroy_ept(&tty_ept);

task_end:
	LOG_INF("OpenAMP Linux TTY responder ended");
    snprintf(tx_buff, sizeof(tx_buff), "[Z] OpenAMP Linux TTY responder ended\n");
    rpmsg_send(&tty_ept, tx_buff, strlen(tx_buff));
}

void rpmsg_mng_task(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	unsigned char *msg;
	unsigned int len;
	int ret = 0;

	LOG_INF("OpenAMP[remote] Linux responder demo started");

	/* Initialize platform */
	ret = platform_init();
	if (ret) {
		LOG_ERR("Failed to initialize platform");
		ret = -1;
		goto task_end;
	}

	rpdev = platform_create_rpmsg_vdev(0, VIRTIO_DEV_DEVICE, NULL,
					   new_service_cb);
	if (!rpdev) {
		LOG_ERR("Failed to create rpmsg virtio device");
		ret = -1;
		goto task_end;
	}

#ifdef CONFIG_SHELL_BACKEND_RPMSG
	(void)shell_backend_rpmsg_init_transport(rpdev);
#endif

	/* start the rpmsg clients */
	k_sem_give(&data_tty_sem);

	while (1) {
		receive_message(&msg, &len);
	}

task_end:
	cleanup_system();
	LOG_INF("OpenAMP demo ended");
}



void setup() {
    printf("MicroSpeech porting to qemu_xtensa\n");
    LOG_INF("Starting OpenAMP application threads!");
    
    // Create the management thread
    k_thread_create(&thread_mng_data, thread_mng_stack, APP_TASK_STACK_SIZE,
        rpmsg_mng_task,
        NULL, NULL, NULL, K_PRIO_COOP(8), 0, K_NO_WAIT);
    
    // Create the TTY thread  
    k_thread_create(&thread_tty_data, thread_tty_stack, 4096,
        app_rpmsg_tty,
        NULL, NULL, NULL, K_PRIO_COOP(7), 0, K_NO_WAIT);
}

void loop() {
    // Keep the main thread alive and let the other threads do their work
    // Check if threads are still running
    k_sleep(K_MSEC(1000));
}
} /* extern "C" */
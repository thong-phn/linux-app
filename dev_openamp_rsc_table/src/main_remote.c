/*
 * Copyright (c) 2020, STMICROELECTRONICS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

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

/* Audio processing constants */
/*
 * Audio Reconstruction System:
 * - Receives 496-byte blocks from primary core via RPMsg
 * - Each block has a 16-byte header containing chunk_id, block_number, total_blocks, block_size
 * - Reconstructs blocks into complete 1000ms audio chunks (~192KB)
 * - Provides complete chunks to application via get_audio_chunk() API
 * - Thread-safe using semaphores for synchronization
 */
#define AUDIO_BLOCK_SIZE 496           /* 496 bytes per block */
#define AUDIO_HEADER_SIZE 16           /* 16-byte header */
#define AUDIO_RPMSG_BUFFER_SIZE 512    /* Total RPMsg buffer size */
#define AUDIO_CHUNK_DURATION_MS 1000   /* 1 second chunks */
#define AUDIO_SAMPLE_RATE 48000        /* Assuming 48kHz sample rate */
#define AUDIO_CHANNELS 2               /* Stereo */
#define AUDIO_SAMPLE_SIZE 2            /* 16-bit samples = 2 bytes */
#define AUDIO_CHUNK_SIZE (AUDIO_SAMPLE_RATE * AUDIO_CHANNELS * AUDIO_SAMPLE_SIZE) /* 192KB per 1s chunk */
#define AUDIO_BLOCKS_PER_CHUNK ((AUDIO_CHUNK_SIZE + AUDIO_BLOCK_SIZE - 1) / AUDIO_BLOCK_SIZE) /* ~387 blocks per chunk */
#define APP_AUDIO_TASK_STACK_SIZE (2048)

K_THREAD_STACK_DEFINE(thread_mng_stack, APP_TASK_STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_rp__client_stack, APP_TASK_STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_tty_stack, APP_TTY_TASK_STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_audio_stack, APP_AUDIO_TASK_STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_audio_app_stack, APP_AUDIO_TASK_STACK_SIZE);

static struct k_thread thread_mng_data;
static struct k_thread thread_rp__client_data;
static struct k_thread thread_tty_data;
static struct k_thread thread_audio_data;
static struct k_thread thread_audio_app_data;

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

static char rx_sc_msg[20];  /* should receive "Hello world!" */
static struct rpmsg_endpoint sc_ept;
static struct rpmsg_rcv_msg sc_msg = {.data = rx_sc_msg};

static struct rpmsg_endpoint tty_ept;
static struct rpmsg_rcv_msg tty_msg;

/* Audio processing structures */
struct audio_block_header {
	uint32_t chunk_id;        /* ID of the 1000ms chunk */
	uint32_t block_number;    /* Block number within chunk (0-based) */
	uint32_t total_blocks;    /* Total blocks in this chunk */
	uint32_t block_size;      /* Size of this block's data */
};

struct audio_chunk {
	uint8_t data[AUDIO_CHUNK_SIZE];  /* 192KB buffer for 1s of audio */
	uint32_t chunk_id;
	uint32_t received_blocks;
	uint32_t total_blocks;
	bool is_complete;
	bool is_ready;
};

static struct rpmsg_endpoint audio_ept;
static struct rpmsg_rcv_msg audio_msg;
static struct audio_chunk current_chunk;
static struct audio_chunk ready_chunk;

static K_SEM_DEFINE(data_sem, 0, 1);
static K_SEM_DEFINE(data_sc_sem, 0, 1);
static K_SEM_DEFINE(data_tty_sem, 0, 1);
static K_SEM_DEFINE(data_audio_sem, 0, 1);
static K_SEM_DEFINE(audio_chunk_ready_sem, 0, 1);

static void platform_ipm_callback(const struct device *dev, void *context,
				  uint32_t id, volatile void *data)
{
	LOG_DBG("%s: msg received from mb %d", __func__, id);
	k_sem_give(&data_sem);
}

static int rpmsg_recv_cs_callback(struct rpmsg_endpoint *ept, void *data,
				  size_t len, uint32_t src, void *priv)
{
	memcpy(sc_msg.data, data, len);
	sc_msg.len = len;
	k_sem_give(&data_sc_sem);

	return RPMSG_SUCCESS;
}

static int rpmsg_recv_tty_callback(struct rpmsg_endpoint *ept, void *data,
				   size_t len, uint32_t src, void *priv)
{
	struct rpmsg_rcv_msg *msg = priv;

	rpmsg_hold_rx_buffer(ept, data);
	msg->data = data;
	msg->len = len;
	k_sem_give(&data_tty_sem);

	return RPMSG_SUCCESS;
}

static int rpmsg_recv_audio_callback(struct rpmsg_endpoint *ept, void *data,
				     size_t len, uint32_t src, void *priv)
{
	struct rpmsg_rcv_msg *msg = priv;

	rpmsg_hold_rx_buffer(ept, data);
	msg->data = data;
	msg->len = len;
	k_sem_give(&data_audio_sem);

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

void app_rpmsg_client_sample(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	unsigned int msg_cnt = 0;
	int ret = 0;

	k_sem_take(&data_sc_sem,  K_FOREVER);

	LOG_INF("OpenAMP[remote] Linux sample client responder started");

	ret = rpmsg_create_ept(&sc_ept, rpdev, "rpmsg-client-sample",
			       RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
			       rpmsg_recv_cs_callback, NULL);
	if (ret) {
		LOG_ERR("[Linux sample client] Could not create endpoint: %d", ret);
		goto task_end;
	}

	while (msg_cnt < 100) {
		k_sem_take(&data_sc_sem,  K_FOREVER);
		msg_cnt++;
		LOG_INF("[Linux sample client] incoming msg %d: %.*s", msg_cnt, sc_msg.len,
			(char *)sc_msg.data);
		rpmsg_send(&sc_ept, sc_msg.data, sc_msg.len);
	}
	rpmsg_destroy_ept(&sc_ept);

task_end:
	LOG_INF("OpenAMP Linux sample client responder ended");
}

void app_rpmsg_tty(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	unsigned char tx_buff[512];
	int ret = 0;

	k_sem_take(&data_tty_sem,  K_FOREVER);

	LOG_INF("OpenAMP[remote] Linux TTY responder started");

	tty_ept.priv = &tty_msg;
	ret = rpmsg_create_ept(&tty_ept, rpdev, "rpmsg-tty",
			       RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
			       rpmsg_recv_tty_callback, NULL);
	if (ret) {
		LOG_ERR("[Linux TTY] Could not create endpoint: %d", ret);
		goto task_end;
	}

	while (tty_ept.addr !=  RPMSG_ADDR_ANY) {
		k_sem_take(&data_tty_sem,  K_FOREVER);
		if (tty_msg.len) {
			LOG_INF("[Linux TTY] incoming msg: %.*s",
				(int)tty_msg.len, (char *)tty_msg.data);
			snprintf(tx_buff, 13, "TTY 0x%04x: ", tty_ept.addr);
			memcpy(&tx_buff[12], tty_msg.data, tty_msg.len);
			rpmsg_send(&tty_ept, tx_buff, tty_msg.len + 12);
			rpmsg_release_rx_buffer(&tty_ept, tty_msg.data);
		}
		tty_msg.len = 0;
		tty_msg.data = NULL;
	}
	rpmsg_destroy_ept(&tty_ept);

task_end:
	LOG_INF("OpenAMP Linux TTY responder ended");
}

/* Audio processing functions */
static void process_audio_block(const struct audio_block_header *header, const uint8_t *block_data)
{
	LOG_DBG("Processing audio block: chunk_id=%u, block=%u/%u, size=%u", 
		header->chunk_id, header->block_number, header->total_blocks, header->block_size);

	/* Check if this is a new chunk */
	if (current_chunk.chunk_id != header->chunk_id) {
		/* Reset current chunk for new chunk */
		current_chunk.chunk_id = header->chunk_id;
		current_chunk.received_blocks = 0;
		current_chunk.total_blocks = header->total_blocks;
		current_chunk.is_complete = false;
		current_chunk.is_ready = false;
		memset(current_chunk.data, 0, sizeof(current_chunk.data));
		LOG_INF("Starting new audio chunk %u with %u blocks", 
			header->chunk_id, header->total_blocks);
	}

	/* Validate block parameters */
	if (header->block_number >= header->total_blocks) {
		LOG_ERR("Invalid block number %u (total: %u)", 
			header->block_number, header->total_blocks);
		return;
	}

	if (header->block_size > AUDIO_BLOCK_SIZE) {
		LOG_ERR("Block size %u exceeds maximum %u", 
			header->block_size, AUDIO_BLOCK_SIZE);
		return;
	}

	/* Calculate offset and copy data */
	uint32_t offset = header->block_number * AUDIO_BLOCK_SIZE;
	if (offset + header->block_size > AUDIO_CHUNK_SIZE) {
		LOG_ERR("Block data would exceed chunk buffer");
		return;
	}

	memcpy(&current_chunk.data[offset], block_data, header->block_size);
	current_chunk.received_blocks++;

	LOG_DBG("Block %u copied to offset %u, received %u/%u blocks", 
		header->block_number, offset, current_chunk.received_blocks, current_chunk.total_blocks);

	/* Check if chunk is complete */
	if (current_chunk.received_blocks >= current_chunk.total_blocks) {
		current_chunk.is_complete = true;
		current_chunk.is_ready = true;
		
		/* Copy to ready buffer and signal application */
		memcpy(&ready_chunk, &current_chunk, sizeof(struct audio_chunk));
		k_sem_give(&audio_chunk_ready_sem);
		
		LOG_INF("Audio chunk %u complete (%u blocks, %u bytes total)", 
			current_chunk.chunk_id, current_chunk.received_blocks,
			current_chunk.received_blocks * AUDIO_BLOCK_SIZE);
	}
}

/* Application interface to get completed audio chunks */
int get_audio_chunk(uint8_t **chunk_data, uint32_t *chunk_size, uint32_t timeout_ms)
{
	k_timeout_t timeout = timeout_ms == 0 ? K_NO_WAIT : K_MSEC(timeout_ms);
	
	if (k_sem_take(&audio_chunk_ready_sem, timeout) == 0) {
		*chunk_data = ready_chunk.data;
		*chunk_size = ready_chunk.total_blocks * AUDIO_BLOCK_SIZE;
		LOG_INF("Providing audio chunk %u to application (%u bytes)", 
			ready_chunk.chunk_id, *chunk_size);
		return 0;
	}
	return -1;  /* Timeout or no data */
}

void app_rpmsg_audio(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	int ret = 0;

	k_sem_take(&data_audio_sem, K_FOREVER);

	LOG_INF("OpenAMP[remote] Audio processor started");

	/* Initialize audio structures */
	memset(&current_chunk, 0, sizeof(current_chunk));
	memset(&ready_chunk, 0, sizeof(ready_chunk));

	audio_ept.priv = &audio_msg;
	ret = rpmsg_create_ept(&audio_ept, rpdev, "rpmsg-audio",
			       RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
			       rpmsg_recv_audio_callback, NULL);
	if (ret) {
		LOG_ERR("[Audio] Could not create endpoint: %d", ret);
		goto task_end;
	}

	LOG_INF("Audio endpoint created, waiting for audio blocks...");

	while (audio_ept.addr != RPMSG_ADDR_ANY) {
		k_sem_take(&data_audio_sem, K_FOREVER);
		
		if (audio_msg.len > 0) {
			/* Validate minimum message size (header + some data) */
			if (audio_msg.len < sizeof(struct audio_block_header)) {
				LOG_ERR("Audio message too small: %zu bytes", audio_msg.len);
				goto release_buffer;
			}

			/* Extract header and data */
			struct audio_block_header *header = (struct audio_block_header *)audio_msg.data;
			uint8_t *block_data = (uint8_t *)audio_msg.data + sizeof(struct audio_block_header);
			size_t expected_size = sizeof(struct audio_block_header) + header->block_size;

			/* Validate message size */
			if (audio_msg.len != expected_size) {
				LOG_ERR("Audio message size mismatch: got %zu, expected %zu", 
					audio_msg.len, expected_size);
				goto release_buffer;
			}

			/* Process the audio block */
			process_audio_block(header, block_data);

release_buffer:
			rpmsg_release_rx_buffer(&audio_ept, audio_msg.data);
		}
		
		audio_msg.len = 0;
		audio_msg.data = NULL;
	}
	
	rpmsg_destroy_ept(&audio_ept);

task_end:
	LOG_INF("OpenAMP Audio processor ended");
}

/* Example application that consumes audio chunks */
void audio_application_task(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	uint8_t *chunk_data;
	uint32_t chunk_size;

	LOG_INF("Audio application started, waiting for audio chunks...");

	while (1) {
		/* Wait for a complete 1000ms audio chunk */
		if (get_audio_chunk(&chunk_data, &chunk_size, 5000) == 0) {
			/* Process the complete 1000ms audio chunk here */
			LOG_INF("Application received %u bytes of audio data", chunk_size);
			
			/* TODO: Feed to micro-speech model */
			
			
			/* Simulate processing time */
			k_msleep(10);
		} else {
			LOG_WRN("Audio application: timeout waiting for chunk");
		}
	}
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
	k_sem_give(&data_sc_sem);
	k_sem_give(&data_tty_sem);
	k_sem_give(&data_audio_sem);

	while (1) {
		receive_message(&msg, &len);
	}

task_end:
	cleanup_system();

	LOG_INF("OpenAMP demo ended");
}

int main(void)
{
	LOG_INF("Starting application threads!");
	k_thread_create(&thread_mng_data, thread_mng_stack, APP_TASK_STACK_SIZE,
			rpmsg_mng_task,
			NULL, NULL, NULL, K_PRIO_COOP(8), 0, K_NO_WAIT);
	k_thread_create(&thread_rp__client_data, thread_rp__client_stack, APP_TASK_STACK_SIZE,
			app_rpmsg_client_sample,
			NULL, NULL, NULL, K_PRIO_COOP(7), 0, K_NO_WAIT);
	k_thread_create(&thread_tty_data, thread_tty_stack, APP_TTY_TASK_STACK_SIZE,
			app_rpmsg_tty,
			NULL, NULL, NULL, K_PRIO_COOP(7), 0, K_NO_WAIT);
	k_thread_create(&thread_audio_data, thread_audio_stack, APP_AUDIO_TASK_STACK_SIZE,
			app_rpmsg_audio,
			NULL, NULL, NULL, K_PRIO_COOP(6), 0, K_NO_WAIT);
	k_thread_create(&thread_audio_app_data, thread_audio_app_stack, APP_AUDIO_TASK_STACK_SIZE,
			audio_application_task,
			NULL, NULL, NULL, K_PRIO_COOP(5), 0, K_NO_WAIT);
	return 0;
}

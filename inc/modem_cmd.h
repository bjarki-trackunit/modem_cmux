/*
 * Copyright (c) 2022 Trackunit Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/device.h>
#include <zephyr/sys/ring_buffer.h>

#include "modem_pipe.h"

#ifndef ZEPHYR_DRIVERS_MODEM_MODEM_CMD
#define ZEPHYR_DRIVERS_MODEM_MODEM_CMD

struct modem_cmd;

/**
 * @brief Process work item
 * @note k_work struct must be placed first
*/
struct modem_cmd_process_item {
	struct k_work_delayable dwork;
	struct modem_cmd *cmd;
};

typedef void (*modem_cmd_received_t)(const uint8_t *buf, uint16_t size, void *user_data);

struct modem_cmd {
	struct modem_pipe *pipe;
	modem_cmd_received_t cmd_received;
	void *user_data;
	uint8_t *receive_buf;
	uint16_t receive_buf_size;
	uint16_t receive_buf_cnt;
	uint8_t *delimiter;
	uint16_t delimiter_size;
	k_timeout_t idle_timeout;
	struct modem_cmd_process_item process;
};

struct modem_cmd_config {
	modem_cmd_received_t cmd_received;
	void *user_data;
	uint8_t *receive_buf;
	uint16_t receive_buf_size;
	uint8_t *delimiter;
	uint16_t delimiter_size;
	k_timeout_t idle_timeout;
};

/**
 * @brief Initialize modem pipe command handler
 * @note Command handler must be attached to pipe
 */
int modem_cmd_init(struct modem_cmd *cmd, const struct modem_cmd_config *config);

/**
 * @brief Attach modem command handler to pipe
 * @note Command handler is now enabled
 */
int modem_cmd_attach(struct modem_cmd *cmd, struct modem_pipe *pipe);

/**
 * @brief Release pipe from command handler
 * @note Command handler is now disabled
 */
int modem_cmd_release(struct modem_cmd *cmd);

#endif /* ZEPHYR_DRIVERS_MODEM_MODEM_PIPE_CMD */

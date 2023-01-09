/*
 * Copyright (c) 2022 Trackunit Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(modem_cmd);

#include <zephyr/kernel.h>
#include <string.h>

#include "modem_cmd.h"

/*
 * Return -EINVAL if delimiter is not found
 * Return lenght of line if delimiter found
 */
int modem_cmd_process_find_delimiter(struct modem_cmd *cmd)
{
	/* Verify delimiter fits */
	if (cmd->receive_buf_cnt < cmd->delimiter_size) {
		return -1;
	}

	uint16_t cnt = cmd->receive_buf_cnt - cmd->delimiter_size + 1;

	for (uint16_t i = 0; i < cnt; i++) {
		if (memcmp(&cmd->receive_buf[i], cmd->delimiter, cmd->delimiter_size) == 0) {
			return (int)(i + cmd->delimiter_size);
		}
	}

	return -1;
}

void modem_cmd_process_line(struct modem_cmd *cmd, uint16_t line_cnt)
{
	cmd->cmd_received(cmd->receive_buf, line_cnt, cmd->user_data);
}

void modem_cmd_process_discard_line(struct modem_cmd *cmd, uint16_t line_cnt)
{
	uint16_t keep_cnt = cmd->receive_buf_cnt - line_cnt;
	uint16_t keep_start = cmd->receive_buf_cnt - keep_cnt;

	for (uint16_t i = 0; i < keep_cnt; i++) {
		cmd->receive_buf[i] = cmd->receive_buf[keep_start + i];
	}

	cmd->receive_buf_cnt = keep_cnt;
}

void modem_cmd_process(struct k_work *item)
{
	struct modem_cmd_process_item *process = (struct modem_cmd_process_item *)item;
	struct modem_cmd *cmd = process->cmd;
	int ret;

	/* Receive data from pipe */
	ret = modem_pipe_receive(cmd->pipe, &cmd->receive_buf[cmd->receive_buf_cnt],
				 (cmd->receive_buf_size - cmd->receive_buf_cnt));

	/* Verify data received */
	if (ret == 0) {
		return;
	}

	/* Advance buffer count */
	cmd->receive_buf_cnt += (uint16_t)ret;

	while (1) {
		/* Check for end delimiter */
		ret = modem_cmd_process_find_delimiter(cmd);

		/* Reset receive buffer if overflow */
		if ((ret < 0) && (cmd->receive_buf_cnt == cmd->receive_buf_size)) {
			cmd->receive_buf_cnt = 0;
			return;
		}

		/* Verify end delimiter found */
		if (ret < 0) {
			return;
		}

		/* Process line, invoking on received event */
		modem_cmd_process_line(cmd, (uint16_t)ret);

		/* Discard processed line */
		modem_cmd_process_discard_line(cmd, (uint16_t)ret);
	}
}

void modem_cmd_pipe_event_handler(struct modem_pipe *pipe, enum modem_pipe_event event,
				  void *user_data)
{
	struct modem_cmd *cmd = (struct modem_cmd *)user_data;

	k_work_reschedule(&cmd->process.dwork, cmd->idle_timeout);
}

int modem_cmd_init(struct modem_cmd *cmd, const struct modem_cmd_config *config)
{
	/* Validate arguments */
	if ((cmd == NULL) || (config == NULL)) {
		return -EINVAL;
	}

	/* Validate config */
	if ((config->cmd_received == NULL) ||
	    (config->receive_buf == NULL) ||
	    (config->receive_buf_size == 0) ||
	    (config->delimiter == NULL) ||
	    (config->delimiter_size == 0)) {
		return -EINVAL;
	}

	memset(cmd, 0x00, sizeof(*cmd));

	/* Configure command handler */
	cmd->pipe = NULL;
	cmd->cmd_received = config->cmd_received;
	cmd->user_data = config->user_data;
	cmd->receive_buf = config->receive_buf;
	cmd->receive_buf_size = config->receive_buf_size;
	cmd->delimiter = config->delimiter;
	cmd->delimiter_size = config->delimiter_size;
	cmd->idle_timeout = config->idle_timeout;

	/* Initialize work item */
	cmd->process.cmd = cmd;
	k_work_init_delayable(&cmd->process.dwork, modem_cmd_process);

	return 0;
}

int modem_cmd_attach(struct modem_cmd *cmd, struct modem_pipe *pipe)
{
	/* Validate arguments */
	if ((cmd == NULL) || (pipe == NULL)) {
		return -EINVAL;
	}

	/* Associate command handler with pipe */
	cmd->pipe = pipe;

	/* Set pipe event handler */
	return modem_pipe_event_handler_set(pipe, modem_cmd_pipe_event_handler, cmd);
}

int modem_cmd_release(struct modem_cmd *cmd)
{
	if (cmd->pipe == NULL) {
		return 0;
	}

	modem_pipe_event_handler_set(cmd->pipe, NULL, NULL);

	cmd->pipe = NULL;

	k_work_cancel_delayable(&cmd->process.dwork);

	return 0;
}

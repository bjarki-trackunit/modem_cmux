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

static void modem_cmd_parse_reset(struct modem_cmd *cmd)
{
	cmd->receive_buf_len = 0;
	cmd->delimiter_match_cnt = 0;
	cmd->argc = 0;
	cmd->parse_match = NULL;

	for (uint16_t i = 0; i < cmd->matches_size; i++) {
		cmd->matches[i].matching = true;
	}
}

/* Exact match is stored at end of receive buffer */
static void modem_cmd_parse_save_match(struct modem_cmd *cmd)
{
	uint8_t *argv;

	/* Store length of match including NULL to avoid overwriting it if buffer overruns */
	cmd->parse_match_len = cmd->receive_buf_len + 1;

	/* Copy match to end of receive buffer */
	argv = &cmd->receive_buf[cmd->receive_buf_size - cmd->parse_match_len];

	/* Copy match to end of receive buffer (excluding NULL) */
	memcpy(argv, &cmd->receive_buf[0], cmd->parse_match_len - 1);

	/* Save match */
	cmd->argv[cmd->argc] = argv;

	/* Terminate match */
	cmd->receive_buf[cmd->receive_buf_size - 1] = '\0';

	/* Increment argument count */
	cmd->argc++;
}

static bool modem_cmd_parse_find_match(struct modem_cmd *cmd)
{
	/* For each match config */
	for (uint16_t i = 0; i < cmd->matches_size; i++) {
		/* Validate match config is matching previous bytes */
		if (cmd->matches[i].matching == false) {
			continue;
		}

		/* Validate receive length does not exceed match length */
		if (cmd->matches[i].match_size < cmd->receive_buf_len) {
			cmd->matches[i].matching = false;

			continue;
		}

		/* Validate match config also matches new byte */
		if ((cmd->matches[i].match[cmd->receive_buf_len - 1]) !=
		    (cmd->receive_buf[cmd->receive_buf_len - 1])) {
			/* Validate wildcards accepted */
			if (cmd->matches[i].wildcards == false) {
				cmd->matches[i].matching = false;

				continue;
			}

			/* Validate wildcard */
			if (cmd->matches[i].match[cmd->receive_buf_len - 1] != '?') {
				cmd->matches[i].matching = false;

				continue;
			}
		}

		/* Validate match is complete */
		if ((cmd->receive_buf_len) < cmd->matches[i].match_size) {
			continue;
		}

		/* Complete match found */
		cmd->parse_match = &cmd->matches[i];

		return true;
	}

	return false;
}

static bool modem_cmd_parse_is_separator(struct modem_cmd *cmd)
{
	for (uint16_t i = 0; i < cmd->parse_match->separators_size; i++) {
		if ((cmd->parse_match->separators[i]) ==
		    (cmd->receive_buf[cmd->receive_buf_len - 1])) {
			return true;
		}
	}

	return false;
}

static bool modem_cmd_parse_end_del_start(struct modem_cmd *cmd)
{
	for (uint16_t i = 0; i < cmd->delimiter_size; i++) {
		if (cmd->receive_buf[cmd->receive_buf_len - 1] == cmd->delimiter[i]) {
			return true;
		}
	}

	return false;
}

static bool modem_cmd_parse_end_del_complete(struct modem_cmd *cmd)
{
	/* Validate length of end delimiter */
	if (cmd->receive_buf_len < cmd->delimiter_size) {
		return false;
	}

	/* Compare end delimiter with receive buffer content */
	return (memcmp(&cmd->receive_buf[cmd->receive_buf_len - cmd->delimiter_size],
		       cmd->delimiter, cmd->delimiter_size) == 0) ? true : false;
}

/* Process byte */
void modem_cmd_process_byte(struct modem_cmd *cmd, uint8_t byte)
{
	/* Validate receive buffer not overrun */
	if (cmd->receive_buf_size == cmd->receive_buf_len) {
		modem_cmd_parse_reset(cmd);

		return;
	}

	/* Copy byte to receive buffer */
	cmd->receive_buf[cmd->receive_buf_len] = byte;
	cmd->receive_buf_len++;

	/* Validate end delimiter not complete */
	if (modem_cmd_parse_end_del_complete(cmd) == true) {
		/* Check if match exists */
		if (cmd->parse_match == NULL) {
			/* Reset parser */
			modem_cmd_parse_reset(cmd);

			return;
		}

		/* Check if trailing argument exists */
		if (cmd->parse_arg_len > 0) {
			cmd->argv[cmd->argc] = &cmd->receive_buf[cmd->receive_buf_len - cmd->delimiter_size - cmd->parse_arg_len];
			cmd->receive_buf[cmd->receive_buf_len - cmd->delimiter_size] = '\0';
			cmd->argc++;
		}

		/* Callback */
		cmd->parse_match->callback(cmd, (char **)cmd->argv, cmd->argc, cmd->user_data);

		/* Reset parser */
		modem_cmd_parse_reset(cmd);

		return;
	}

	/* Validate end delimiter not started */
	if (modem_cmd_parse_end_del_start(cmd) == true) {

		return;
	}

	/* Find matching command if missing */
	if (cmd->parse_match == NULL) {
		/* Find matching command */
		if (modem_cmd_parse_find_match(cmd) == false) {
			return;
		}

		/* Save match */
		modem_cmd_parse_save_match(cmd);

		/* Prepare argument parser */
		cmd->parse_arg_len = 0;

		return;
	}

	/* Check if separator reached */
	if (modem_cmd_parse_is_separator(cmd) == true) {
		/* Check if argument is empty */
		if (cmd->parse_arg_len == 0) {
			/* Save empty argument */
			cmd->argv[cmd->argc] = "";
		} else {
			/* Save pointer to start of argument */
			cmd->argv[cmd->argc] = &cmd->receive_buf[cmd->receive_buf_len - cmd->parse_arg_len - 1];

			/* Replace separator with string terminator */
			cmd->receive_buf[cmd->receive_buf_len - 1] = '\0';
		}

		/* Increment argument count */
		cmd->argc++;

		/* Reset parse argument length */
		cmd->parse_arg_len = 0;

		return;
	}

	/* Increment argument length */
	cmd->parse_arg_len++;
}

/* Receive chunk of bytes */
static bool modem_cmd_receive_bytes(struct modem_cmd *cmd)
{
	int ret;

	ret =  modem_pipe_receive(cmd->pipe, cmd->work_buf, sizeof(cmd->work_buf));

	if (ret < 1) {
		return false;
	}

	cmd->work_buf_len = (uint16_t)ret;

	return true;
}

/* Process chunk of received bytes */
void modem_cmd_process_bytes(struct modem_cmd *cmd)
{
	for (uint16_t i = 0; i < cmd->work_buf_len; i++) {
		modem_cmd_process_byte(cmd, cmd->work_buf[i]);
	}
}

void modem_cmd_process(struct k_work *item)
{
	struct modem_cmd_process_item *process = (struct modem_cmd_process_item *)item;
	struct modem_cmd *cmd = process->cmd;

	/* Receive all available data */
	while (modem_cmd_receive_bytes(cmd) == true) {
		/* Process all available data */
		modem_cmd_process_bytes(cmd);
	}
}

void modem_cmd_pipe_event_handler(struct modem_pipe *pipe, enum modem_pipe_event event,
				  void *user_data)
{
	struct modem_cmd *cmd = (struct modem_cmd *)user_data;

	k_work_reschedule(&cmd->process.dwork, cmd->process_timeout);
}

int modem_cmd_init(struct modem_cmd *cmd, const struct modem_cmd_config *config)
{
	/* Validate arguments */
	if ((cmd == NULL) || (config == NULL)) {
		return -EINVAL;
	}

	/* Validate config */
	if ((config->receive_buf == NULL) ||
	    (config->receive_buf_size == 0) ||
	    (config->argv == NULL) ||
	    (config->argv_size == 0) ||
	    (config->delimiter == NULL) ||
	    (config->delimiter_size == 0) ||
	    (config->matches == NULL) ||
	    (config->matches_size == 0)) {
		return -EINVAL;
	}

	memset(cmd, 0x00, sizeof(*cmd));

	/* Configure command handler */
	cmd->pipe = NULL;
	cmd->user_data = config->user_data;
	cmd->receive_buf = config->receive_buf;
	cmd->receive_buf_size = config->receive_buf_size;
	cmd->argv = config->argv;
	cmd->argv_size = config->argv_size;
	cmd->delimiter = config->delimiter;
	cmd->delimiter_size = config->delimiter_size;
	cmd->matches = config->matches;
	cmd->matches_size = config->matches_size;
	cmd->process_timeout = config->process_timeout;

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

	/* Reset parser */
	modem_cmd_parse_reset(cmd);

	/* Set pipe event handler */
	return modem_pipe_event_handler_set(pipe, modem_cmd_pipe_event_handler, cmd);
}

int modem_cmd_send(struct modem_cmd *cmd, const char *str)
{
	int ret;
	int sent;

	/* Validate arguments */
	if ((cmd == NULL) || (str == NULL)) {
		return -EINVAL;
	}

	/* Validate pipe attached */
	if (cmd->pipe == NULL) {
		return -EPERM;
	}

	ret = modem_pipe_transmit(cmd->pipe, str, strlen(str));

	if (ret < 0) {
		return ret;
	}

	sent = ret;

	ret = modem_pipe_transmit(cmd->pipe, cmd->delimiter, cmd->delimiter_size);

	if (ret < 0) {
		return ret;
	}

	return sent + ret;
}

uint32_t modem_cmd_send_sync_event(struct modem_cmd *cmd, const char *str, struct k_event *event,
			      uint32_t events, k_timeout_t timeout)
{
	int ret;

	/* Validate arguments */
	if ((cmd == NULL) || (str == NULL) || (event == NULL) || events == 0) {
		return -EINVAL;
	}

	/* Clear events */
	k_event_clear(event, events);

	/* Send command */
	ret = modem_cmd_send(cmd, str);

	if (ret < 0) {
		return ret;
	}

	/* Await events or timeout */
	return k_event_wait(event, events, false, timeout);
}

int modem_cmd_release(struct modem_cmd *cmd)
{
	if (cmd->pipe == NULL) {
		return 0;
	}

	modem_pipe_event_handler_set(cmd->pipe, NULL, NULL);

	cmd->pipe = NULL;

	struct k_work_sync sync;
	k_work_cancel_delayable_sync(&cmd->process.dwork, &sync);

	return 0;
}

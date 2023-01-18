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

#define MODEM_CMD_MATCHES_INDEX_RESPONSE        (0)
#define MODEM_CMD_MATCHES_INDEX_ABORT           (1)
#define MODEM_CMD_MATCHES_INDEX_UNSOL           (2)

#define MODEM_CMD_SCRIPT_STATUS_RUNNING_BIT     (0)

#define MODEM_CMD_EVENT_SCRIPT_ABORTED          (BIT(0))
#define MODEM_CMD_EVENT_SCRIPT_STOPPED          (BIT(1))

#define MODEM_CMD_SCRIPT_ABORT_TIMEOUT          (K_MSEC(500))

static void modem_cmd_script_stop(struct modem_cmd *cmd)
{
	uint32_t events;

	/* Save script event to post */
	events = (cmd->script_cmd_it == cmd->script->script_cmds_size) ?
		MODEM_CMD_EVENT_SCRIPT_STOPPED : MODEM_CMD_EVENT_SCRIPT_ABORTED;

	/* Clear script */
	cmd->script = NULL;

	/* Clear response and abort commands */
	cmd->matches[MODEM_CMD_MATCHES_INDEX_ABORT] = NULL;
	cmd->matches_size[MODEM_CMD_MATCHES_INDEX_ABORT] = 0;
	cmd->matches[MODEM_CMD_MATCHES_INDEX_RESPONSE] = NULL;
	cmd->matches_size[MODEM_CMD_MATCHES_INDEX_RESPONSE] = 0;

	/* Post script event */
	k_event_post(&cmd->script_event, events);

	LOG_DBG("");
}

static void modem_cmd_script_next(struct modem_cmd *cmd, bool initial)
{
	/* Advance iterator if not initial */
	if (initial == true) {
		/* Reset iterator */
		cmd->script_cmd_it = 0;
	} else {
		/* Advance iterator */
		cmd->script_cmd_it++;
	}

	/* Check if end of script reached */
	if (cmd->script_cmd_it == cmd->script->script_cmds_size) {
		modem_cmd_script_stop(cmd);

		return;
	}

	/* Update response command handlers */
	cmd->matches[MODEM_CMD_MATCHES_INDEX_RESPONSE] =
		&cmd->script->script_cmds[cmd->script_cmd_it].response_match;

	cmd->matches_size[MODEM_CMD_MATCHES_INDEX_RESPONSE] = 1;

	/* Initialize response match */
	cmd->matches[MODEM_CMD_MATCHES_INDEX_RESPONSE][0].matching = true;

	/* Check if request must be sent */
	if (strlen(cmd->script->script_cmds[cmd->script_cmd_it].request) == 0) {
		return;
	}

	/* Send request */
	modem_cmd_send(cmd, cmd->script->script_cmds[cmd->script_cmd_it].request);}

static void modem_cmd_script_start(struct modem_cmd *cmd, const struct modem_cmd_script *script)
{
	/* Save script */
	cmd->script = script;

	/* Set abort matches */
	cmd->matches[MODEM_CMD_MATCHES_INDEX_ABORT] = script->abort_matches;
	cmd->matches_size[MODEM_CMD_MATCHES_INDEX_ABORT] = script->abort_matches_size;

	LOG_DBG("");

	/* Set first script command */
	modem_cmd_script_next(cmd, true);
}

static void modem_cmd_script_run_handler(struct k_work *item)
{
	struct modem_cmd_script_run_work_item *script_run_work =
		(struct modem_cmd_script_run_work_item *)item;

	struct modem_cmd *cmd = script_run_work->cmd;
	const struct modem_cmd_script *script = script_run_work->script;

	/* Start script */
	modem_cmd_script_start(cmd, script);
}

static void modem_cmd_script_abort_handler(struct k_work *item)
{
	struct modem_cmd_script_abort_work_item *script_abort_work =
		(struct modem_cmd_script_abort_work_item *)item;

	struct modem_cmd *cmd = script_abort_work->cmd;

	/* Validate script is currently running */
	if (cmd->script == NULL) {
		return;
	}

	/* Abort script */
	modem_cmd_script_stop(cmd);
}

static void modem_cmd_parse_reset(struct modem_cmd *cmd)
{
	/* Reset parameters used for parsing */
	cmd->receive_buf_len = 0;
	cmd->delimiter_match_size = 0;
	cmd->argc = 0;
	cmd->parse_match = NULL;

	/* Reset matches matching state */
	for (uint16_t i = 0; i < ARRAY_SIZE(cmd->matches); i++) {
		for (uint16_t u = 0; u < cmd->matches_size[i]; u++) {
			cmd->matches[i][u].matching = true;
		}
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
	/* Find in all matches types */
	for (uint16_t i = 0; i < ARRAY_SIZE(cmd->matches); i++) {
		/* Find in all matches of matches type */
		for (uint16_t u = 0; u < cmd->matches_size[i]; u++) {
			/* Validate match config is matching previous bytes */
			if (cmd->matches[i][u].matching == false) {
				continue;
			}

			/* Validate receive length does not exceed match length */
			if (cmd->matches[i][u].match_size < cmd->receive_buf_len) {
				cmd->matches[i][u].matching = false;

				continue;
			}

			/* Validate match config also matches new byte */
			if ((cmd->matches[i][u].match[cmd->receive_buf_len - 1]) !=
			(cmd->receive_buf[cmd->receive_buf_len - 1])) {
				/* Validate wildcards accepted */
				if (cmd->matches[i][u].wildcards == false) {
					cmd->matches[i][u].matching = false;

					continue;
				}

				/* Validate wildcard */
				if (cmd->matches[i][u].match[cmd->receive_buf_len - 1] != '?') {
					cmd->matches[i][u].matching = false;

					continue;
				}
			}

			/* Validate match is complete */
			if ((cmd->receive_buf_len) < cmd->matches[i][u].match_size) {
				continue;
			}

			/* Complete match found */
			cmd->parse_match = &cmd->matches[i][u];
			cmd->parse_match_type = i;

			return true;
		}
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
		       cmd->delimiter, cmd->delimiter_size) == 0)
		       ? true
		       : false;
}

/* Receive chunk of bytes */
static bool modem_cmd_receive_bytes(struct modem_cmd *cmd)
{
	int ret;

	ret = modem_pipe_receive(cmd->pipe, cmd->work_buf, sizeof(cmd->work_buf));

	if (ret < 1) {
		return false;
	}

	cmd->work_buf_len = (uint16_t)ret;

	return true;
}

static void modem_cmd_on_command_received_log(struct modem_cmd *cmd)
{
	/* Log entire line if command is match all */
	if ((cmd->parse_match->match_size == 0) && (cmd->argc > 1)) {
		LOG_DBG("%s", cmd->argv[1]);

		return;
	}

	/* Only log match */
	LOG_DBG("%s", cmd->argv[0]);
}

static void modem_cmd_on_command_received_unsol(struct modem_cmd *cmd)
{
	/* Callback */
	if (cmd->parse_match->callback != NULL) {
		cmd->parse_match->callback(cmd, (char **)cmd->argv, cmd->argc, cmd->user_data);
	}
}

static void modem_cmd_on_command_received_abort(struct modem_cmd *cmd)
{
	/* Callback */
	if (cmd->parse_match->callback != NULL) {
		cmd->parse_match->callback(cmd, (char **)cmd->argv, cmd->argc, cmd->user_data);
	}

	/* Abort script */
	modem_cmd_script_stop(cmd);
}

static void modem_cmd_on_command_received_resp(struct modem_cmd *cmd)
{
	/* Callback */
	if (cmd->parse_match->callback != NULL) {
		cmd->parse_match->callback(cmd, (char **)cmd->argv, cmd->argc, cmd->user_data);
	}

	/* Advance script */
	modem_cmd_script_next(cmd, false);
}

static bool modem_cmd_parse_find_catch_all_match(struct modem_cmd *cmd)
{
	/* Find in all matches types */
	for (uint16_t i = 0; i < ARRAY_SIZE(cmd->matches); i++) {
		/* Find in all matches of matches type */
		for (uint16_t u = 0; u < cmd->matches_size[i]; u++) {
			/* Validate match config is matching previous bytes */
			if (cmd->matches[i][u].match_size == 0) {
				cmd->parse_match = &cmd->matches[i][u];
				cmd->parse_match_type = i;

				return true;
			}
		}
	}

	return false;
}

static void modem_cmd_on_command_received(struct modem_cmd *cmd)
{
	modem_cmd_on_command_received_log(cmd);

	switch (cmd->parse_match_type)
	{
	case MODEM_CMD_MATCHES_INDEX_UNSOL:
		modem_cmd_on_command_received_unsol(cmd);
		break;

	case MODEM_CMD_MATCHES_INDEX_ABORT:
		modem_cmd_on_command_received_abort(cmd);
		break;

	case MODEM_CMD_MATCHES_INDEX_RESPONSE:
		modem_cmd_on_command_received_resp(cmd);
		break;
	}

}

static void modem_cmd_on_unknown_command_received(struct modem_cmd *cmd)
{
	/* Try to find catch all match */
	if (modem_cmd_parse_find_catch_all_match(cmd) == false) {
		LOG_DBG("%.*s", cmd->receive_buf_len, cmd->receive_buf);

		return;
	}

	/* Terminate received command */
	cmd->receive_buf[cmd->receive_buf_len - cmd->delimiter_match_size] = '\0';

	/* Parse command */
	cmd->argv[0] = "";
	cmd->argv[1] = cmd->receive_buf;
	cmd->argc = 2;

	/* Invoke on response received */
	modem_cmd_on_command_received(cmd);
}

static void modem_cmd_process_byte(struct modem_cmd *cmd, uint8_t byte)
{
	/* Validate receive buffer not overrun */
	if (cmd->receive_buf_size == cmd->receive_buf_len) {
		LOG_WRN("Receive buffer overrun");
		modem_cmd_parse_reset(cmd);

		return;
	}

	/* Validate argv buffer not overrun */
	if (cmd->argc == cmd->argv_size) {
		LOG_WRN("Argv buffer overrun");
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
			/* Handle unknown command */
			modem_cmd_on_unknown_command_received(cmd);

			/* Reset parser */
			modem_cmd_parse_reset(cmd);

			return;
		}

		/* Check if trailing argument exists */
		if (cmd->parse_arg_len > 0) {
			cmd->argv[cmd->argc] =
				&cmd->receive_buf[cmd->receive_buf_len - cmd->delimiter_size -
						  cmd->parse_arg_len];
			cmd->receive_buf[cmd->receive_buf_len - cmd->delimiter_size] = '\0';
			cmd->argc++;
		}

		/* Handle received command */
		modem_cmd_on_command_received(cmd);

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
			cmd->argv[cmd->argc] =
				&cmd->receive_buf[cmd->receive_buf_len - cmd->parse_arg_len - 1];

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

/* Process chunk of received bytes */
static void modem_cmd_process_bytes(struct modem_cmd *cmd)
{
	for (uint16_t i = 0; i < cmd->work_buf_len; i++) {
		modem_cmd_process_byte(cmd, cmd->work_buf[i]);
	}
}

static void modem_cmd_process_handler(struct k_work *item)
{
	struct modem_cmd_work_item *process_work = (struct modem_cmd_work_item *)item;
	struct modem_cmd *cmd = process_work->cmd;

	/* Receive all available data */
	while (modem_cmd_receive_bytes(cmd) == true) {
		/* Process all available data */
		modem_cmd_process_bytes(cmd);
	}
}

static void modem_cmd_pipe_event_handler(struct modem_pipe *pipe, enum modem_pipe_event event,
				  void *user_data)
{
	struct modem_cmd *cmd = (struct modem_cmd *)user_data;

	k_work_reschedule(&cmd->process_work.dwork, cmd->process_timeout);
}

/*********************************************************
 * GLOBAL FUNCTIONS
 *********************************************************/
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
	    (config->unsol_matches == NULL) ||
	    (config->unsol_matches_size == 0)) {
		return -EINVAL;
	}

	/* Clear context */
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
	cmd->matches[MODEM_CMD_MATCHES_INDEX_UNSOL] = config->unsol_matches;
	cmd->matches_size[MODEM_CMD_MATCHES_INDEX_UNSOL] = config->unsol_matches_size;
	cmd->process_timeout = config->process_timeout;

	/* Initialize work items */
	cmd->process_work.cmd = cmd;
	k_work_init_delayable(&cmd->process_work.dwork, modem_cmd_process_handler);

	cmd->script_run_work.cmd = cmd;
	k_work_init(&cmd->script_run_work.work, modem_cmd_script_run_handler);

	cmd->script_abort_work.cmd = cmd;
	k_work_init(&cmd->script_abort_work.work, modem_cmd_script_abort_handler);

	/* Initialize script event */
	k_event_init(&cmd->script_event);

	/* Initialize script state */
	atomic_set(&cmd->script_status, 0);

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

int modem_cmd_script_run(struct modem_cmd *cmd, const struct modem_cmd_script *script,
			 k_timeout_t timeout)
{
	uint32_t events;
	int ret;

	/* Validate attached */
	if (cmd->pipe == NULL) {
		return -EPERM;
	}

	/* Validate arguments */
	if ((cmd == NULL) || (script == NULL)) {
		return -EINVAL;
	}

	/* Validate script */
	if ((script->script_cmds == NULL) ||
	    (script->script_cmds_size == 0) ||
	    ((script->abort_matches != NULL) && (script->abort_matches_size == 0))) {
		return -EINVAL;
	}

	/* Validate script is not currently running */
	if (atomic_test_and_set_bit(&cmd->script_status, MODEM_CMD_SCRIPT_STATUS_RUNNING_BIT) == true) {
		return -EBUSY;
	}

	/* Clear script events */
	k_event_clear(&cmd->script_event,
		      MODEM_CMD_EVENT_SCRIPT_ABORTED | MODEM_CMD_EVENT_SCRIPT_STOPPED);

	/* Initialize work item data */
	cmd->script_run_work.script = script;

	/* Submit script run work */
	k_work_submit(&cmd->script_run_work.work);

	/* Wait for script aborted, stopped or timed out */
	events = k_event_wait(&cmd->script_event,
			      MODEM_CMD_EVENT_SCRIPT_ABORTED |
			      MODEM_CMD_EVENT_SCRIPT_STOPPED,
			      false, timeout);

	/* Return script result if timeout did not occur */
	if (events != 0) {
		/* Determine script result */
		ret = (events == MODEM_CMD_EVENT_SCRIPT_STOPPED) ? 0 : -EAGAIN;

		/* Update script status */
		atomic_clear_bit(&cmd->script_status, MODEM_CMD_SCRIPT_STATUS_RUNNING_BIT);

		return ret;
	}

	/* Submit script abort work */
	k_work_submit(&cmd->script_abort_work.work);

	/* Wait for script aborted, stopped or timed out */
	events = k_event_wait(&cmd->script_event,
				MODEM_CMD_EVENT_SCRIPT_ABORTED |
				MODEM_CMD_EVENT_SCRIPT_STOPPED,
				false, MODEM_CMD_SCRIPT_ABORT_TIMEOUT);

	/* Validate script stopped or aborted */
	if (events == 0) {
		LOG_ERR("Script blocked");

		/* Leave script status as running forever */
		return -EBUSY;
	}

	/* Determine script result */
	ret = (events == MODEM_CMD_EVENT_SCRIPT_STOPPED) ? 0 : -EAGAIN;

	/* Update script status */
	atomic_clear_bit(&cmd->script_status, MODEM_CMD_SCRIPT_STATUS_RUNNING_BIT);

	return ret;
}

int modem_cmd_send(struct modem_cmd *cmd, const char *str)
{
	int ret;
	int sent;

	/* Validate arguments */
	if ((cmd == NULL) || (str == NULL)) {
		return -EINVAL;
	}

	/* Validate attached */
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

	LOG_DBG("%s", str);

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
	/* Verify attached */
	if (cmd->pipe == NULL) {
		return 0;
	}

	/* Release pipe */
	modem_pipe_event_handler_set(cmd->pipe, NULL, NULL);

	/* Cancel process work */
	struct k_work_sync sync;
	k_work_cancel_delayable_sync(&cmd->process_work.dwork, &sync);

	cmd->pipe = NULL;

	return 0;
}

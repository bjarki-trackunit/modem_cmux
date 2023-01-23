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
 * @brief Callback called when matching command is received
 *
 * @param cmd Pointer to command handler instance
 * @param argv Pointer to array of parsed arguments
 * @param argc Number of parsed arguments, arg 0 holds the exact match
 * @param user_data Free to use user data set during modem_cmd_init()
 */
typedef void (*modem_cmd_match_callback)(struct modem_cmd *cmd, char **argv, uint16_t argc,
					 void *user_data);

/**
 * @brief Modem command match
 */
struct modem_cmd_match {
	/* Match array */
	const uint8_t *match;
	const uint8_t match_size;

	/* Separators array */
	const uint8_t *separators;
	const uint8_t separators_size;

	/* Set if modem command handler shall use wildcards when matching */
	const bool wildcards;

	/* Type of modem command handler */
	const modem_cmd_match_callback callback;

	/* Used by matching algorithm */
	bool matching;
};

#define MODEM_CMD_MATCH(_match, _separators, _callback)                 \
	{                                                               \
		.match = (uint8_t *)(_match),                           \
		.match_size = (uint8_t)(sizeof(_match) - 1),            \
		.separators = (uint8_t *)(_separators),                 \
		.separators_size = (uint8_t)(sizeof(_separators) - 1),  \
		.wildcards = false,                                     \
		.callback = _callback,                                  \
		.matching = false,                                      \
	}

#define MODEM_CMD_MATCH_WILDCARD(_match, _separators, _callback)        \
	{                                                               \
		.match = (uint8_t *)(_match),                           \
		.match_size = (uint8_t)(sizeof(_match) - 1),            \
		.separators = (uint8_t *)(_separators),                 \
		.separators_size = (uint8_t)(sizeof(_separators) - 1),  \
		.wildcards = true,                                      \
		.callback = _callback,                                  \
		.matching = false,                                      \
	}

/**
 * @brief Modem command script command
 *
 * @param request Request to send to modem formatted as char string
 * @param response Expected response to request
 */
struct modem_cmd_script_cmd {
	const char *request;
	struct modem_cmd_match response_match;
};

#define MODEM_CMD_SCRIPT_CMD(_request, _response_match)                 \
	{                                                               \
		.request = _request,                                    \
		.response_match = _response_match,             	        \
	}

/**
 * @brief Modem command script
 *
 * @param name Name of script
 * @param name_size Size of name of script
 * @param script_cmds Array of script commands
 * @param script_cmds_size Elements in array of script commands
 * @param abort_matches Array of abort matches
 * @param abort_matches_size Elements in array of abort matches
 *
 * @example
 *
 * static script1_cmds = {
 *         MODEM_CMD_SCRIPT_CMD("AT", MODEM_CMD_MATCH("OK", "", NULL)),
 *         MODEM_CMD_SCRIPT_CMD("ATI", MODEM_CMD_MATCH("OK", "", NULL)),
 * };
 *
 * static script1_abort_matches = {
 *         MODEM_CMD_MATCH("ERROR", ",", on_error),
 *         MODEM_CMD_MATCH("NO CARRIER", "", on_no_carrier),
 * };
 *
 * static MODEM_CMD_SCRIPT_DEFINE(script1, script1_cmds, script1_abort_matches);
 */
struct modem_cmd_script {
	const char *name;
	struct modem_cmd_script_cmd *const script_cmds;
	const uint16_t script_cmds_size;
	struct modem_cmd_match *const abort_matches;
	const uint16_t abort_matches_size;
};

/* DEPRECATED */
#define MODEM_CMD_SCRIPT(_script_cmds, _abort_matches)                  \
	{                                                               \
		.name = "",                                             \
		.script_cmds = _script_cmds,                            \
		.script_cmds_size = ARRAY_SIZE(_script_cmds),           \
		.abort_matches = _abort_matches,                        \
		.abort_matches_size = ARRAY_SIZE(_abort_matches),       \
	}

#define MODEM_CMD_SCRIPT_DEFINE(_sym, _script_cmds, _abort_matches)     \
	struct modem_cmd_script _sym = {                                \
		.name = #_sym,                                          \
		.script_cmds = _script_cmds,                            \
		.script_cmds_size = ARRAY_SIZE(_script_cmds),           \
		.abort_matches = _abort_matches,                        \
		.abort_matches_size = ARRAY_SIZE(_abort_matches),       \
	}

/**
 * @brief Process work item
 *
 * @note k_work struct must be placed first
 */
struct modem_cmd_work_item {
	struct k_work_delayable dwork;
	struct modem_cmd *cmd;
};

/**
 * @brief Script run work item
 *
 * @param work Work item
 * @param cmd Modem command instance
 * @param script Pointer to new script to run
 *
 * @note k_work struct must be placed first
 */
struct modem_cmd_script_run_work_item {
	struct k_work work;
	struct modem_cmd *cmd;
	const struct modem_cmd_script *script;
};

/**
 * @brief Script abort work item
 *
 * @param work Work item
 * @param cmd Modem command instance
 *
 * @note k_work struct must be placed first
 */
struct modem_cmd_script_abort_work_item {
	struct k_work work;
	struct modem_cmd *cmd;
};

/**
 * @brief Modem command internal context
 * @warning Do not mopdify any members of this struct directly
 */
struct modem_cmd {
	/* Pipe used to send and receive data */
	struct modem_pipe *pipe;

	/* User data passed with match callbacks */
	void *user_data;

	/* Receive buffer */
	uint8_t *receive_buf;
	uint16_t receive_buf_size;
	uint16_t receive_buf_len;

	/* Work buffer */
	uint8_t work_buf[32];
	uint16_t work_buf_len;

	/* Command delimiter */
	uint8_t *delimiter;
	uint16_t delimiter_size;
	uint16_t delimiter_match_len;

	/* Parsed arguments */
	uint8_t **argv;
	uint16_t argv_size;
	uint16_t argc;

	/* Matches
	 * Index 0 -> Response matches
	 * Index 1 -> Abort matches
	 * Index 2 -> Unsolicited matches
	 */
	struct modem_cmd_match *matches[3];
	uint16_t matches_size[3];

	/* Script execution */
	const struct modem_cmd_script *script;
	struct modem_cmd_script_run_work_item script_run_work;
	struct modem_cmd_script_abort_work_item script_abort_work;
	uint16_t script_cmd_it;
	struct k_event script_event;

	/* Match parsing */
	struct modem_cmd_match *parse_match;
	uint16_t parse_match_len;
	uint16_t parse_arg_len;
	uint16_t parse_match_type;

	/* Process received data */
	struct modem_cmd_work_item process_work;
	k_timeout_t process_timeout;
};

/**
 * @brief Modem command configuration
 *
 * @param user_data Free to use data passed with modem match callbacks
 * @param receive_buf Receive buffer used to store parsed arguments
 * @param receive_buf_size Size of receive buffer should be longest line + longest match
 * @param delimiter Delimiter
 * @param delimiter_size Size of delimiter
 * @param argv Array of pointers used to point to parsed arguments
 * @param argv_size Elements in array of pointers
 * @param unsol_matches Array of unsolicited matches
 * @param unsol_matches_size Elements in array of unsolicited matches
 * @param process_timeout Delay from receive ready event to pipe receive occurs
 */
struct modem_cmd_config {
	void *user_data;
	uint8_t *receive_buf;
	uint16_t receive_buf_size;
	uint8_t *delimiter;
	uint16_t delimiter_size;
	uint8_t **argv;
	uint16_t argv_size;
	struct modem_cmd_match *unsol_matches;
	uint16_t unsol_matches_size;
	k_timeout_t process_timeout;
};

/**
 * @brief Initialize modem pipe command handler
 * @note Command handler must be attached to pipe
 */
int modem_cmd_init(struct modem_cmd *cmd, const struct modem_cmd_config *config);

/**
 * @brief Attach modem command handler to pipe
 *
 * @returns 0 if successful
 * @returns negative errno code if failure
 *
 * @note Command handler is enabled if successful
 */
int modem_cmd_attach(struct modem_cmd *cmd, struct modem_pipe *pipe);

/**
 * @brief Run script
 *
 * @param cmd Modem command instance
 * @param script Script to run
 *
 * @returns 0 if successful
 * @returns -EBUSY if a script is currently running
 * @returns -EPERM if modem pipe is not attached
 * @returns -EINVAL if arguments or script is invalid
 *
 * @note Script runs asynchronously until complete or aborted.
 * @note Use modem_cmd_script_wait() to synchronize with script termination
 */
int modem_cmd_script_run(struct modem_cmd *cmd, const struct modem_cmd_script *script);

/**
 * @brief Abort script
 *
 * @param cmd Modem command instance
 *
 * @note Use modem_cmd_script_wait() to synchronize with script termination
 */
void modem_cmd_script_abort(struct modem_cmd *cmd);

/**
 * @brief Wait until script execution complete
 *
 * @param cmd Modem command instance
 *
 * @returns 0 if script is terminated within timeout
 * @returns -EAGAIN if script is aborted within timeout
 * @returns -EBUSY if timeout elapses while script is still running
 *
 * @note To check status of script, set timeout to K_NO_WAIT
 */
int modem_cmd_script_wait(struct modem_cmd *cmd, k_timeout_t timeout);

/* DEPRECATED */
/**
 * @brief Send command to modem asynchronously
 *
 * @param cmd Modem command instance
 * @param str Command to send without delimiter
 *
 * @returns 0 if successful
 * @returns negative errno code if failure
 */
int modem_cmd_send(struct modem_cmd *cmd, const char *str);

/* DEPRECATED */
/**
 * @brief Send command to modem synchronously using event
 *
 * @param cmd Modem command instance
 * @param str Command to send without delimiter
 * @param event Event object to await
 * @param events Events to await
 * @param timeout Timeout within which any event must occur
 *
 * @returns Set of matching events if successful
 * @returns 0 if timeout occurs
 *
 * @note Events will be cleared before sending command
 */
uint32_t modem_cmd_send_sync_event(struct modem_cmd *cmd, const char *str, struct k_event *event,
				   uint32_t events, k_timeout_t timeout);

/**
 * @brief Release pipe from command handler
 * @note Command handler is now disabled
 */
int modem_cmd_release(struct modem_cmd *cmd);

#endif /* ZEPHYR_DRIVERS_MODEM_MODEM_PIPE_CMD */

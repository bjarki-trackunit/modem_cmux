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
 * @brief Callback invoked when received line matches match configuration
 *
 * @param cmd Pointer to command handler instance
 * @param argv Pointer to array of parsed arguments
 * @param argc Number of parsed arguments, arg 0 holds the exact match which triggered callback
 * @param user_data Free to use user data set during modem_cmd_init()
 */
typedef void (*modem_cmd_match_callback)(struct modem_cmd *cmd, char **argv, uint16_t argc,
					 void *user_data);

/**
 * @brief Modem command match configuration
 *
 * @details Determines which parameters to match in recevied line, and which
 * callback to call if match occurs.
 *
 * @param match This is the string which the recevied line must match
 * @param wildcards This allows for the use of wildcards ("?") in the match string
 * @param separators Char string containing chars which will seperate arguments (",.:")
 * @param callback Called if received line matches configuration
 *
 * @warning Use the provided macros MODEM_CMD_MATCH() and MODEM_CMD_MATCH_WILDCARD() to
 * initialize this struct
 *
 * @example
 *
 * struct modem_cmd_match matches[] = {
 *         MODEM_CMD_MATCH("OK", "", on_ok),
 *         MODEM_CMD_MATCH("AT+CREG", ",", on_creg),
 *         MODEM_CMD_MATCH_WILDCARD("??RMC", ",", on_rmc),
 * };
 *
 */
struct modem_cmd_match {
	const uint8_t *match;
	const uint8_t match_size;
	const uint8_t *separators;
	const uint8_t separators_size;
	const modem_cmd_match_callback callback;
	const bool wildcards;
	bool matching;
};

#define MODEM_CMD_MATCH(_match, _separators, _callback)                 \
	{                                                               \
		.match = (uint8_t *)(_match),                           \
		.match_size = (uint8_t)(sizeof(_match) - 1),            \
		.separators = (uint8_t *)(_separators),                 \
		.separators_size = (uint8_t)(sizeof(_separators) - 1),  \
		.callback = _callback,                                  \
		.wildcards = false,                                     \
		.matching = false,                                      \
	}

#define MODEM_CMD_MATCH_WILDCARD(_match, _separators, _callback)        \
	{                                                               \
		.match = (uint8_t *)(_match),                           \
		.match_size = (uint8_t)(sizeof(_match) - 1),            \
		.separators = (uint8_t *)(_separators),                 \
		.separators_size = (uint8_t)(sizeof(_separators) - 1),  \
		.callback = _callback,                                  \
		.wildcards = true,                                      \
		.matching = false,                                      \
	}

/**
 * @brief Process work item
 *
 * @note k_work struct must be placed first
*/
struct modem_cmd_process_item {
	struct k_work_delayable dwork;
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

	uint8_t *delimiter;
	uint16_t delimiter_size;
	uint16_t delimiter_match_cnt;

	/* Arguments */
	uint8_t **argv;
	uint16_t argv_size;
	uint16_t argc;

	/* Matches */
	struct modem_cmd_match *matches;
	uint16_t matches_size;

	/* Match command parsing */
	struct modem_cmd_match *parse_match;
	uint16_t parse_match_len;
	uint16_t parse_arg_len;

	struct modem_cmd_process_item process;
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
 * @param matches Array of match configurations
 * @param matches_size Elements in array of match configurations
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
	struct modem_cmd_match *matches;
	uint16_t matches_size;
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
 * @brief Send command to modem asynchronously
 *
 * @param cmd Modem command instance
 * @param str Command to send without delimiter
 *
 * @returns 0 if successful
 * @returns negative errno code if failure
 */
int modem_cmd_send(struct modem_cmd *cmd, const char *str);

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
 * @brief Send command to modem synchronously
 *
 * @param cmd Modem command instance
 * @param str Command to send without delimiter
 */
int modem_cmd_send_sync_condition(struct modem_cmd *cmd, const char *str);

/**
 * @brief Release pipe from command handler
 * @note Command handler is now disabled
 */
int modem_cmd_release(struct modem_cmd *cmd);

#endif /* ZEPHYR_DRIVERS_MODEM_MODEM_PIPE_CMD */

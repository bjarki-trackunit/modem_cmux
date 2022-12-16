/*
 * Copyright (c) 2022 Trackunit Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <zephyr/kernel.h>

#ifndef ZEPHYR_DRIVERS_MODEM_MODEM_PIPE
#define ZEPHYR_DRIVERS_MODEM_MODEM_PIPE

struct modem_pipe;

enum modem_pipe_event {
	/** @brief Data ready to be received through pipe */
	MODEM_PIPE_EVENT_RECEIVE_READY = 0,
	/** @brief Pipe was closed */
	MODEM_PIPE_EVENT_CLOSED,
};

typedef void (*modem_pipe_event_handler_t)(struct modem_pipe *pipe, enum modem_pipe_event event,
					   void *user_data);

typedef int (*modem_pipe_event_handler_set_t)(struct modem_pipe *pipe,
					      modem_pipe_event_handler_t handler, void *user_data);

typedef int (*modem_pipe_transmit_t)(struct modem_pipe *pipe, const uint8_t *buf, size_t size);

typedef int (*modem_pipe_receive_t)(struct modem_pipe *pipe, uint8_t *buf, size_t size);

struct modem_pipe_driver_api {
	modem_pipe_event_handler_set_t event_handler_set;
	modem_pipe_transmit_t transmit;
	modem_pipe_receive_t receive;
};

struct modem_pipe {
	void* data;
	struct modem_pipe_driver_api *api;
};

/**
 * @brief Set event handler
 * @note The modem pipe implementation must ensure the pipe belongs to the context which the pipe
 * is connected to.
 */
static inline int modem_pipe_event_handler_set(struct modem_pipe *pipe,
					       modem_pipe_event_handler_t handler, void *user_data)
{
	return pipe->api->event_handler_set(pipe, handler, user_data);
}

/**
 * @brief Transmit data through pipe
 * @note Returns immediately
 * @param pipe Pipe to transmit through
 * @param buf Destination for reveived data
 * @param size Capacity of destination for recevied data
 * @return Number of bytes placed in pipe
 */
static inline int modem_pipe_transmit(struct modem_pipe *pipe, const uint8_t *buf, size_t size)
{
	return pipe->api->transmit(pipe, buf, size);
}

/**
 * @brief Reveive data through pipe
 * @note Returns immediately
 * @param pipe Pipe to receive from
 * @param buf Destination for reveived data
 * @param size Capacity of destination for recevied data
 * @return Number of bytes received from pipe
 */
static inline int modem_pipe_receive(struct modem_pipe *pipe, uint8_t *buf, size_t size)
{
	return pipe->api->receive(pipe, buf, size);
}

#endif /* ZEPHYR_DRIVERS_MODEM_MODEM_PIPE */

/*
 * Copyright (c) 2022 Trackunit Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(modem_cmux);

#include <zephyr/kernel.h>
#include <zephyr/sys/crc.h>
#include <string.h>

#include "modem_cmux.h"

#define MODEM_CMUX_N1 256 /* default I frame size, GSM 07.10 ch 6.2.2.1 */
#define MODEM_CMUX_N2 3	  /* retry 3 times */

#define MODEM_CMUX_FCS_POLYNOMIAL 0xE0 /* reversed crc8 */
#define MODEM_CMUX_FCS_INIT_VALUE 0xFF

#define MODEM_CMUX_EA 0x01 /* Extension bit      */
#define MODEM_CMUX_CR 0x02 /* Command / Response */
#define MODEM_CMUX_PF 0x10 /* Poll / Final       */

/* Frame types sent to and from bus */
#define MODEM_CMUX_FRAME_TYPE_RR   0x01 /* Receive Ready                            */
#define MODEM_CMUX_FRAME_TYPE_UI   0x03 /* Unnumbered Information                   */
#define MODEM_CMUX_FRAME_TYPE_RNR  0x05 /* Receive Not Ready                        */
#define MODEM_CMUX_FRAME_TYPE_REJ  0x09 /* Reject                                   */
#define MODEM_CMUX_FRAME_TYPE_DM   0x0F /* Disconnected Mode                        */
#define MODEM_CMUX_FRAME_TYPE_SABM 0x2F /* Set Asynchronous Balanced Mode           */
#define MODEM_CMUX_FRAME_TYPE_DISC 0x43 /* Disconnect                               */
#define MODEM_CMUX_FRAME_TYPE_UA   0x63 /* Unnumbered Acknowledgement               */
#define MODEM_CMUX_FRAME_TYPE_UIH  0xEF /* Unnumbered Information with Header check */

/* Commands sent in UIH frames on control channel */
#define MODEM_CMUX_CMD_NSC   0x08 /* Non Supported Command Response           */
#define MODEM_CMUX_CMD_TEST  0x10 /* Test Command                             */
#define MODEM_CMUX_CMD_PSC   0x20 /* Power Saving Control                     */
#define MODEM_CMUX_CMD_RLS   0x28 /* Remote Line Status Command               */
#define MODEM_CMUX_CMD_FCOFF 0x30 /* Flow Control Off Command                 */
#define MODEM_CMUX_CMD_PN    0x40 /* DLC parameter negotiation                */
#define MODEM_CMUX_CMD_RPN   0x48 /* Remote Port Negotiation Command          */
#define MODEM_CMUX_CMD_FCON  0x50 /* Flow Control On Command                  */
#define MODEM_CMUX_CMD_CLD   0x60 /* Multiplexer close down                   */
#define MODEM_CMUX_CMD_SNC   0x68 /* Service Negotiation Command              */
#define MODEM_CMUX_CMD_MSC   0x70 /* Modem Status Command                     */

/*************************************************************************************************
 * Definitions
 *************************************************************************************************/
#define MODEM_CMUX_F_MARKER		 (0xF9)
#define MODEM_CMUX_DLCI_ADDRESS_MIN	 (1U)
#define MODEM_CMUX_DLCI_ADDRESS_MAX	 (32767U)
#define MODEM_CMUX_FRAME_SIZE_MIN	 (6U)
#define MODEM_CMUX_FRAME_HEADER_SIZE_MAX (6U)
#define MODEM_CMUX_FRAME_TAIL_SIZE	 (2U)
#define MODEM_CMUX_RECEIVE_BUF_SIZE_MIN                                                            \
	(128U + MODEM_CMUX_FRAME_HEADER_SIZE_MAX + MODEM_CMUX_FRAME_TAIL_SIZE)
#define MODEM_CMUX_DLCI_RECEIVE_BUF_SIZE_MIN                                                       \
	(MODEM_CMUX_FRAME_HEADER_SIZE_MAX + MODEM_CMUX_FRAME_TAIL_SIZE)
#define MODEM_CMUX_FRAME_TRANSMIT_INTERVAL_MIN_MS (10U)

#define MODEM_CMUX_EVENT_BUS_PIPE_RECEIVE_READY_BIT (0)

/*************************************************************************************************
 * Declarations
 *************************************************************************************************/
struct modem_cmux_frame_encoded {
	uint8_t header[MODEM_CMUX_FRAME_HEADER_SIZE_MAX];
	uint8_t header_len;
	const uint8_t *data;
	uint16_t data_len;
	uint8_t tail[MODEM_CMUX_FRAME_TAIL_SIZE];
	uint8_t tail_len;
};

/*************************************************************************************************
 * Logging helpers
 *************************************************************************************************/
static void modem_cmux_log_unknown_frame(struct modem_cmux *cmux)
{
	char data[24];
	uint8_t data_cnt = (cmux->frame.data_len < 8) ? cmux->frame.data_len : 8;
	for (uint8_t i = 0; i < data_cnt; i++) {
		snprintk(&data[i * 3], sizeof(data) - (i * 3), "%02X,", cmux->frame.data[i]);
	}

	/* Remove trailing */
	if (data_cnt > 0) {
		data[(data_cnt * 3) - 1] = '\0';
	}

	LOG_DBG("ch:%u, type:%u, data:%s", cmux->frame.dlci_address, cmux->frame.type, data);
}

/************************************************************************************************
 * Static non-threadsafe helpers
 *************************************************************************************************/
static void modem_cmux_dlci_pipe_raise_event(struct modem_cmux_dlci *dlci,
					     enum modem_pipe_event event)
{
	/* Notify pipe closed */
	if (dlci->pipe_event_handler != NULL) {
		dlci->pipe_event_handler(dlci->pipe, event, dlci->pipe_event_handler_user_data);
	}
}

static struct modem_cmux_dlci *modem_cmux_dlci_alloc(struct modem_cmux *cmux)
{
	for (uint16_t i = 0; i < cmux->dlcis_cnt; i++) {
		if (cmux->dlcis[i].allocated == true) {
			continue;
		}

		cmux->dlcis[i].allocated = true;

		return &cmux->dlcis[i];
	}

	return NULL;
}

static void modem_cmux_dlci_free(struct modem_cmux *cmux, struct modem_cmux_dlci *dlci)
{
	dlci->allocated = false;
}

/* Find allocated DLCI channel context with matcing DLCI channel address */
static struct modem_cmux_dlci *modem_cmux_dlci_find(struct modem_cmux *cmux, uint16_t dlci_address)
{
	for (uint16_t i = 0; i < cmux->dlcis_cnt; i++) {
		if (cmux->dlcis[i].allocated == false) {
			continue;
		}

		if (cmux->dlcis[i].dlci_address != dlci_address) {
			continue;
		}

		return &cmux->dlcis[i];
	}

	return NULL;
}

static void modem_cmux_dlci_deinit(struct modem_cmux_dlci *dlci)
{
	/* Notify pipe closed */
	modem_cmux_dlci_pipe_raise_event(dlci, MODEM_PIPE_EVENT_CLOSED);

	/* Release pipe */
	modem_pipe_event_handler_set(dlci->pipe, NULL, NULL);

	dlci->dlci_address = 0;
	dlci->cmux = NULL;
	dlci->pipe = NULL;
	dlci->pipe_event_handler = NULL;
	dlci->pipe_event_handler_user_data = NULL;
	ring_buf_reset(&dlci->receive_rb);
	dlci->state = MODEM_CMUX_DLCI_STATE_CLOSED;
}

static void modem_cmux_dlci_close_all(struct modem_cmux *cmux)
{
	struct modem_cmux_dlci *dlci;

	for (uint16_t i = 0; i < cmux->dlcis_cnt; i++) {
		/* Ease readability */
		dlci = &cmux->dlcis[i];

		/* Check if allocated */
		if (dlci->allocated == false) {
			continue;
		}

		/* Check if already closed */
		if (dlci->state == MODEM_CMUX_DLCI_STATE_CLOSED) {
			continue;
		}

		/* Deinit CMUX DLCI channel context and pipe */
		modem_cmux_dlci_deinit(dlci);

		/* Free DLCI channel context */
		modem_cmux_dlci_free(cmux, dlci);
	}
}

static void modem_cmux_raise_event(struct modem_cmux *cmux, struct modem_cmux_event event)
{
	/* Validate event handler set */
	if (cmux->event_handler == NULL) {
		return;
	}

	/* Invoke event handler */
	cmux->event_handler(cmux, event, cmux->event_handler_user_data);
}

static void modem_cmux_bus_event_handler(struct modem_pipe *pipe, enum modem_pipe_event event,
					 void *user_data)
{
	struct modem_cmux *cmux = (struct modem_cmux *)user_data;

	if (event == MODEM_PIPE_EVENT_RECEIVE_READY) {
		k_work_schedule(&cmux->process_received.dwork, cmux->receive_timeout);
	}
}

static int modem_cmux_bus_write(struct modem_cmux *cmux, const uint8_t *data, size_t size)
{
	int ret;
	size_t sent = 0;

	while (sent < size) {
		ret = modem_pipe_transmit(cmux->pipe, data, (size - sent));
		if (ret < 0) {
			return -EAGAIN;
		}

		sent += (size_t)ret;

		k_yield();
	}

	return 0;
}

static int modem_cmux_bus_write_frame_encode(const struct modem_cmux_frame *frame,
					     struct modem_cmux_frame_encoded *encoded)
{
	uint8_t pos = 0;

	/* SOF */
	encoded->header[pos] = MODEM_CMUX_F_MARKER;
	pos++;

	/* DLCI Address */
	if (63 < frame->dlci_address) {
		encoded->header[pos] =
			(((uint8_t)frame->cr) << 1) | ((uint8_t)(frame->dlci_address << 2));
		pos++;

		encoded->header[pos] = ((uint8_t)(frame->dlci_address >> 6));
		pos++;
	} else {
		encoded->header[pos] = (0x01) | (((uint8_t)frame->cr) << 1) |
				       ((uint8_t)(frame->dlci_address << 2));
		pos++;
	}

	/* Frame type and poll/final */
	encoded->header[pos] = frame->type;
	encoded->header[pos] |= ((uint8_t)frame->pf) << 4;
	pos++;

	/* Data length */
	if (127 < frame->data_len) {
		encoded->header[pos] = (uint8_t)(frame->data_len << 1);
		pos++;

		encoded->header[pos] = (uint8_t)(frame->data_len >> 7);
		pos++;
	} else {
		encoded->header[pos] = (0x01) | (uint8_t)(frame->data_len << 1);
		pos++;
	}

	/* Header length */
	encoded->header_len = pos;

	/* Data */
	encoded->data = frame->data;
	encoded->data_len = frame->data_len;

	/* FCS */
	if (frame->type == MODEM_CMUX_FRAME_TYPE_UIH) {
		encoded->tail[0] =
			0xFF - crc8(&encoded->header[1], (encoded->header_len - 1),
				    MODEM_CMUX_FCS_POLYNOMIAL, MODEM_CMUX_FCS_INIT_VALUE, true);
	} else {
		encoded->tail[0] =
			0xFF - crc8(&encoded->header[1],
				    (encoded->header_len - 1 + encoded->data_len),
				    MODEM_CMUX_FCS_POLYNOMIAL, MODEM_CMUX_FCS_INIT_VALUE, true);
	}

	/* EOF */
	encoded->tail[1] = MODEM_CMUX_F_MARKER;
	encoded->tail_len = 2;

	return 0;
}

static int modem_cmux_bus_write_frame(struct modem_cmux *cmux, const struct modem_cmux_frame *frame)
{
	int ret;
	struct modem_cmux_frame_encoded encoded;

	ret = modem_cmux_bus_write_frame_encode(frame, &encoded);
	if (ret < 0) {
		return -EINVAL;
	}

	/* Write header */
	ret = modem_cmux_bus_write(cmux, encoded.header, encoded.header_len);
	if (ret < 0) {
		return -EAGAIN;
	}

	/* Write data */
	ret = modem_cmux_bus_write(cmux, encoded.data, encoded.data_len);
	if (ret < 0) {
		return -EAGAIN;
	}

	/* Write tail */
	ret = modem_cmux_bus_write(cmux, encoded.tail, encoded.tail_len);
	if (ret < 0) {
		return -EAGAIN;
	}

	return 0;
}

static int modem_cmux_bus_pipe_receive(struct modem_cmux *cmux)
{
	int ret;

	ret = modem_pipe_receive(cmux->pipe, cmux->work_buf, sizeof(cmux->work_buf));

	if (ret < 0) {
		return ret;
	}

	cmux->work_buf_len = (uint16_t)ret;

	return ret;
}

/*************************************************************************************************
 * Modem CMUX processing
 *************************************************************************************************/
static void modem_cmux_process_on_frame_received_ua_control(struct modem_cmux *cmux)
{
	/* Update cmux state */
	cmux->state = MODEM_CMUX_STATE_CONNECTED;

	/* Notify cmux state updated */
	struct modem_cmux_event cmux_event = {.dlci_address = 0,
					      .type = MODEM_CMUX_EVENT_CONNECTED};

	modem_cmux_raise_event(cmux, cmux_event);

	return;
}

static void modem_cmux_process_on_frame_received_ua(struct modem_cmux *cmux)
{
	struct modem_cmux_dlci *dlci;

	/* Find DLCI channel context using DLCI address */
	dlci = modem_cmux_dlci_find(cmux, cmux->frame.dlci_address);
	if (dlci == NULL) {
		return;
	}

	if (dlci->state == MODEM_CMUX_DLCI_STATE_OPENING) {
		/* Update CMXU DLCI channel state */
		dlci->state = MODEM_CMUX_DLCI_STATE_OPEN;

		/* Notify CMXU DLCI channel state changed */
		struct modem_cmux_event cmux_event = {.dlci_address = cmux->frame.dlci_address,
						      .type = MODEM_CMUX_EVENT_OPENED};

		modem_cmux_raise_event(cmux, cmux_event);

		return;
	}

	if (dlci->state == MODEM_CMUX_DLCI_STATE_CLOSING) {
		/* Notify CMXU DLCI channel state changed */
		struct modem_cmux_event cmux_event = {.dlci_address = cmux->frame.dlci_address,
						      .type = MODEM_CMUX_EVENT_CLOSED};

		modem_cmux_raise_event(cmux, cmux_event);

		/* Deinit CMUX DLCI channel context and pipe */
		modem_cmux_dlci_deinit(dlci);

		/* Free DLCI channel context */
		modem_cmux_dlci_free(cmux, dlci);

		return;
	}
}

static void modem_cmux_process_on_frame_received_uih_control(struct modem_cmux *cmux)
{
	/* Modem CMUX disconnected */
	if (cmux->frame.data_len == 2) {
		if ((cmux->frame.data[0] == ((((uint8_t)MODEM_CMUX_CMD_CLD) << 1) | 0x01)) &&
		    (cmux->frame.data[1] == 0x01) && (cmux->frame.cr == true) &&
		    (cmux->frame.pf == false) && (cmux->state == MODEM_CMUX_STATE_DISCONNECTING)) {
			/* Update CMUX state */
			cmux->state = MODEM_CMUX_STATE_DISCONNECTED;

			/* Notify CMUX state changed */
			struct modem_cmux_event cmux_event = {
				.dlci_address = 0, .type = MODEM_CMUX_EVENT_DISCONNECTED};

			modem_cmux_raise_event(cmux, cmux_event);

			/* Release bus pipe */
			modem_pipe_event_handler_set(cmux->pipe, NULL, NULL);
			cmux->pipe = NULL;

			return;
		}
	}

	/* Modem status event */
	if (cmux->frame.data_len == 4) {
		if ((cmux->frame.data[0] == ((((uint8_t)MODEM_CMUX_CMD_MSC) << 1) | 0x03)) &&
		    (cmux->frame.cr == false) && (cmux->frame.pf == true)) {
			/* Notify CMUX event occured */
			struct modem_cmux_event cmux_event = {
				.dlci_address = 0, .type = MODEM_CMUX_EVENT_MODEM_STATUS};

			modem_cmux_raise_event(cmux, cmux_event);

			/* Respond with identical frame */
			modem_cmux_bus_write_frame(cmux, &cmux->frame);

			return;
		}
	}

	/* Modem status event */
	if (cmux->frame.data_len == 4) {
		if ((cmux->frame.data[0] == ((((uint8_t)MODEM_CMUX_CMD_MSC) << 1) | 0x01)) &&
		    (cmux->frame.cr == true) && (cmux->frame.pf == true)) {
			/* Notify CMUX event occured */
			struct modem_cmux_event cmux_event = {
				.dlci_address = 0, .type = MODEM_CMUX_EVENT_MODEM_STATUS};

			modem_cmux_raise_event(cmux, cmux_event);

			/* Respond with identical frame */
			modem_cmux_bus_write_frame(cmux, &cmux->frame);

			return;
		}
	}

	/* Notify unknown frame */
	modem_cmux_log_unknown_frame(cmux);
}

static void modem_cmux_process_on_frame_received_uih(struct modem_cmux *cmux)
{
	struct modem_cmux_dlci *dlci;

	/* Find DLCI channel context using DLCI address */
	dlci = modem_cmux_dlci_find(cmux, cmux->frame.dlci_address);
	if (dlci == NULL) {
		return;
	}

	/* Copy data to DLCI channel receive buffer */
	k_mutex_lock(&dlci->receive_rb_lock, K_FOREVER);

	ring_buf_put(&dlci->receive_rb, cmux->frame.data, cmux->frame.data_len);

	k_mutex_unlock(&dlci->receive_rb_lock);

	/* Notify data received */
	modem_cmux_dlci_pipe_raise_event(dlci, MODEM_PIPE_EVENT_RECEIVE_READY);

	return;
}

static void modem_cmux_process_on_frame_received(struct modem_cmux *cmux)
{
	if (cmux->frame.dlci_address == 0) {
		switch (cmux->frame.type) {
		case MODEM_CMUX_FRAME_TYPE_UA:
			modem_cmux_process_on_frame_received_ua_control(cmux);
			break;
		case MODEM_CMUX_FRAME_TYPE_UIH:
			modem_cmux_process_on_frame_received_uih_control(cmux);
			break;
		default:
			break;
		}

		return;
	}

	switch (cmux->frame.type) {
	case MODEM_CMUX_FRAME_TYPE_UA:
		modem_cmux_process_on_frame_received_ua(cmux);
		break;
	case MODEM_CMUX_FRAME_TYPE_UIH:
		modem_cmux_process_on_frame_received_uih(cmux);
		break;
	default:
		break;
	}
}

static void modem_cmux_process_received_byte(struct modem_cmux *cmux, uint8_t byte)
{
	switch (cmux->receive_state) {
	case MODEM_CMUX_RECEIVE_STATE_SOF:
		if (byte == MODEM_CMUX_F_MARKER) {
			LOG_DBG("Receiving frame");

			/* Initialize */
			cmux->receive_buf_len = 0;
			cmux->frame_header_len = 0;

			/* Await address */
			cmux->receive_state = MODEM_CMUX_RECEIVE_STATE_ADDRESS;
		}

		break;

	case MODEM_CMUX_RECEIVE_STATE_ADDRESS:
		/* Store header for FCS */
		cmux->frame_header[cmux->frame_header_len] = byte;
		cmux->frame_header_len++;

		/* Get CR */
		cmux->frame.cr = (byte & 0x02) ? true : false;

		/* Get DLCI address */
		cmux->frame.dlci_address = (byte >> 2) & 0x3F;

		/* Await control */
		cmux->receive_state = MODEM_CMUX_RECEIVE_STATE_CONTROL;

		break;

	case MODEM_CMUX_RECEIVE_STATE_CONTROL:
		/* Store header for FCS */
		cmux->frame_header[cmux->frame_header_len] = byte;
		cmux->frame_header_len++;

		/* Get PF */
		cmux->frame.pf = (byte & MODEM_CMUX_PF) ? true : false;

		/* Get frame type */
		cmux->frame.type = byte & (~MODEM_CMUX_PF);

		/* Await data length */
		cmux->receive_state = MODEM_CMUX_RECEIVE_STATE_LENGTH;

		break;

	case MODEM_CMUX_RECEIVE_STATE_LENGTH:
		/* Store header for FCS */
		cmux->frame_header[cmux->frame_header_len] = byte;
		cmux->frame_header_len++;

		/* Get first 7 bits of data length */
		cmux->frame.data_len = (byte >> 1);

		/* Check if length field continues */
		if ((byte & MODEM_CMUX_EA) == 0) {
			/* Await continued length field */
			cmux->receive_state = MODEM_CMUX_RECEIVE_STATE_LENGTH_CONT;

			break;
		}

		/* Check if no data field */
		if (cmux->frame.data_len == 0) {
			/* Await FCS */
			cmux->receive_state = MODEM_CMUX_RECEIVE_STATE_FCS;

			break;
		}

		/* Await data */
		cmux->receive_state = MODEM_CMUX_RECEIVE_STATE_DATA;

		break;

	case MODEM_CMUX_RECEIVE_STATE_LENGTH_CONT:
		/* Store header for FCS */
		cmux->frame_header[cmux->frame_header_len] = byte;
		cmux->frame_header_len++;

		/* Get last 8 bits of data length */
		cmux->frame.data_len |= ((uint16_t)byte) << 7;

		/* Await data */
		cmux->receive_state = MODEM_CMUX_RECEIVE_STATE_DATA;

		break;

	case MODEM_CMUX_RECEIVE_STATE_DATA:
		/* Copy byte to data */
		cmux->receive_buf[cmux->receive_buf_len] = byte;
		cmux->receive_buf_len++;

		/* Check if datalen reached */
		if (cmux->frame.data_len == cmux->receive_buf_len) {
			/* Await FCS */
			cmux->receive_state = MODEM_CMUX_RECEIVE_STATE_FCS;

			break;
		}

		/* Check if receive buffer overrun */
		if (cmux->receive_buf_len == cmux->receive_buf_size) {
			LOG_DBG("Receive buf overrun");

			/* Drop frame */
			cmux->receive_state = MODEM_CMUX_RECEIVE_STATE_EOF;

			break;
		}

		break;

	case MODEM_CMUX_RECEIVE_STATE_FCS:
		uint8_t fcs;

		/* Compute FCS */
		if (cmux->frame.type == MODEM_CMUX_FRAME_TYPE_UIH) {
			fcs = 0xFF - crc8(cmux->frame_header, cmux->frame_header_len,
					  MODEM_CMUX_FCS_POLYNOMIAL, MODEM_CMUX_FCS_INIT_VALUE,
					  true);
		} else {
			/* FIX ME */
			fcs = 0xFF - crc8(cmux->frame_header, cmux->frame_header_len,
					  MODEM_CMUX_FCS_POLYNOMIAL, MODEM_CMUX_FCS_INIT_VALUE,
					  true);
		}

		/* Validate FCS */
		#if 0
		TODO("Issue with at+qeng=\"servingcell\"")
		if (fcs != byte) {
			LOG_DBG("Frame FCS err");

			/* Drop frame */
			cmux->receive_state = MODEM_CMUX_RECEIVE_STATE_SOF;

			break;
		}
		#endif

		cmux->receive_state = MODEM_CMUX_RECEIVE_STATE_EOF;

		break;

	case MODEM_CMUX_RECEIVE_STATE_EOF:
		/* Validate byte is EOF */
		if (byte != MODEM_CMUX_F_MARKER) {
			/* Unexpected byte */
			cmux->receive_state = MODEM_CMUX_RECEIVE_STATE_SOF;

			break;
		}

		LOG_DBG("Received frame");

		/* Process frame */
		cmux->frame.data = cmux->receive_buf;
		modem_cmux_process_on_frame_received(cmux);

		/* Await start of next frame */
		cmux->receive_state = MODEM_CMUX_RECEIVE_STATE_SOF;

		break;

	default:
		break;
	}
}

static void modem_cmux_process_received_bytes(struct modem_cmux *cmux)
{
	for (uint16_t i = 0; i < cmux->work_buf_len; i++) {
		modem_cmux_process_received_byte(cmux, cmux->work_buf[i]);
	}
}

static void modem_cmux_process_received(struct k_work *item)
{
	struct modem_cmux_work_delayable *cmux_process = (struct modem_cmux_work_delayable *)item;
	struct modem_cmux *cmux = cmux_process->cmux;

	/* Receive and process all available data from bus pipe */
	while (modem_cmux_bus_pipe_receive(cmux) > 0) {
		modem_cmux_process_received_bytes(cmux);
	}
}

/*************************************************************************************************
 * Thread-safe DLCI channel pipe API
 *************************************************************************************************/
static int modem_cmux_dlci_pipe_event_handler_set(struct modem_pipe *pipe,
						  modem_pipe_event_handler_t handler,
						  void *user_data)
{
	struct modem_cmux_dlci *dlci = (struct modem_cmux_dlci *)pipe->data;
	struct modem_cmux *cmux = dlci->cmux;

	k_mutex_lock(&cmux->lock, K_FOREVER);

	dlci->pipe_event_handler = handler;
	dlci->pipe_event_handler_user_data = user_data;

	k_mutex_unlock(&cmux->lock);

	return 0;
}

static int modem_cmux_dlci_pipe_transmit(struct modem_pipe *pipe, const uint8_t *buf, uint32_t size)
{
	struct modem_cmux_dlci *dlci = (struct modem_cmux_dlci *)pipe->data;
	struct modem_cmux *cmux = dlci->cmux;
	int ret;

	struct modem_cmux_frame frame = {
		.dlci_address = dlci->dlci_address,
		.cr = true,
		.pf = false,
		.type = MODEM_CMUX_FRAME_TYPE_UIH,
		.data = buf,
		.data_len = size,
	};

	k_mutex_lock(&cmux->lock, K_FOREVER);

	ret = modem_cmux_bus_write_frame(cmux, &frame);
	if (ret < 0) {
		k_mutex_unlock(&cmux->lock);
		return ret;
	}

	k_mutex_unlock(&cmux->lock);
	return size;
}

static int modem_cmux_dlci_pipe_receive(struct modem_pipe *pipe, uint8_t *buf, uint32_t size)
{
	struct modem_cmux_dlci *dlci = (struct modem_cmux_dlci *)pipe->data;
	uint32_t ret;

	/* Validate arguments */
	if ((pipe == NULL) || (buf == NULL) || (size == 0)) {
		return -EINVAL;
	}

	k_mutex_lock(&dlci->receive_rb_lock, K_FOREVER);

	ret = ring_buf_get(&dlci->receive_rb, buf, size);

	k_mutex_unlock(&dlci->receive_rb_lock);

	return (int)ret;
}

struct modem_pipe_driver_api modem_cmux_dlci_pipe_api = {
	.event_handler_set = modem_cmux_dlci_pipe_event_handler_set,
	.transmit = modem_cmux_dlci_pipe_transmit,
	.receive = modem_cmux_dlci_pipe_receive,
};

/*************************************************************************************************
 * DLCI channel initialization
 *************************************************************************************************/
static void modem_cmux_dlci_init(struct modem_cmux *cmux, struct modem_cmux_dlci *dlci,
				 uint16_t dlci_address, struct modem_pipe *pipe,
				 uint8_t *receive_buf, uint16_t receive_buf_size)
{
	dlci->dlci_address = dlci_address;
	dlci->cmux = cmux;
	dlci->pipe = pipe;
	dlci->pipe->data = dlci;
	pipe->api = &modem_cmux_dlci_pipe_api;
	dlci->pipe_event_handler = NULL;
	dlci->pipe_event_handler_user_data = NULL;
	ring_buf_init(&dlci->receive_rb, receive_buf_size, receive_buf);
	k_mutex_init(&dlci->receive_rb_lock);
	dlci->state = MODEM_CMUX_DLCI_STATE_CLOSED;
}

/*************************************************************************************************
 * Public threadsafe functions
 *************************************************************************************************/
int modem_cmux_init(struct modem_cmux *cmux, const struct modem_cmux_config *config)
{
	/* Validate arguments */
	if (cmux == NULL || config == NULL) {
		return -EINVAL;
	}

	/* Validate config */
	if ((config->event_handler == NULL) || (config->dlcis == NULL) ||
	    (config->dlcis_cnt == 0) || (config->receive_buf == NULL) ||
	    (config->receive_buf_size == 0)) {
		return -EINVAL;
	}

	/* Clear CMUX context */
	memset(cmux, 0x00, sizeof(*cmux));

	/* Clear DLCI channel contexts */
	for (uint16_t i = 0; i < config->dlcis_cnt; i++) {
		memset(&config->dlcis[i], 0x00, sizeof(config->dlcis[0]));
	}

	/* Copy configuration to context */
	cmux->event_handler = config->event_handler;
	cmux->event_handler_user_data = config->event_handler_user_data;
	cmux->dlcis = config->dlcis;
	cmux->dlcis_cnt = config->dlcis_cnt;
	cmux->receive_buf = config->receive_buf;
	cmux->receive_buf_size = config->receive_buf_size;
	cmux->receive_timeout = config->receive_timeout;

	/* Initialize delayable work */
	cmux->process_received.cmux = cmux;
	k_work_init_delayable(&cmux->process_received.dwork, modem_cmux_process_received);

	return 0;
}

int modem_cmux_connect(struct modem_cmux *cmux, struct modem_pipe *pipe)
{
	int ret;

	struct modem_cmux_frame frame = {
		.dlci_address = 0,
		.cr = true,
		.pf = true,
		.type = MODEM_CMUX_FRAME_TYPE_SABM,
		.data = NULL,
		.data_len = 0,
	};

	k_mutex_lock(&cmux->lock, K_FOREVER);

	/* Verify cmux disconnected */
	if (cmux->state != MODEM_CMUX_STATE_DISCONNECTED) {
		k_mutex_unlock(&cmux->lock);
		return -EPERM;
	}

	/* Attach bus pipe */
	cmux->pipe = pipe;
	ret = modem_pipe_event_handler_set(cmux->pipe, modem_cmux_bus_event_handler, cmux);
	if (ret < 0) {
		k_mutex_unlock(&cmux->lock);
		return ret;
	}

	/* Update CMUX state */
	cmux->state = MODEM_CMUX_STATE_CONNECTING;

	/* Send connection request */
	ret = modem_cmux_bus_write_frame(cmux, &frame);
	if (ret < 0) {
		k_mutex_unlock(&cmux->lock);
		return ret;
	}

	k_mutex_unlock(&cmux->lock);
	return 0;
}

int modem_cmux_dlci_open(struct modem_cmux *cmux, struct modem_cmux_dlci_config *config,
			 struct modem_pipe *pipe)
{
	int ret;
	struct modem_cmux_dlci *dlci;

	struct modem_cmux_frame frame = {
		.dlci_address = config->dlci_address,
		.cr = true,
		.pf = true,
		.type = MODEM_CMUX_FRAME_TYPE_SABM,
		.data = NULL,
		.data_len = 0,
	};

	/* Validate arguments */
	if ((cmux == NULL) || (config == NULL) || (pipe == NULL)) {
		return -EINVAL;
	}

	/* Validate configuration */
	if ((config->receive_buf == NULL) ||
	    (config->receive_buf_size == MODEM_CMUX_DLCI_RECEIVE_BUF_SIZE_MIN) ||
	    (MODEM_CMUX_DLCI_ADDRESS_MAX < config->dlci_address) ||
	    (config->dlci_address < MODEM_CMUX_DLCI_ADDRESS_MIN)) {
		return -EINVAL;
	}

	k_mutex_lock(&cmux->lock, K_FOREVER);

	/* Verify cmux connected */
	if (cmux->state != MODEM_CMUX_STATE_CONNECTED) {
		k_mutex_unlock(&cmux->lock);
		return -EPERM;
	}

	/* Verify channel not already open */
	dlci = modem_cmux_dlci_find(cmux, config->dlci_address);
	if (dlci != NULL) {
		k_mutex_unlock(&cmux->lock);
		return -EPERM;
	}

	/* Try to alloc DLCI channel context */
	dlci = modem_cmux_dlci_alloc(cmux);
	if (dlci == NULL) {
		k_mutex_unlock(&cmux->lock);
		return -ENOMEM;
	}

	/* Initialize DLCI channel context and pipe */
	modem_cmux_dlci_init(cmux, dlci, config->dlci_address, pipe, config->receive_buf,
			     config->receive_buf_size);

	/* Update DLCI channel context state */
	dlci->state = MODEM_CMUX_DLCI_STATE_OPENING;

	/* Try to open DLCI channel */
	ret = modem_cmux_bus_write_frame(cmux, &frame);
	if (ret < 0) {
		modem_cmux_dlci_free(cmux, dlci);
		k_mutex_unlock(&cmux->lock);
		return ret;
	}

	k_mutex_unlock(&cmux->lock);
	return 0;
}

int modem_cmux_dlci_close(struct modem_pipe *pipe)
{
	struct modem_cmux_dlci *dlci = (struct modem_cmux_dlci *)pipe->data;
	struct modem_cmux *cmux = dlci->cmux;

	int ret;

	/* Validate argument */
	if ((pipe == NULL)) {
		return -EINVAL;
	}

	k_mutex_lock(&cmux->lock, K_FOREVER);

	/* Validate pipe is open */
	if (dlci->state != MODEM_CMUX_DLCI_STATE_OPEN) {
		k_mutex_unlock(&cmux->lock);
		return -EPERM;
	}

	/* Try to close DLCI channel */
	struct modem_cmux_frame frame = {
		.dlci_address = dlci->dlci_address,
		.cr = true,
		.pf = true,
		.type = MODEM_CMUX_FRAME_TYPE_DISC,
		.data = NULL,
		.data_len = 0,
	};

	/* Update CMUX DLCI channel state */
	dlci->state = MODEM_CMUX_DLCI_STATE_CLOSING;

	ret = modem_cmux_bus_write_frame(cmux, &frame);
	if (ret < 0) {
		k_mutex_unlock(&cmux->lock);
		return ret;
	}

	k_mutex_unlock(&cmux->lock);
	return 0;
}

int modem_cmux_disconnect(struct modem_cmux *cmux)
{
	int ret;
	uint8_t cmd[] = {(((uint8_t)MODEM_CMUX_CMD_CLD) << 1) | 0x03, 0x01};

	struct modem_cmux_frame frame = {
		.dlci_address = 0,
		.cr = true,
		.pf = false,
		.type = MODEM_CMUX_FRAME_TYPE_UIH,
		.data = cmd,
		.data_len = sizeof(cmd),
	};

	k_mutex_lock(&cmux->lock, K_FOREVER);

	/* Verify cmux connected */
	if (cmux->state != MODEM_CMUX_STATE_CONNECTED) {
		k_mutex_unlock(&cmux->lock);
		return -EPERM;
	}

	/* Close all open DLCI channels */
	modem_cmux_dlci_close_all(cmux);

	/* Update state */
	cmux->state = MODEM_CMUX_STATE_DISCONNECTING;

	/* Request disconnect */
	ret = modem_cmux_bus_write_frame(cmux, &frame);
	if (ret < 0) {
		k_mutex_unlock(&cmux->lock);
		return ret;
	}

	/* Lazy await disconnect */
	k_msleep(300);

	/* Release bus pipe */
	modem_pipe_event_handler_set(cmux->pipe, NULL, NULL);

	/* Cancel work */
	struct k_work_sync sync;
	k_work_cancel_delayable_sync(&cmux->process_received.dwork, &sync);

	k_mutex_unlock(&cmux->lock);

	return 0;
}

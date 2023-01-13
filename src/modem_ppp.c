#include <zephyr/net/ppp.h>
#include <zephyr/sys/crc.h>
#include <zephyr/logging/log.h>

#include <string.h>

#include "modem_ppp.h"

LOG_MODULE_REGISTER(modem_ppp, 4);
/*********************************************************
 * DEFINES / MACROS
 *********************************************************/
#define MODEM_PPP_FRAME_SOF		    (0x7E)
#define MODEM_PPP_FRAME_ESCAPE		    (0x7D)
#define MODEM_PPP_FRAME_SOF_SIZE	    (1)
#define MODEM_PPP_FRAME_WRAPPED_HEADER_SIZE (3)
#define MODEM_PPP_FRAME_TAIL_SIZE	    (2)

/*********************************************************
 * TYPE DEFINITIONS
 *********************************************************/

/*********************************************************
 * LOCAL FUNCTIONS DECLARATION
 *********************************************************/
static int modem_ppp_get_protocol(uint8_t *protocol, struct net_pkt *pkt);
static uint16_t modem_ppp_wrap_write_byte(uint8_t *buf, uint16_t size, uint8_t byte);
static uint16_t modem_ppp_fcs_init(const uint8_t *buf, uint16_t len);
static uint16_t modem_ppp_fcs_update(uint16_t fcs, const uint8_t *buf, uint16_t len);
static void modem_ppp_fcs_final(uint8_t *fcs_bytes, uint16_t fcs);
static int modem_ppp_wrap_net_pkt(struct modem_ppp *ppp, struct net_pkt *pkt);
static int modem_ppp_send_net_pkt(struct modem_ppp *ppp);
static bool modem_ppp_receive_data(struct modem_ppp *ppp);
static void modem_ppp_process_received_byte(struct modem_ppp *ppp, uint8_t byte);
static void modem_ppp_process_received_data(struct modem_ppp *ppp);
static void modem_ppp_pipe_event_handler(struct modem_pipe *pipe, enum modem_pipe_event event,
					 void *user_data);

/*********************************************************
 * LOCAL VARIABLES
 *********************************************************/
static const uint8_t ppp_frame_header[2] = {0xFF, 0x03};

/*********************************************************
 * LOCAL FUNCTIONS
 *********************************************************/
static int modem_ppp_get_protocol(uint8_t *protocol, struct net_pkt *pkt)
{
	if (net_pkt_is_ppp(pkt) == true) {
		return -EALREADY;
	}

	if (net_pkt_family(pkt) == AF_INET) {
		protocol[0] = PPP_IP & 0xFF;
		protocol[1] = (PPP_IP >> 8) & 0xFF;

		return 0;
	}

	if (net_pkt_family(pkt) == AF_INET6) {
		protocol[0] = PPP_IPV6 & 0xFF;
		protocol[1] = (PPP_IPV6 >> 8) & 0xFF;

		return 0;
	}

	if ((net_pkt_family(pkt) == AF_PACKET) && (IS_ENABLED(CONFIG_NET_SOCKETS_PACKET))) {
		uint8_t type = (NET_IPV6_HDR(pkt)->vtc & 0xf0);

		if (type == 0x40) {
			protocol[0] = PPP_IP & 0xFF;
			protocol[1] = (PPP_IP >> 8) & 0xFF;

			return 0;
		}

		if (type == 0x60) {
			protocol[0] = PPP_IPV6 & 0xFF;
			protocol[1] = (PPP_IPV6 >> 8) & 0xFF;

			return 0;
		}
	}

	return -EPROTONOSUPPORT;
}

static uint16_t modem_ppp_wrap_write_byte(uint8_t *buf, uint16_t size, uint8_t byte)
{
	if (size < 1) {
		return 0;
	}

	if ((byte == 0x7E) || (byte == 0x7D) || (byte < 0x20)) {
		if (size < 2) {
			return 0;
		}

		buf[0] = 0x7D;
		buf[1] = byte ^ 0x20;

		return 2;
	}

	buf[0] = byte;

	return 1;
}

static uint16_t modem_ppp_fcs_init(const uint8_t *buf, uint16_t len)
{
	return crc16_ccitt(0xFFFF, buf, len);
}

static uint16_t modem_ppp_fcs_update(uint16_t fcs, const uint8_t *buf, uint16_t len)
{
	return crc16_ccitt(fcs, buf, len);
}

static void modem_ppp_fcs_final(uint8_t *fcs_bytes, uint16_t fcs)
{
	fcs = fcs ^ 0xFFFF;
	fcs_bytes[0] = fcs & 0xFF;
	fcs_bytes[1] = (fcs >> 8) & 0xFF;
}

/* The following steps are required to wrap a networking packet
 *
 * 1. Compute the FCS, which covers the PPP frame after the SOF byte, before escaping
 *    illegal bytes.
 * 2. Write SOF to buffer
 * 3. Write PPP framce contents including the FCS to the buffer escaping illegal bytes
 *    in the process.
 * 4. Write EOF
 *
 * Note that PPP frames already contain the appropriate protocol in the header of
 * their net buffer. Normal net packets must have the protocol added.
 */
static int modem_ppp_wrap_net_pkt(struct modem_ppp *ppp, struct net_pkt *pkt)
{
	int ret;
	uint16_t pos = 0;
	uint8_t protocol[2];
	uint16_t fcs;
	uint8_t fcs_bytes[2];

	/* Get protocol */
	ret = modem_ppp_get_protocol(protocol, pkt);
	if (ret == -EPROTONOSUPPORT) {
		return ret;
	}

	/* Initialize FCS */
	fcs = modem_ppp_fcs_init(ppp_frame_header, sizeof(ppp_frame_header));

	/* Write SOF flag to buffer */
	ppp->tx_buf[pos] = MODEM_PPP_FRAME_SOF;
	pos++;

	/* Write header to buffer */
	pos += modem_ppp_wrap_write_byte(&ppp->tx_buf[pos], ppp->tx_buf_size - pos,
					 ppp_frame_header[0]);
	pos += modem_ppp_wrap_write_byte(&ppp->tx_buf[pos], ppp->tx_buf_size - pos,
					 ppp_frame_header[1]);

	/* Write protocol if neccesary */
	if (ret == 0) {
		/* Update FCS with protocol */
		fcs = modem_ppp_fcs_update(fcs, protocol, sizeof(protocol));

		/* Write protocol to buffer */
		pos += modem_ppp_wrap_write_byte(&ppp->tx_buf[pos], ppp->tx_buf_size - pos,
						 protocol[0]);
		pos += modem_ppp_wrap_write_byte(&ppp->tx_buf[pos], ppp->tx_buf_size - pos,
						 protocol[1]);
	}

	/* Update FCS with network packet buffer */
	fcs = modem_ppp_fcs_update(fcs, pkt->buffer->data, pkt->buffer->len);

	/* Write network packet buffer to buffer */
	for (uint16_t i = 0; i < pkt->buffer->len; i++) {
		pos += modem_ppp_wrap_write_byte(&ppp->tx_buf[pos], ppp->tx_buf_size - pos,
						 pkt->buffer->data[i]);
	}

	/* Finalize FCS */
	modem_ppp_fcs_final(fcs_bytes, fcs);

	/* Write FCS */
	pos += modem_ppp_wrap_write_byte(&ppp->tx_buf[pos], ppp->tx_buf_size - pos, fcs_bytes[0]);
	pos += modem_ppp_wrap_write_byte(&ppp->tx_buf[pos], ppp->tx_buf_size - pos, fcs_bytes[1]);

	/* EOF */
	ppp->tx_buf[pos] = MODEM_PPP_FRAME_SOF;
	pos++;

	/* Store lenght of PPP frame */
	ppp->tx_buf_len = pos;

	return 0;
}

static int modem_ppp_send_net_pkt(struct modem_ppp *ppp)
{
	int ret;
	size_t sent = 0;

	while (sent < ppp->tx_buf_len) {
		ret = modem_pipe_transmit(ppp->pipe, &ppp->tx_buf[sent], (ppp->tx_buf_len - sent));

		if (ret < 0) {
			return ret;
		}

		if (ret == 0) {
			k_yield();

			continue;
		}

		sent += (size_t)ret;
	}

	return 0;
}

static bool modem_ppp_receive_data(struct modem_ppp *ppp)
{
	return false;

	int ret;

	/* Receive data from pipe */
	ret = modem_pipe_receive(ppp->pipe, ppp->rx_buf, ppp->rx_buf_size);

	/* Validate data received */
	if (ret <= 0) {
		return false;
	}

	/* Store lenght of receive buffer */
	ppp->rx_buf_len = (uint32_t)ret;

	return true;
}

static void modem_ppp_process_received_byte(struct modem_ppp *ppp, uint8_t byte)
{
	switch (ppp->receive_state) {
	case MODEM_PPP_RECEIVE_STATE_HDR_SOF:
		if (byte == 0x7E) {
			ppp->receive_state = MODEM_PPP_RECEIVE_STATE_HDR_FF;
		}

		break;

	case MODEM_PPP_RECEIVE_STATE_HDR_FF:
		if (byte == 0xFF) {
			ppp->receive_state = MODEM_PPP_RECEIVE_STATE_HDR_7D;

		} else {
			ppp->receive_state = MODEM_PPP_RECEIVE_STATE_HDR_SOF;
		}

		break;

	case MODEM_PPP_RECEIVE_STATE_HDR_7D:
		if (byte == 0x7D) {
			ppp->receive_state = MODEM_PPP_RECEIVE_STATE_HDR_23;

		} else {
			ppp->receive_state = MODEM_PPP_RECEIVE_STATE_HDR_SOF;
		}

		break;

	case MODEM_PPP_RECEIVE_STATE_HDR_23:
		if (byte == 0x23) {
			ppp->pkt = net_pkt_rx_alloc_with_buffer(ppp->net_iface, 256, AF_UNSPEC, 0,
								K_NO_WAIT);

			if (ppp->pkt == NULL) {
				ppp->receive_state = MODEM_PPP_RECEIVE_STATE_HDR_SOF;
			} else {
				LOG_DBG("Receiving PPP frame -> net_pkt(0x%08x)", (size_t)ppp->pkt);

				ppp->receive_state = MODEM_PPP_RECEIVE_STATE_WRITING;
			}

			net_pkt_cursor_init(ppp->pkt);

		} else {
			ppp->receive_state = MODEM_PPP_RECEIVE_STATE_HDR_SOF;
		}

		break;

	case MODEM_PPP_RECEIVE_STATE_WRITING:
		if (byte == 0x7E) {
			LOG_DBG("Received PPP frame -> net_pkt(0x%08x)", (size_t)ppp->pkt);

			/* Remove FCS */
			net_pkt_remove_tail(ppp->pkt, MODEM_PPP_FRAME_TAIL_SIZE);

			/* Receive on network interface */
			net_recv_data(ppp->net_iface, ppp->pkt);

			/* Remove reference to network packet */
			ppp->pkt = NULL;

			/* Reset state */
			ppp->receive_state = MODEM_PPP_RECEIVE_STATE_HDR_SOF;

			break;
		}

		if (byte == 0x7D) {
			ppp->receive_state = MODEM_PPP_RECEIVE_STATE_UNESCAPING;

			break;
		}

		if (net_pkt_write_u8(ppp->pkt, byte) < 0) {
			net_pkt_unref(ppp->pkt);

			ppp->pkt = NULL;

			ppp->receive_state = MODEM_PPP_RECEIVE_STATE_HDR_SOF;
		}

		break;

	case MODEM_PPP_RECEIVE_STATE_UNESCAPING:
		if (net_pkt_write_u8(ppp->pkt, (byte ^ 0x20)) < 0) {
			net_pkt_unref(ppp->pkt);

			ppp->pkt = NULL;

			ppp->receive_state = MODEM_PPP_RECEIVE_STATE_HDR_SOF;

			break;
		}

		ppp->receive_state = MODEM_PPP_RECEIVE_STATE_WRITING;

		break;
	}
}

static void modem_ppp_process_received_data(struct modem_ppp *ppp)
{
	for (size_t i = 0; i < ppp->rx_buf_len; i++) {
		modem_ppp_process_received_byte(ppp, ppp->rx_buf[i]);
	}
}

static void modem_ppp_pipe_event_handler(struct modem_pipe *pipe, enum modem_pipe_event event,
					 void *user_data)
{
	struct modem_ppp *ppp = (struct modem_ppp *)user_data;

	/* Validate receive ready */
	if (event != MODEM_PIPE_EVENT_RECEIVE_READY) {
		return;
	}

	k_mutex_lock(&ppp->lock, K_FOREVER);

	/* Validate attached */
	if (ppp->attached == false) {
		k_mutex_unlock(&ppp->lock);
		return;
	}

	/* Receive and process available data */
	while (modem_ppp_receive_data(ppp) == true) {
		modem_ppp_process_received_data(ppp);
	}

	k_mutex_unlock(&ppp->lock);
}

/*********************************************************
 * GLOBAL FUNCTIONS
 *********************************************************/
int modem_ppp_init(struct modem_ppp *ppp, struct modem_ppp_config *config)
{
	/* Validate arguments */
	if ((ppp == NULL) || (config == NULL)) {
		return -EINVAL;
	}

	/* Validate configuration */
	if ((config->rx_buf == NULL) || (config->rx_buf_size == 0) || (config->tx_buf == NULL) ||
	    (config->tx_buf_size == 0)) {
		return -EINVAL;
	}

	/* Clear PPP instance */
	memset(ppp, 0x00, sizeof(*ppp));

	/* Copy configuration */
	ppp->rx_buf = config->rx_buf;
	ppp->rx_buf_size = config->rx_buf_size;
	ppp->tx_buf = config->tx_buf;
	ppp->tx_buf_size = config->tx_buf_size;

	/* Initialize mutex */
	k_mutex_init(&ppp->lock);

	return 0;
}

int modem_ppp_attach(struct modem_ppp *ppp, struct modem_pipe *pipe, struct net_if *iface)
{
	int ret;

	/* Validate arguments */
	if ((ppp == NULL) || (pipe == NULL) || (iface == NULL)) {
		return -EINVAL;
	}

	k_mutex_lock(&ppp->lock, K_FOREVER);

	/* Associate command handler with pipe and network interface */
	ppp->net_iface = iface;
	ppp->pipe = pipe;

	/* Set pipe event handler */
	ret = modem_pipe_event_handler_set(pipe, modem_ppp_pipe_event_handler, ppp);

	if (ret < 0) {
		ppp->net_iface = NULL;
		ppp->pipe = NULL;

		k_mutex_unlock(&ppp->lock);

		return ret;
	}

	/* Update state */
	ppp->attached = true;

	LOG_DBG("Attached: pipe(0x%08x), iface(0x%08x)", (size_t)ppp->pipe, (size_t)ppp->net_iface);

	k_mutex_unlock(&ppp->lock);

	return 0;
}

int modem_ppp_send(struct modem_ppp *ppp, struct net_pkt *pkt)
{
	int ret;

	/* Validate arguments */
	if (ppp == NULL || pkt == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&ppp->lock, K_FOREVER);

	/* Validate pipe attached */
	if (ppp->attached == false) {
		k_mutex_unlock(&ppp->lock);

		return -EPERM;
	}

	LOG_DBG("Sending: net_pkt(0x%08x)", (size_t)pkt);

	/* Wrap network packet */
	ret = modem_ppp_wrap_net_pkt(ppp, pkt);

	if (ret < 0) {
		k_mutex_unlock(&ppp->lock);

		return ret;
	}

	ret = modem_ppp_send_net_pkt(ppp);

	if (ret < 0) {
		LOG_DBG("Send failed: net_pkt(0x%08x)", (size_t)pkt);

		k_mutex_unlock(&ppp->lock);

		return ret;
	}

	LOG_DBG("Sent: net_pkt(0x%08x)", (size_t)pkt);

	return 0;
}

int modem_ppp_release(struct modem_ppp *ppp)
{
	/* Validate arguments */
	if ((ppp->pipe == NULL)) {
		return 0;
	}

	k_mutex_lock(&ppp->lock, K_FOREVER);

	modem_pipe_event_handler_set(ppp->pipe, NULL, NULL);

	/* Clear references to pipe and network interface */
	ppp->net_iface = NULL;
	ppp->pipe = NULL;

	/* Unreference network packet if exists */
	if (ppp->pkt != NULL) {
		net_pkt_unref(ppp->pkt);
		ppp->pkt = NULL;
	}

	/* Update state */
	ppp->attached = false;

	LOG_DBG("Released");

	k_mutex_unlock(&ppp->lock);

	return 0;
}

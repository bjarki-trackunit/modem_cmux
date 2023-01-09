#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>

#include "modem_pipe.h"

#ifndef ZEPHYR_DRIVERS_MODEM_MODEM_PPP
#define ZEPHYR_DRIVERS_MODEM_MODEM_PPP

enum modem_ppp_receive_state {
	/* Searching for start of frame and header */
	MODEM_PPP_RECEIVE_STATE_HDR_SOF = 0,
	MODEM_PPP_RECEIVE_STATE_HDR_FF,
	MODEM_PPP_RECEIVE_STATE_HDR_7D,
	MODEM_PPP_RECEIVE_STATE_HDR_23,
	/* Writing bytes to network packet */
	MODEM_PPP_RECEIVE_STATE_WRITING,
	/* Unescaping next byte before writing to network packet */
	MODEM_PPP_RECEIVE_STATE_UNESCAPING,
};

struct modem_ppp {
	/* Network interface receiving PPP network packets */
	struct net_if *net_iface;

	/* Wrapped PPP frames are sent and received through this pipe */
	struct modem_pipe *pipe;

	/* Buffer used for processing received data */
	uint8_t *rx_buf;
	size_t rx_buf_len;
	size_t rx_buf_size;

	/* Receive PPP frame state */
	enum modem_ppp_receive_state receive_state;

	/* Allocated network packet being created */
	struct net_pkt *pkt;

	/* Network packet to send is wrapped into in this buffer */
	uint8_t *tx_buf;
	size_t tx_buf_len;
	size_t tx_buf_size;

	/* Synchronize */
	struct k_mutex lock;

	/* Indicate if attached */
	bool attached;
};

/**
 * @brief Contains modem PPP instance confuguration data
 * @param rx_buf Buffer for unwrapping PPP frame
 * @param rx_buf_size Size of RX buffer, should be the same size as network buffer
 * @param tx_buf Buffer for wrapping network packet in PPP frame
 * @param tx_buf_size Size of TX buffer, which should be twice the size of network buffer
 */
struct modem_ppp_config {
	uint8_t *rx_buf;
	size_t rx_buf_size;
	uint8_t *tx_buf;
	size_t tx_buf_size;
};

int modem_ppp_init(struct modem_ppp *ppp, struct modem_ppp_config *config);

/**
 * @brief Attach pipe to instance
 * @param ppp Modem PPP instance
 * @param pipe Pipe to attach to modem PPP instance
 * @param iface Network interface receiving PPP network packets
 */
int modem_ppp_attach(struct modem_ppp *ppp, struct modem_pipe *pipe, struct net_if *iface);

/**
 * @brief Wrap network packet in PPP frame and send it
 * @param ppp Modem PPP context
 * @param pkt Network packet to be wrapped and sent
 */
int modem_ppp_send(struct modem_ppp *ppp, struct net_pkt *pkt);

/**
 * @brief Release pipe from instance
 * @param ppp Modem PPP instance
 */
int modem_ppp_release(struct modem_ppp *ppp);

#endif /* ZEPHYR_DRIVERS_MODEM_MODEM_PPP */

/*
 * Copyright (c) 2022 Trackunit Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/device.h>

#include <zephyr/net/socket.h>
#include <zephyr/net/net_ip.h>

#include <string.h>

#include "modem_pipe_uart.h"
#include "modem_cmux.h"
#include "modem_cmd.h"
#include "modem_ppp.h"

#include "zephyr/net/ppp.h"
#include "zephyr/net/net_l2.h"

#define HTTP_PORT 80
#define HTTP_HOST "www.google.com"
#define HTTP_PATH "/"
#define SSTRLEN(s) (sizeof(s) - 1)
#define REQUEST "GET " HTTP_PATH " HTTP/1.0\r\nHost: " HTTP_HOST "\r\n\r\n"
static char recv_buf[1024];

/* Device tree handles */
const struct gpio_dt_spec en1_gpio = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), en1_gpios);
const struct gpio_dt_spec en2_gpio = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), en2_gpios);
const struct gpio_dt_spec mdm_power_gpio = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), mdm_power_gpios);

struct quectel_bgxx_config {
	const struct device *uart;
};

struct quectel_bgxx_data {
	/* UART bus */
	struct modem_pipe_uart bus_pipe_uart;
	struct modem_pipe bus_pipe;
	uint8_t bus_pipe_rx_buf[256];
	uint8_t bus_pipe_tx_buf[256];

	/* Command parser */
	struct modem_cmd cmd;
	uint8_t cmd_receive_buf[128];
	uint8_t cmd_delimiter[2];
	struct k_event cmd_events;

	/* Networking */
	struct net_if *net_iface;
	uint8_t net_link_addr[6];

	/* CMUX */
	struct modem_cmux cmux;
	struct modem_cmux_dlci cmux_dlcis[3];
	uint8_t cmux_receive_buf[256];

	/* CMUX DLCI channel pipes */
	struct modem_pipe ppp_pipe;
	uint8_t ppp_dlci_receive_buf[256U];
	struct modem_pipe at_pipe;
	uint8_t at_dlci_receive_buf[128U];

	/* CMUX events */
	struct k_event cmux_events;

	/* PPP wrapping */
	struct modem_ppp ppp;
	uint8_t ppp_rx_buf[256];
	uint8_t ppp_tx_buf[256];

	/* Synchronization */
	struct k_mutex lock;
};

uint8_t ppp_tx_buf[256];
uint8_t ppp_rx_buf[256];
uint16_t ppp_rx_buf_len = 0;
uint8_t response_test[128];

#define QUECTEL_BGXX_PPP_DLCI_ADDRESS (1)
#define QUECTEL_BGXX_AT_DLCI_ADDRESS  (2)

#define EVENT_CMUX_CONNECTED       (BIT(1))
#define EVENT_CMUX_PPP_DLCI_OPEN   (BIT(2))
#define EVENT_CMUX_AT_DLCI_OPEN    (BIT(3))
#define EVENT_CMUX_PPP_DLCI_CLOSED (BIT(4))
#define EVENT_CMUX_AT_DLCI_CLOSED  (BIT(5))
#define EVENT_CMUX_DISCONNECTED    (BIT(6))

#define EVENT_CMD_AT_RDY           (BIT(0))
#define EVENT_CMD_AT_APP_RDY       (BIT(1))
#define EVENT_CMD_AT_POWER_DOWN    (BIT(2))
#define EVENT_CMD_AT_OK            (BIT(3))
#define EVENT_CMD_AT_CONNECT       (BIT(4))

/* Lazy command handler (should be written to use struct modem_cmd like zephyr/drivers/modem_cmd_handler.h) */
void quectel_bgxx_cmd_on_received(const uint8_t *buf, uint16_t size, void* user_data)
{
	struct quectel_bgxx_data *data = (struct quectel_bgxx_data *)user_data;

	for (uint16_t i = 0; i < size; i++) {
		printk("%c", buf[i]);
	}

	if (size == (sizeof("RDY\r\n") - 1)) {
		if (memcmp(buf, "RDY\r\n", sizeof("RDY\r\n") - 1) == 0) {
			k_event_post(&data->cmd_events, EVENT_CMD_AT_RDY);
			return;
		}
	}

	if (size == (sizeof("APP RDY\r\n") - 1)) {
		if (memcmp(buf, "APP RDY\r\n", sizeof("APP RDY\r\n") - 1) == 0) {
			k_event_post(&data->cmd_events, EVENT_CMD_AT_APP_RDY);
			return;
		}
	}

	if (size == (sizeof("NORMAL POWER DOWN\r\n") - 1)) {
		if (memcmp(buf, "NORMAL POWER DOWN\r\n", sizeof("NORMAL POWER DOWN\r\n") - 1) == 0) {
			k_event_post(&data->cmd_events, EVENT_CMD_AT_POWER_DOWN);
			return;
		}
	}

	if (size == (sizeof("OK\r\n") - 1)) {
		if (memcmp(buf, "OK\r\n", sizeof("OK\r\n") - 1) == 0) {
			k_event_post(&data->cmd_events, EVENT_CMD_AT_OK);
			return;
		}
	}

	if (size == (sizeof("CONNECT 150000000\r\n") - 1)) {
		if (memcmp(buf, "CONNECT 150000000\r\n", sizeof("CONNECT 150000000\r\n") - 1) == 0) {
			k_event_post(&data->cmd_events, EVENT_CMD_AT_CONNECT);
			return;
		}
	}
}

void quectel_bgxx_cmux_event_handler(struct modem_cmux *cmux, struct modem_cmux_event event,
				     void *user_data)
{
    struct quectel_bgxx_data *data = (struct quectel_bgxx_data *)user_data;

    if (event.type == MODEM_CMUX_EVENT_CONNECTED) {
	k_event_post(&data->cmux_events, EVENT_CMUX_CONNECTED);
	return;
    }

    if ((event.type == MODEM_CMUX_EVENT_OPENED) &&
	(event.dlci_address == QUECTEL_BGXX_PPP_DLCI_ADDRESS)) {
	k_event_post(&data->cmux_events, EVENT_CMUX_PPP_DLCI_OPEN);
	return;
    }

    if ((event.type == MODEM_CMUX_EVENT_OPENED) &&
	(event.dlci_address == QUECTEL_BGXX_AT_DLCI_ADDRESS)) {
	k_event_post(&data->cmux_events, EVENT_CMUX_AT_DLCI_OPEN);
	return;
    }

    if ((event.type == MODEM_CMUX_EVENT_CLOSED) &&
	(event.dlci_address == QUECTEL_BGXX_PPP_DLCI_ADDRESS)) {
	k_event_post(&data->cmux_events, EVENT_CMUX_PPP_DLCI_CLOSED);
	return;
    }

    if ((event.type == MODEM_CMUX_EVENT_CLOSED) &&
	(event.dlci_address == QUECTEL_BGXX_AT_DLCI_ADDRESS)) {
	k_event_post(&data->cmux_events, EVENT_CMUX_AT_DLCI_CLOSED);
	return;
    }

    if (event.type == MODEM_CMUX_EVENT_DISCONNECTED) {
	k_event_post(&data->cmux_events, EVENT_CMUX_DISCONNECTED);
	return;
    }
}

/* Called once to initialize driver by kernel */
int quectel_bgxx_init(const struct device *dev)
{
	struct quectel_bgxx_data *data = (struct quectel_bgxx_data *)dev->data;
	struct quectel_bgxx_config *config = (struct quectel_bgxx_config *)dev->config;
	int ret;

	/* Mutex */
	k_mutex_init(&data->lock);

	/* Events */
	k_event_init(&data->cmux_events);
	k_event_init(&data->cmd_events);

	/* UART bus pipe */
	struct modem_pipe_uart_config bus_pipe_uart_config = {
		.uart = config->uart,
		.rx_buf = data->bus_pipe_rx_buf,
		.rx_buf_size = sizeof(data->bus_pipe_rx_buf),
		.tx_buf = data->bus_pipe_tx_buf,
		.tx_buf_size = sizeof(data->bus_pipe_tx_buf),
	};

	ret = modem_pipe_uart_init(&data->bus_pipe_uart, &bus_pipe_uart_config);
	if (ret < 0) {
		return ret;
	}

	ret = modem_pipe_uart_open(&data->bus_pipe_uart, &data->bus_pipe);
	if (ret < 0) {
		return ret;
	}

	/* Command parser */
	memcpy(data->cmd_delimiter, "\r\n", sizeof("\r\n") - 1);

	struct modem_cmd_config cmd_config = {
		.cmd_received = quectel_bgxx_cmd_on_received,
		.user_data = data,
		.receive_buf = data->cmd_receive_buf,
		.receive_buf_size = sizeof(data->cmd_receive_buf),
		.delimiter = data->cmd_delimiter,
		.delimiter_size = sizeof(data->cmd_delimiter),
		.idle_timeout = K_MSEC(5),
	};

	ret = modem_cmd_init(&data->cmd, &cmd_config);
	if (ret < 0) {
		return ret;
	}

	/* CMUX */
	struct modem_cmux_config cmux_config = {
		.event_handler = quectel_bgxx_cmux_event_handler,
		.event_handler_user_data = data,
		.dlcis = data->cmux_dlcis,
		.dlcis_cnt = ARRAY_SIZE(data->cmux_dlcis),
		.receive_buf = data->cmux_receive_buf,
		.receive_buf_size = sizeof(data->cmux_receive_buf),
	};

	ret = modem_cmux_init(&data->cmux, &cmux_config);
	if (ret < 0) {
		return ret;
	}

	/* Initialize PPP */
	struct modem_ppp_config ppp_config = {
		.rx_buf = data->ppp_rx_buf,
		.rx_buf_size = sizeof(data->ppp_rx_buf),
		.tx_buf = data->ppp_tx_buf,
		.tx_buf_size = sizeof(data->ppp_tx_buf),
	};

	ret = modem_ppp_init(&data->ppp, &ppp_config);
	if (ret < 0) {
		return ret;
	}

	/* Attach modem command handler to bus pipe */
	ret = modem_cmd_attach(&data->cmd, &data->bus_pipe);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

/* Power on modem, enable CMUX */
int quectel_bgxx_resume(const struct device *dev)
{
	struct quectel_bgxx_data *data = (struct quectel_bgxx_data *)dev->data;
	int ret;
	uint32_t events;

	struct modem_cmux_dlci_config ppp_dlci_config = {
		.dlci_address = QUECTEL_BGXX_PPP_DLCI_ADDRESS,
		.receive_buf = data->ppp_dlci_receive_buf,
		.receive_buf_size = sizeof(data->ppp_dlci_receive_buf),
	};

	struct modem_cmux_dlci_config at_dlci_config = {
		.dlci_address = QUECTEL_BGXX_AT_DLCI_ADDRESS,
		.receive_buf = data->at_dlci_receive_buf,
		.receive_buf_size = sizeof(data->at_dlci_receive_buf),
	};

	/* Do hardware resume here */
	/* ... */

	k_mutex_lock(&data->lock, K_FOREVER);

	ret = modem_pipe_transmit(&data->bus_pipe, "AT+CMUX=0,0,5\r\n", sizeof("AT+CMUX=0,0,5\r\n") - 1);
	if (ret < sizeof("AT+CMUX=0,0,5\r\n") - 1) {
		k_mutex_unlock(&data->lock);
		return -EAGAIN;
	}

	events = k_event_wait(&data->cmd_events,
				  (EVENT_CMD_AT_OK),
				  true, K_MSEC(1000));

	if ((events & EVENT_CMD_AT_OK) == 0) {
		k_mutex_unlock(&data->lock);
		return -EAGAIN;
	}

	/* Release bus pipe currently used by command handler */
	ret = modem_cmd_release(&data->cmd);
	if (ret < 0) {
		k_mutex_unlock(&data->lock);
		return ret;
	}

	/* Wait for BGXX switch to CMUX mode */
	k_msleep(300);

	ret = modem_cmux_connect(&data->cmux, &data->bus_pipe);
	if (ret < 0) {
		k_mutex_unlock(&data->lock);
		return ret;
	}

	events = k_event_wait(&data->cmux_events, (EVENT_CMUX_CONNECTED), true, K_MSEC(330));

	if (events == 0) {
	    k_mutex_unlock(&data->lock);
	    return -EAGAIN;
	}

	ret = modem_cmux_dlci_open(&data->cmux, &ppp_dlci_config, &data->ppp_pipe);
	if (ret < 0) {
		k_mutex_unlock(&data->lock);
		return ret;
	}

	ret = modem_cmux_dlci_open(&data->cmux, &at_dlci_config, &data->at_pipe);
	if (ret < 0) {
		k_mutex_unlock(&data->lock);
		return ret;
	}

	events = k_event_wait_all(&data->cmux_events,
				  (EVENT_CMUX_PPP_DLCI_OPEN | EVENT_CMUX_AT_DLCI_OPEN),
				  true, K_MSEC(330));

	if (((events & EVENT_CMUX_PPP_DLCI_OPEN) == 0) ||
	    ((events & EVENT_CMUX_AT_DLCI_OPEN) == 0)) {
		k_mutex_unlock(&data->lock);
		return -EAGAIN;
	}

	/* Set PPP DLCI channel pipe event handler */
	/* ... */

	/* Attach AT DLCI channel pipe to command handler */
	ret = modem_cmd_attach(&data->cmd, &data->at_pipe);
	if (ret < 0) {
		k_mutex_unlock(&data->lock);
		return ret;
	}

	k_mutex_unlock(&data->lock);

	return 0;
}

/* Disconnect CMUX, modem to low power state */
static int quectel_bgxx_suspend(const struct device *dev)
{
	struct quectel_bgxx_data *data = (struct quectel_bgxx_data *)dev->data;
	int ret;
	uint32_t events;

	k_mutex_lock(&data->lock, K_FOREVER);

	ret = modem_cmux_dlci_close(&data->ppp_pipe);
	if (ret < 0) {
		k_mutex_unlock(&data->lock);
		return ret;
	}

	ret = modem_cmux_dlci_close(&data->at_pipe);
	if (ret < 0) {
		k_mutex_unlock(&data->lock);
		return ret;
	}

	events = k_event_wait_all(&data->cmux_events,
				  (EVENT_CMUX_PPP_DLCI_CLOSED | EVENT_CMUX_AT_DLCI_CLOSED),
				  true, K_MSEC(330));

	if (((events & EVENT_CMUX_PPP_DLCI_CLOSED) == 0) ||
	    ((events & EVENT_CMUX_AT_DLCI_CLOSED) == 0)) {
		k_mutex_unlock(&data->lock);
		return -EAGAIN;
	}

	/* Immediate disconnect after closing DLCI channels causes modem reset on BG95 */
	k_msleep(50);

	ret = modem_cmux_disconnect(&data->cmux);
	if (ret < 0) {
		k_mutex_unlock(&data->lock);
		return ret;
	}

	events = k_event_wait(&data->cmux_events, (EVENT_CMUX_DISCONNECTED), true, K_MSEC(330));

	if (events == 0) {
	    k_mutex_unlock(&data->lock);
	    return -EAGAIN;
	}

	/* Wait for BGXX switch to AT command mode */
	k_msleep(500);

	/* Reattach modem command handler to bus pipe */
	ret = modem_cmd_attach(&data->cmd, &data->bus_pipe);
	if (ret < 0) {
		return ret;
	}

	/* Power down hardware */
	ret = modem_pipe_transmit(&data->bus_pipe, "AT+QPOWD\r\n", sizeof("AT+QPOWD\r\n") - 1);
	if (ret < sizeof("AT+QPOWD\r\n") - 1) {
		k_mutex_unlock(&data->lock);
		return -EAGAIN;
	}

	events = k_event_wait(&data->cmd_events,
				  (EVENT_CMD_AT_OK),
				  true, K_MSEC(1000));

	if ((events & EVENT_CMD_AT_OK) == 0) {
		k_mutex_unlock(&data->lock);
		return -EAGAIN;
	}

	k_mutex_unlock(&data->lock);

	return 0;
}

static void quectel_bgxx_net_if_api_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct quectel_bgxx_data *data = (struct quectel_bgxx_data *)dev->data;

	net_ppp_init(iface);

	net_if_flag_set(iface, NET_IF_NO_AUTO_START);

	/* Fixed test link address */
	data->net_link_addr[0] = 0x00;
	data->net_link_addr[1] = 0x00;
	data->net_link_addr[2] = 0x5E;
	data->net_link_addr[3] = 0x00;
	data->net_link_addr[4] = 0x53;
	data->net_link_addr[5] = 0x01;

	net_if_set_link_addr(iface, data->net_link_addr, sizeof(data->net_link_addr),
			     NET_LINK_ETHERNET);

	/* Store reference to network interface */
	data->net_iface= iface;
}

static int quectel_bgxx_net_if_ppp_api_start(const struct device *dev)
{
	struct quectel_bgxx_data *data = (struct quectel_bgxx_data *)dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);

	ret = modem_pipe_transmit(&data->ppp_pipe, "ATDT*99#\r\n", sizeof("ATDT*99#\r\n") - 1);

	if (ret < sizeof("ATDT*99#\r\n") - 1) {
		k_mutex_unlock(&data->lock);
		return -EAGAIN;
	}

	k_msleep(1000);

	ret = modem_ppp_attach(&data->ppp, &data->ppp_pipe, data->net_iface);
	if (ret < 0) {
		k_mutex_unlock(&data->lock);
		return -EAGAIN;
	}

	k_mutex_unlock(&data->lock);

	return 0;
}

static int quectel_bgxx_net_if_ppp_api_stop(const struct device *dev)
{
	struct quectel_bgxx_data *data = (struct quectel_bgxx_data *)dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);

	ret = modem_ppp_release(&data->ppp);
	if (ret < 0) {
		k_mutex_unlock(&data->lock);
		return -EAGAIN;
	}

	k_mutex_unlock(&data->lock);

	return 0;
}

static int quectel_bgxx_net_if_ppp_api_send(const struct device *dev, struct net_pkt *pkt)
{
	struct quectel_bgxx_data *data = (struct quectel_bgxx_data *)dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);

	ret = modem_ppp_send(&data->ppp, pkt);

	k_mutex_unlock(&data->lock);

	return 0;
}

static const struct ppp_api quectel_bgxx_net_if_ppp_api = {
	.iface_api.init = quectel_bgxx_net_if_api_init,
	.start = quectel_bgxx_net_if_ppp_api_start,
	.stop = quectel_bgxx_net_if_ppp_api_stop,
	.send = quectel_bgxx_net_if_ppp_api_send,
};

const struct quectel_bgxx_config quectel_bgxx_config0 = {
    .uart = DEVICE_DT_GET(DT_ALIAS(modem_uart)),
};

struct quectel_bgxx_data quectel_bgxx_data0;

NET_DEVICE_INIT(quectel_bgxx, "BGXX", quectel_bgxx_init,
		NULL, &quectel_bgxx_data0, &quectel_bgxx_config0,
		41, &quectel_bgxx_net_if_ppp_api,
		PPP_L2, NET_L2_GET_CTX_TYPE(PPP_L2), 1500);

int pins_init(const struct device *dev)
{
	/* IMPORTANT pins must be configured this way to use
	 * UART through the STM propriatary connectors
	 */
	gpio_pin_configure_dt(&en1_gpio, GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure_dt(&en2_gpio, GPIO_OUTPUT_ACTIVE);

	gpio_pin_configure_dt(&mdm_power_gpio, GPIO_OUTPUT_ACTIVE);
	k_msleep(1000);
	gpio_pin_set_dt(&mdm_power_gpio, 0);
	k_msleep(3000);
	gpio_pin_set_dt(&mdm_power_gpio, 1);
	k_msleep(300);
	gpio_pin_set_dt(&mdm_power_gpio, 0);
	k_msleep(7000);

	return 0;
}

SYS_INIT(pins_init, POST_KERNEL, 40);

static void event_handler(struct net_mgmt_event_callback*, uint32_t mgmt_event, struct net_if*)
{
	if (mgmt_event == NET_EVENT_L4_CONNECTED)
	{
		printk("Network connected");
	}

	if (mgmt_event == NET_EVENT_L4_DISCONNECTED)
	{
		printk("Network disconnected");
	}
}

int main(void)
{
	struct net_if *bgxx_iface = net_if_get_first_by_type(&NET_L2_GET_NAME(PPP));
	const struct device *bgxx = bgxx_iface->if_dev->dev;
	struct quectel_bgxx_data *bgxx_data = (struct quectel_bgxx_data *)bgxx->data;
	uint32_t events;
	int ret;

	/* Network status */
	static struct net_mgmt_event_callback mgmt_cb;

	net_mgmt_init_event_callback(&mgmt_cb, event_handler, NET_EVENT_L4_CONNECTED | NET_EVENT_L4_DISCONNECTED);
	net_mgmt_add_event_callback(&mgmt_cb);

	quectel_bgxx_resume(bgxx);

	ret = modem_pipe_transmit(&bgxx_data->at_pipe, "ATE0\r\n",
				  sizeof("ATE0\r\n") - 1);

	k_msleep(500);

	ret = modem_pipe_transmit(&bgxx_data->at_pipe, "ATH\r\n",
				  sizeof("ATH\r\n") - 1);

	k_msleep(500);

	ret = modem_pipe_transmit(&bgxx_data->at_pipe, "AT+CMEE=1\r\n",
				  sizeof("AT+CMEE=1\r\n") - 1);

	k_msleep(500);

	ret = modem_pipe_transmit(&bgxx_data->at_pipe, "AT+CREG=0\r\n",
				  sizeof("AT+CREG=0\r\n") - 1);

	k_msleep(500);

	ret = modem_pipe_transmit(&bgxx_data->at_pipe, "AT+CGDCONT=1,\"IP\",\"trackunit.m2m\",,0,0\r\n",
				  sizeof("AT+CGDCONT=1,\"IP\",\"trackunit.m2m\",,0,0\r\n") - 1);

	k_msleep(500);

	for (uint8_t i = 0; i < 5; i++) {
		ret = modem_pipe_transmit(&bgxx_data->at_pipe, "AT+CREG?\r\n",
					  sizeof("AT+CREG?\r\n") - 1);

		k_msleep(5000);
	}

	k_msleep(500);

	for (uint8_t i = 0; i < 5; i++) {
		ret = modem_pipe_transmit(&bgxx_data->at_pipe, "AT+CGATT?\r\n",
					  sizeof("AT+CGATT?\r\n") - 1);

		k_msleep(5000);
	}

	net_ppp_carrier_on(bgxx_iface);

	k_msleep(90000);


	/* Test some networking */
	struct zsock_addrinfo address;
	static struct sockaddr sock_address;
	volatile int sock_fd;

	address.ai_family = AF_INET;
	address.ai_protocol = IPPROTO_UDP;
	address.ai_socktype = SOCK_DGRAM;
	address.ai_addr = &sock_address;
	address.ai_addrlen = sizeof(sock_address);

	struct sockaddr_in * sockaddr_cast_ipv4 = net_sin(&sock_address);
	sockaddr_cast_ipv4->sin_family = AF_INET;
	sockaddr_cast_ipv4->sin_port = htons(80);
	ret = net_addr_pton(AF_INET, "142.250.185.228", &sockaddr_cast_ipv4->sin_addr);

	// Open socket
	sock_fd = zsock_socket(AF_INET, address.ai_socktype, address.ai_protocol);

	// Connect
	ret = zsock_connect(sock_fd, address.ai_addr, address.ai_addrlen);

	// Send
	ret = zsock_send(sock_fd, REQUEST, SSTRLEN(REQUEST), 0);

	k_msleep(10000);

	ret = zsock_recv(sock_fd, recv_buf, sizeof(recv_buf), 0);

	// Disconnect
	ret = zsock_close(sock_fd);

	k_msleep(10000);

	net_ppp_carrier_off(bgxx_iface);

	quectel_bgxx_suspend(bgxx);

	return 0;
}

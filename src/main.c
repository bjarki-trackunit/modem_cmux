#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/device.h>

#include <string.h>

#include "modem_pipe_uart.h"
#include "modem_cmux.h"
#include "modem_cmd.h"

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

        /* Synchronization */
        struct k_mutex lock;
};

const struct quectel_bgxx_config config = {
    .uart = DEVICE_DT_GET(DT_ALIAS(modem_uart)),
};

struct quectel_bgxx_data data;

struct device bgxx = {
    .name = "BGXX",
    .data = &data,
    .config = &config,
};

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

/* Lazy command handler (should be written to use struct modem_cmd like zephyr/drivers/modem_cmd_handler.h) */
void quectel_bgxx_cmd_on_received(const uint8_t *buf, uint16_t size, void* user_data)
{
        const struct device *dev = (const struct device *)user_data;
        struct quectel_bgxx_data *data = (struct quectel_bgxx_data *)dev->data;

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
}

void quectel_bgxx_cmux_event_handler(struct modem_cmux *cmux, struct modem_cmux_event event,
                                     void *user_data)
{
    const struct device *dev = (const struct device *)user_data;
    struct quectel_bgxx_data *data = (struct quectel_bgxx_data *)dev->data;

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
int quectel_bgxx_init(struct device *dev)
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
                .user_data = dev,
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
                .event_handler_user_data = dev,
                .dlcis = data->cmux_dlcis,
                .dlcis_cnt = ARRAY_SIZE(data->cmux_dlcis),
                .receive_buf = data->cmux_receive_buf,
                .receive_buf_size = sizeof(data->cmux_receive_buf),
        };

        ret = modem_cmux_init(&data->cmux, &cmux_config);
        if (ret < 0) {
                return ret;
        }

        return 0;
}

/* Will be called as response to net_if_up(ppp_iface) */
int quectel_bgxx_net_if_up(struct device *dev)
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

        /* Attach DLCI channel pipe to command handler */
        ret = modem_cmd_attach(&data->cmd, &data->at_pipe);
        if (ret < 0) {
                k_mutex_unlock(&data->lock);
                return ret;
        }

        k_mutex_unlock(&data->lock);
        return 0;
}

/* Will be called as response to net_if_down(ppp_iface) */
int quectel_bgxx_net_if_down(struct device *dev)
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

        k_mutex_unlock(&data->lock);
        return 0;
}

/*
 * Lazy reset of modem (modem must be poweren on before running this function)
 * ensuring known state before running main
 */
int quectel_bgxx_resume(struct device *dev)
{
        struct quectel_bgxx_data *data = (struct quectel_bgxx_data *)dev->data;
        int ret;

        /* Attach modem command handler to bus pipe */
        ret = modem_cmd_attach(&data->cmd, &data->bus_pipe);
        if (ret < 0) {
                return ret;
        }

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

int main(void)
{
        volatile int ret;
        uint32_t events;


        /* IMPORTANT pins must be configured this way to use
         * UART through the STM propriatary connectors
         */
        gpio_pin_configure_dt(&en1_gpio, GPIO_OUTPUT_ACTIVE);
        gpio_pin_configure_dt(&en2_gpio, GPIO_OUTPUT_ACTIVE);

        /* Modem configure */
        ret = quectel_bgxx_init(&bgxx);

        /* Power on modem (AT command mode through bus pipe) */
        ret = quectel_bgxx_resume(&bgxx);

        /* Enable CMUX,
         * connect DLCI 2 to AT pipe,
         * (in future) connect DLCI1 to PPP pipe,
         * enable PPP iface
         */
        ret = quectel_bgxx_net_if_up(&bgxx);

        struct quectel_bgxx_data *data = (struct quectel_bgxx_data *)bgxx.data;

        /* Test AT pipe */
        ret = modem_pipe_transmit(&data->at_pipe, "ATI\r\n", sizeof("ATI\r\n") - 1);

        events = k_event_wait(&data->cmd_events, EVENT_CMD_AT_OK, true, K_MSEC(1000));

        /* (in future) Test PPP pipe (adapter between pipe and net_if_ppp must be made)
        uint8_t pkt[20];
        ret = modem_pipe_transmit(&data->ppp_pipe, pkt, sizeof(pkt);
        */

        /* (in future) disable PPP iface,
         * close DLCIs, Disable CMUX,
         * Reattach cmd handler to bus pipe (command mode)
         */
        ret = quectel_bgxx_net_if_down(&bgxx);

        /* Test AT command mode sending through bus pipe directly */
        ret = modem_pipe_transmit(&data->bus_pipe, "ATI\r\n", sizeof("ATI\r\n") - 1);

        events = k_event_wait(&data->cmd_events, EVENT_CMD_AT_OK, true, K_MSEC(1000));

        return 0;
}

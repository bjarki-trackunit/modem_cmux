#ifndef ZEPHYR_DRIVERS_MODEM_SHELL
#define ZEPHYR_DRIVERS_MODEM_SHELL

#ifdef __cplusplus
extern "C" {
#endif

void tumodem_send_at_cmd(const char *atcmd_str);

#ifdef __cplusplus
}
#endif
#endif

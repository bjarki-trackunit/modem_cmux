#include "modem_shell.h"
#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>


/*********************************************************
 * DEFINES / MACROS
 *********************************************************/

/*********************************************************
 * TYPE DEFINITIONS
 *********************************************************/

/*********************************************************
 * LOCAL FUNCTIONS DECLARATION
 *********************************************************/
static int modem_send_cmd(const struct shell *shell, size_t argc, char **argv);

/*********************************************************
 * LOCAL VARIABLES
 *********************************************************/

/*********************************************************
 * LOCAL FUNCTIONS
 *********************************************************/
static int modem_send_cmd(const struct shell *shell, size_t argc, char **argv)
{
	/* list */

	if (2 == argc) {
		char cmd_to_send[30];
		int cmd_to_send_len = strlen(argv[1]);

		memcpy(cmd_to_send, argv[1], cmd_to_send_len);
		cmd_to_send[cmd_to_send_len++] = '\r';
		cmd_to_send[cmd_to_send_len++] = 0x00;
		tumodem_send_at_cmd((const char *)cmd_to_send);
		return 0;
	} else {
		shell_fprintf(shell, SHELL_ERROR, "Please enter AT command\n");
		return -EINVAL;
	}
}

SHELL_STATIC_SUBCMD_SET_CREATE(tumodem_cmd, SHELL_CMD(send, NULL, "Send AT cmd", modem_send_cmd),
			       SHELL_SUBCMD_SET_END /* Array terminated. */
);

SHELL_CMD_REGISTER(tumodem, &tumodem_cmd, "TU modem commands", NULL);

/*********************************************************
 * GLOBAL FUNCTIONS
 *********************************************************/

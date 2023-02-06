# Modem CMUX Zephyr POC
## Target
### Board
The application and its libraries can be built for any board which
has a UART. The .overlay files are used to configure the boards
hardware to match the applications requirenments.

Currently, there is only an overlay for the STM32 U585I IOT02A
dev board. It has some GPIOs that must be set to use UART on
their propriatary connectors, named en1 and en2 in the .overlay
file.

### Zephyr
This project is built for Zephyr 3.1.0.rc3, but it should also
work with upstream.

### Building application
1. Set up the Zephyr workspace using this guide [Guide](https://docs.zephyrproject.org/latest/develop/getting_started/index.html)
2. Clone this repo into folder containing zephyr, modules, .west, etc.
3. Replace #if defined(CONFIG_NET_PPP) line 1189 in net_pkt.h with #if defined(CONFIG_NET_L2_PPP)
4. Run command **export ZEPHYR_EXTRA_MODULES="$PWD/modem_cmux/out_of_tree"**
5. Build using **west build -p -b b_u585i_iot02a modem_cmux/**

### Debuggin application (on STM32 U585I IOT02A)
This board has an onboard STLINK, the guide for setting it up can be found
at this path relative to zephyr workspace **/zephyr/boards/arm/b_u585i_iot02a/doc/index.rst**

### Disclaimer
This app is in a very early, barely stable state, yet it demonstrates the concepts
quite well.

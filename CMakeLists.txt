# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(net_test)

target_sources(app PRIVATE src/main.c src/modem_pipe_uart.c src/modem_cmux.c src/modem_cmd.c src/modem_ppp.c src/modem_shell.cpp)
target_include_directories(app PRIVATE inc)

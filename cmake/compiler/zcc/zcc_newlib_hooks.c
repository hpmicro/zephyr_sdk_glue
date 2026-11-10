/*
 * Copyright (c) 2026 HPMicro
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/toolchain.h>
#include <zephyr/sys/util.h>

#include <stdio.h>

static int stdout_hook_default(int c)
{
	ARG_UNUSED(c);

	return EOF;
}

static int (*stdout_hook)(int) = stdout_hook_default;

void __stdout_hook_install(int (*hook)(int))
{
	stdout_hook = hook;
}

static unsigned char stdin_hook_default(void)
{
	return 0;
}

static unsigned char (*stdin_hook)(void) = stdin_hook_default;

void __stdin_hook_install(unsigned char (*hook)(void))
{
	stdin_hook = hook;
}

__weak int _read(int fd, void *buf, int nbytes)
{
	char *buffer = buf;
	int i;

	ARG_UNUSED(fd);

	for (i = 0; i < nbytes; i++) {
		buffer[i] = stdin_hook();
		if ((buffer[i] == '\n') || (buffer[i] == '\r')) {
			return i + 1;
		}
	}

	return nbytes;
}

__weak int _write(int fd, const void *buf, int nbytes)
{
	const char *buffer = buf;

	ARG_UNUSED(fd);

	for (int i = 0; i < nbytes; i++) {
		if (buffer[i] == '\n') {
			stdout_hook('\r');
		}
		stdout_hook(buffer[i]);
	}

	return nbytes;
}

__weak FUNC_NORETURN void _exit(int status)
{
	ARG_UNUSED(status);

	for (;;) {
	}
}

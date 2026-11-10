# Copyright (c) 2026 HPMicro
# SPDX-License-Identifier: Apache-2.0

"""Adapt Zephyr's OpenOCD and J-Link runners to use ZDB for ZCC builds."""

import os
import subprocess
from pathlib import Path

from runners.core import FileType
from runners.jlink import JLinkBinaryRunner, MISSING_REQUIREMENTS
from runners.openocd import OpenOcdBinaryRunner


_openocd_do_run = OpenOcdBinaryRunner.do_run
_jlink_do_run = JLinkBinaryRunner.do_run


def _uses_zdb(runner):
    return bool(runner.cfg.gdb and Path(runner.cfg.gdb).stem.lower() == 'zdb')


def _require_zdb(runner):
    if runner.tui_arg:
        raise ValueError('ZDB does not support the GDB --tui option')


def _openocd_zdb_do_run(self, command, **kwargs):
    if not _uses_zdb(self) or command not in ('debug', 'attach'):
        return _openocd_do_run(self, command, **kwargs)

    _require_zdb(self)
    if self.elf_name is None:
        raise ValueError('Cannot debug; no .elf specified')
    self.require(self.openocd_cmd[0])
    self.cfg_cmd = []
    if self.openocd_config is not None:
        for config in self.openocd_config:
            self.cfg_cmd.extend(['-f', config])

    # ZDB does not implement GDB's load command. Program the image with
    # OpenOCD first, then launch an interactive ZDB session.
    if command == 'debug' and self.load_arg:
        self.do_flash(**kwargs)

    pre_init_cmd = self._openocd_cmd(self.pre_init)
    if self.thread_info_enabled and self.supports_thread_info():
        pre_init_cmd.extend(['-c', f'${self.target_handle} configure -rtos Zephyr'])

    server_cmd = (
        self.openocd_cmd
        + self.serial
        + self.cfg_cmd
        + [
            '-c', f'tcl_port {self.tcl_port}',
            '-c', f'telnet_port {self.telnet_port}',
            '-c', f'gdb_port {self.gdb_port}',
        ]
        + pre_init_cmd
        + self.init_arg
        + self.targets_arg
        + self.halt_arg
    )

    if self.rtt_server:
        rtt_address = self.get_rtt_address()
        if rtt_address is None:
            raise ValueError('RTT Control block not found')
        server_cmd += [
            '-c', f'rtt setup 0x{rtt_address:x} 0x10 "SEGGER RTT"',
            '-c', 'rtt start',
            '-c', f'rtt server start {self.rtt_port} 0',
        ]

    client_cmd = [
        self.cfg.gdb,
        self.elf_name,
        '-o', f'gdb-remote localhost:{self.gdb_client_port}',
    ]
    if command == 'debug':
        client_cmd.extend(['-o', 'monitor reset halt'])
    for command_text in self.gdb_init or []:
        client_cmd.extend(['-o', command_text])

    self.require(client_cmd[0])
    self.print_gdbserver_message()
    server_proc = self.popen_ignore_int(server_cmd, stderr=subprocess.DEVNULL)
    try:
        self.run_client(client_cmd)
    finally:
        server_proc.terminate()
        server_proc.wait()


def _jlink_zdb_do_run(self, command, **kwargs):
    if not _uses_zdb(self) or command not in ('debug', 'attach'):
        return _jlink_do_run(self, command, **kwargs)
    if MISSING_REQUIREMENTS:
        raise RuntimeError(
            'one or more Python dependencies were missing; '
            'see the getting started guide for details on how to fix'
        )

    _require_zdb(self)
    self.commander = os.fspath(Path(self.require(self.commander)).resolve())
    self.logger.debug(f'JLink executable: {self.commander}')
    self.logger.info(f'JLink version: {self.jlink_version_str}')

    rtos = self.thread_info_enabled and self.supports_thread_info
    plugin_dir = os.fspath(
        Path(self.commander).parent / 'GDBServer' / 'RTOSPlugin_Zephyr'
    )
    big_endian = self.build_conf.getboolean('CONFIG_BIG_ENDIAN')

    if self.dev_id:
        device_type = (
            self._detect_dev_id_type(self.dev_id)
            if self.dev_id_type == 'auto'
            else self.dev_id_type
        )
        if device_type == 'serialno':
            device_select = f'usb={self.dev_id}'
        elif device_type == 'tty':
            device_select = f'tty={self.dev_id}'
        elif device_type in ('ip', 'tunnel'):
            device_select = f'{device_type}={self.dev_id}'
        else:
            device_select = f'usb={self.dev_id}'
    else:
        device_select = 'usb'

    server_cmd = (
        [self.gdbserver]
        + ['-select', device_select]
        + ['-port', str(self.gdb_port)]
        + ['-if', self.iface]
        + ['-speed', self.speed]
        + ['-device', self.device]
        + ['-silent']
        + ['-endian', 'big' if big_endian else 'little']
        + ['-singlerun']
        + (['-nogui'] if self.supports_nogui else [])
        + (['-rtos', plugin_dir] if rtos else [])
        + ['-rtttelnetport', str(self.rtt_port)]
        + self.tool_opt
    )

    if self.file is not None:
        if self.file_type != FileType.ELF:
            raise ValueError('Cannot debug; elf file required')
        elf_name = self.file
    elif self.elf_name is None:
        raise ValueError('Cannot debug; elf is missing')
    else:
        elf_name = self.elf_name

    # J-Link Commander handles programming before ZDB connects to the GDB
    # server. Attach deliberately skips this step.
    if command == 'debug':
        self.flash(**kwargs)

    client_cmd = [self.cfg.gdb, elf_name]
    if self.is_batch:
        client_cmd.append('--batch')
    client_cmd.extend(
        ['-o', f'gdb-remote {self.gdb_host or "localhost"}:{self.gdb_port}']
    )
    if command == 'debug':
        client_cmd.extend(['-o', 'monitor halt', '-o', 'monitor reset'])
        if self.is_batch:
            client_cmd.extend(['-o', 'monitor go', '-o', 'process detach'])

    if not self.gdb_host:
        self.require(self.gdbserver)
        self.print_gdbserver_message()
        self.run_server_and_client(server_cmd, client_cmd)
    else:
        self.run_client(client_cmd)


OpenOcdBinaryRunner.do_run = _openocd_zdb_do_run
JLinkBinaryRunner.do_run = _jlink_zdb_do_run

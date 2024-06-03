#!/usr/local/bin/python3

import re, os, logging
import configparser

error = configparser.Error

SDK_GLUE_BASE = os.path.realpath(os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", ".."))
WORKSPACE_BASE = os.path.realpath(os.path.join(SDK_GLUE_BASE, ".."))
HPM_SDK_BASE = os.path.join(WORKSPACE_BASE, "sdk_env", "hpm_sdk")
MODULES_BASE = os.path.join(WORKSPACE_BASE, "modules")
SDK_ENV_BASE = os.path.join(WORKSPACE_BASE, "sdk_env")
ZEPHYR_BASE = os.path.join(WORKSPACE_BASE, "zephyr")
SCRIPT_BASE = os.path.join(SDK_GLUE_BASE, "scripts")

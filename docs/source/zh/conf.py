# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

import os
import os.path
import sys
import datetime
from pathlib import Path

HPM_ZSG_BASE = Path(__file__).resolve().parents[3]
project = 'HPMicro Supplemental Development Kit'
copyright = '2024-%s, HPMicro' % datetime.date.today().year
author = '先楫半导体软件组'
sys.path.insert(0, str(HPM_ZSG_BASE / "docs" / "_ext"))

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    'sphinx_rtd_theme',
    "external_content",
]

templates_path = ['_templates']
exclude_patterns = []

language = 'zh_CN'
html_show_sphinx = False

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
# html_static_path = ['_static']

external_content_contents = [
    (HPM_ZSG_BASE, "CHANGELOG.md"),
    (HPM_ZSG_BASE, "samples/**/*_zh.rst",),
    (HPM_ZSG_BASE, "samples/**/doc"),
]

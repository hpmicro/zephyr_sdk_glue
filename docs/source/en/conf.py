# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

import os
import os.path
import datetime
from pathlib import Path

HPM_ZSG_BASE = Path(__file__).resolve().parents[3]
project = 'HPMicro Supplemental Development Kit'
copyright = '2024-%s, HPMicro' % datetime.date.today().year
author = 'HPMicro Software Team'


# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = []

templates_path = ['_templates']
exclude_patterns = []

language = 'en'

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
# html_static_path = ['_static']


external_content_contents = [
    (HPM_ZSG_BASE, "CHANGELOG.md"),
    (HPM_ZSG_BASE, "samples/**/*_en.rst",),
    (HPM_ZSG_BASE, "samples/**/doc"),
]
# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

import datetime

project = 'Zephyr SDK Glue'
copyright = '2024-%s, HPMicro' % datetime.date.today().year
author = '先楫半导体软件组'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    'sphinx_rtd_theme',
    'myst_parser',
]

templates_path = ['_templates']
exclude_patterns = []

language = 'zh_CN'
html_show_sphinx = False

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
html_theme_options = {
    'collapse_navigation': False,
    'sticky_navigation': True,
    'navigation_depth': 4,
}
html_static_path = ['_static']
html_css_files = ['custom.css']
html_js_files = ['sidebar-state.js']

source_suffix = {
    '.rst': 'restructuredtext',
    '.md': 'markdown',
}

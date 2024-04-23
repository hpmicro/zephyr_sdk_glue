#!/usr/local/bin/python3

import re
import os
import sys
import datetime
from enum import Enum

class LicenseType(Enum):
    LICENSE_APACHE = 0
    LICENSE_BSD = 1
    
class FileType(Enum):
    FILE_TYPE_C = 0
    FILE_TYPE_SCRIPT = 1         # kconfig yaml cmake

CURRENT_TIME = datetime.datetime.now()

LICENSE_APACHE_CONTEXT = """ Copyright (c) <YYYY> <ORGANIZATION>
SPDX-License-Identifier: Apache-2.0"""

LIC_BSD_3CLAUSE_CONTENT = """ Copyright (c) <YYYY> <ORGANIZATION>

 SPDX-License-Identifier: BSD-3-Clause"""

AUTOGENERATE_HEADER_CONTEXT = f"""
<---------------------- AUTO_GENERATE ---------------------->
    This file was auto-generated, do not edit anything.
    generated on {CURRENT_TIME.strftime('%Y-%m-%d %H:%M:%S')} """
    
def get_header_text(logging, file_type = FileType.FILE_TYPE_C):
    header_text = generate_commented_content(AUTOGENERATE_HEADER_CONTEXT, file_type)
    return header_text

def get_license_text(lic, organization, logging, file_type = FileType.FILE_TYPE_C, starting_year = 2024):
    if lic == LicenseType.LICENSE_APACHE:
        return apache(organization, logging = logging, file_type = file_type, starting_year = starting_year)
    elif lic == LicenseType.LICENSE_BSD:
        return bsd3clause(organization, logging = logging, file_type = file_type, starting_year = starting_year)
    else:
        logging.error("Not supported license: " + str(lic))
        return None

def generate_commented_content(text, file_type):
    if file_type == FileType.FILE_TYPE_C:
        first_line = '/*\n'
        last_line = '\n */'
        comment_char = ' *'
    elif file_type == FileType.FILE_TYPE_SCRIPT:
        first_line = '#\n'
        last_line = '\n'
        comment_char = '# '

    update_lic = re.sub(r'^', r'%s' % comment_char, text)
    update_lic = re.sub(r'\n', r'\n%s' % comment_char, update_lic)

    return first_line + update_lic + last_line + '\n'

def apache(organization, file_type = FileType.FILE_TYPE_C, year=datetime.datetime.now().year, logging = None, starting_year = 2024):
    lic_text = generate_commented_content(LICENSE_APACHE_CONTEXT, file_type)
    if year > starting_year:
        lic_text = re.sub(r'<YYYY>', "2024-%d" % (year), lic_text)
    else:
        lic_text = re.sub(r'<YYYY>', str(year), lic_text)
    lic_text = re.sub(r'<ORGANIZATION>', organization, lic_text)
    return lic_text

def bsd3clause(organization, file_type = FileType.FILE_TYPE_C, year=datetime.datetime.now().year, logging = None, starting_year = 2024):
    lic_text = generate_commented_content(LIC_BSD_3CLAUSE_CONTENT, file_type)
    if year > starting_year:
        lic_text = re.sub(r'<YYYY>', "2024-%d" % (year), lic_text)
    else:
        lic_text = re.sub(r'<YYYY>', str(year), lic_text)
    lic_text = re.sub(r'<ORGANIZATION>', organization, lic_text)
    return lic_text


# Copyright (c) 2024 HPMicro
# SPDX-License-Identifier: BSD-3-Clause

import sys, re, os, logging, argparse

import lib.sdk_glue_utils as utils
import lib.sdk_glue_banner as ba


base_dir = utils.SDK_GLUE_BASE
operate_dir = os.path.join(utils.HPM_SDK_BASE, "soc")
output_dir = os.path.join(utils.SDK_GLUE_BASE, "include", "dt-bindings", "clock")
soc_list = ["HPM5361", "HPM6280", "HPM6360", "HPM6750", "HPM6880"]

class dts_header:
    config_error = utils.error
    
    def __init__(self, soc, lic_args):
        self.soc = soc.upper()
        self.lic_args = lic_args
        self.clock_path = ""
        self.sysctl_path = ""
        self.sysctl_reg_path = ""
        self.output_path = output_dir
        self.output_name = soc.lower() + "-clocks.h"

    def get_soc(self):
        return self.soc

    def get_lic_args(self):
        return self.lic_args

    def get_license_text(self):
        if self.lic_args.get('lic') is None or self.lic_args.get('org') is None:
            raise self.config_error(
                "No license and organization parse")

        lic = self.lic_args['lic']
        org = self.lic_args['org']   
        return ba.get_license_text(lic, org, logging)

    def get_header_text(self):
        header_macro = 'ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_' + re.sub(r'[^a-zA-Z0-9_]', '_', self.output_name).upper()
        text = '#ifndef ' + header_macro + '\n' + '#define ' + header_macro + '\n\n'
        header = text + ba.get_header_text(logging)
        return header
    
    def get_end_text(self):
        header_macro = 'ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_' + re.sub(r'[^a-zA-Z0-9_]', '_', self.output_name).upper()
        return ("#endif  /* %s */" % header_macro)

    def check_series_name(self, name):
        match = re.search(r'HPM\d{4}', name)
        if match:
            soc = match.group(0)
            series = soc[:5] + '00'
            return series
        else:
            return None
            
    def check_clock_path(self):
        soc = self.soc
        series = self.check_series_name(soc)
        clock_path = os.path.join(operate_dir, series, soc, "hpm_clock_drv.h")
        sysctl_path = os.path.join(operate_dir, series, soc, "hpm_sysctl_drv.h")
        sysctl_reg_path = os.path.join(operate_dir, series, "ip", "hpm_sysctl_regs.h")

        if not os.path.exists(clock_path):
            raise self.config_error(
                "No %s clock header file" % soc)

        if not os.path.exists(sysctl_path):
            raise self.config_error(
                "No %s sysctl header file" % soc)

        if not os.path.exists(sysctl_reg_path):
            raise self.config_error(
                "No %s sysctl reg header file" % soc)
        
        self.clock_path = clock_path
        self.sysctl_path = sysctl_path
        self.sysctl_reg_path = sysctl_reg_path

    def parse_define(self, line):
        line = line.strip()
        tokens = line.split()
        if not line.startswith('#define'):
            return None, None

        if len(tokens) < 3:
            return None, None

        name = tokens[1]
        match = re.search(r'0x[\dA-Fa-f]+', tokens[2])
        if match:
            value = match.group()
            value = str(int(value, 16))
        else:
            value = ''.join(filter(str.isdigit, tokens[2]))
        return name, value
    
    def amend_enum(self, text):
        update_text = ""
        result = ""
        define_dict = self._build_define_dict()
        count_dict = {}
        cnt = None
        
## replace macro to number(str)
        for line in text.strip().split(","):
            if "=" in line:
                parts = line.split("=")
                replace_name = parts[1].strip()
                if replace_name in define_dict:
                    update_text += line.replace(replace_name, define_dict.get(replace_name, "")) + ","
                else:
                    update_text += line + ","
            else:
                update_text += line + ","

## fill table
        for line in update_text.strip().split(","):
            if "=" in line:
                key, value = line.split("=")
                key = key.strip()
                value = value.strip()
                match = re.search(r'0x[\dA-Fa-f]+', value)
                
                if value.isdigit() or match:
                    if match:
                        hex_value = match.group()
                        value = str(int(hex_value, 16))
                        line = line.replace(hex_value, value)

                    count_dict[key] = value
                    result += line
                    cnt = int(value)
                else:
                    result += line.replace(value, str(cnt))

            elif cnt is not None:
                key = line.strip()
                if key:
                    cnt += 1
                    count_dict[key] = str(cnt)
                    line = line + " = " + str(cnt)
                    result += line
        return result

    def _build_define_dict(self):
        dict = {}
        with open(self.clock_path, mode='r', encoding='utf-8') as f:
            for line in f:
                name, value = self.parse_define(line)
                if name is not None:
                    if name.startswith('CLK_SRC_GROUP') or name.startswith('RESOURCE_'):
                        dict[name] = value
        
        with open(self.sysctl_reg_path, mode='r', encoding='utf-8') as s:
            for line in s:
                name, value = self.parse_define(line)
                if name is not None:
                    if name.startswith('SYSCTL_RESOURCE') or name.startswith('SYSCTL_CLOCK'):
                        dict[name] = value
        return dict

    def _build_enum_dict(self):
        dict = {}
        text = ""
        
        with open(self.sysctl_path, mode='r', encoding='utf-8', newline='') as f:
            sysctl_content = f.read()
            
        resource = r'typedef\s+enum\s*{([^}]*)}\s+sysctl_resource_t'
        source_match = re.search(resource, sysctl_content, re.DOTALL)
        node = r'typedef\s+enum\s*{([^}]*)}\s+clock_node_t'
        node_match = re.search(node, sysctl_content, re.DOTALL)

        if not source_match or not node_match:
            raise self.config_error(
                "No sysctl enum in %s" % self.sysctl_path)

        text = source_match.group(1).strip() + "\n" + node_match.group(1).strip()
        text = self.amend_enum(text)

        for line in text.strip().split("\n"):
            if "=" in line:
                name, value = line.split("=")
                name = name.strip()
                value = value.strip()
                if name is not None:
                    dict[name] = value
        return dict            

    def generate_clock_sources(self):
        text = ""
        result = ""
        with open(self.clock_path, mode="r", encoding='utf-8', newline='') as f:
            clock_content = f.read()

        pattern = r'typedef\s+enum\s+_clock_sources\s*{([^}]*)}\s+clk_src_t'
        matches = re.search(pattern, clock_content, re.DOTALL)

        if matches:
            text = matches.group(1).strip()
        else:
            raise self.config_error(
                "No clock sources in %s" % self.clock_path)

        dict = self._build_define_dict()
        for line in text.strip().split("\n"):
            if line.strip(","):
                parts = line.split("=")
                name = parts[0].strip().upper()
                src = parts[1].strip().replace("MAKE_CLK_SRC(", "").replace(")", "").split(",")
                group = src[0].strip()
                index = int(src[1].strip())
                number = ((int(dict[group]) << 4) | (index))
                item = "#define " + name + "  " + str(number) + "\n"
                result = result + item
        return result

    def generate_clock_names(self):
        text = ""
        result = ""
        with open(self.clock_path, mode="r", encoding='utf-8', newline='') as f:
            clock_content = f.read()

        pattern = r'typedef\s+enum\s+_clock_name\s*{([^}]*)}\s+clock_name_t'
        matches = re.search(pattern, clock_content, re.DOTALL)

        if matches:
            text = matches.group(1).strip()
        else:
            raise self.config_error(
                "No clock names in %s" % self.clock_path)

        enum_dict = self._build_enum_dict()
        define_dict = self._build_define_dict()
        for line in text.strip().split("\n"):
            line = line.strip()
            if line and not line.startswith("//") and not line.startswith("/*"):
                parts = line.split("=")
                name = parts[0].strip().upper()
                if len(parts) > 1:
                    value = parts[1].strip()
                    if "MAKE_CLOCK_NAME(" in value:
                        src = parts[1].strip().replace("MAKE_CLOCK_NAME(", "").replace(")", "").split(",")
                        res = src[0].strip()
                        node = src[2].strip()
                        group = int(define_dict[src[1].strip()])
                        if res in enum_dict:
                            res = int(enum_dict[res])
                        else:
                            res = int(define_dict[res])

                        if node in enum_dict:
                            node  = int(enum_dict[node])
                        else:
                            node = int(node)
                        sort_number = res << 16 | group << 8 | node
                    else:
                        value = value.rstrip(',')
                        if value in enum_dict:
                            sort_number = int(enum_dict[value])
                        elif value.isdigit():
                            sort_number = int(value)
                        else:
                            if value in define_dict:
                                sort_number = int(define_dict[value])
                            else:
                                sort_number = value.upper()
                item = "#define " + name + "  " + str(sort_number) + "\n"
                result = result + item
        return result

    def generate_clock_header(self):
        self.check_clock_path()
        license_text = self.get_license_text()
        header_text = self.get_header_text()
        clk_source = self.generate_clock_sources()
        clk_name = self.generate_clock_names()
        output = os.path.join(self.output_path, self.output_name)
        
        if license_text and header_text and clk_name and clk_source:
            pass
        else:
            return False
        
        with open(output, newline='', mode='w', encoding='utf-8') as header:
            header.write(str(license_text))
            header.write("\n")
            header.write(str(header_text))
            header.write("\n")
            header.write(str(clk_source))
            header.write("\n")
            header.write(str(clk_name))
            header.write("\n\n")
            header.write(self.get_end_text())
            header.write("\n")
            header.close()
        return True

def main():
    parser = argparse.ArgumentParser(description = "generate zephyr dts header files")
    parser.add_argument('--soc',
                        help='''generate hpmicro soc zephyr dts header
                        options: "HPM5361", "HPM6280", "HPM6360", "HPM6750", "HPM6880" ''')
    parser.add_argument('--org',
                        help='organization name')
    parser.add_argument('--license',
                        help='''specify license
                        options: "APACHE", "BSD" ''')
    args = parser.parse_args()
    soc = args.soc
    org = args.org
    lic = args.license
    
    if lic is None or lic == "APACHE":
        lic = ba.LicenseType.LICENSE_APACHE
    elif lic == "BSD":
        lic = ba.LicenseType.LICENSE_BSD
    
    if org is None:
        org = "HPMicro"
    lic_args = {'lic': lic, 'org': org}
        
    if soc is None:
        for name in soc_list:
            ins = dts_header(name, lic_args)
            ins.generate_clock_header()
    else:
        ins = dts_header(soc, lic_args)
        ins.generate_clock_header()
    sys.exit(0)

if __name__ == "__main__":
    main()
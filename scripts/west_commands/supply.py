#!/usr/local/bin/python3

import sys, re, os, logging, argparse
import subprocess
import lib.utils as utils

from textwrap import dedent
from west.commands import WestCommand
from west import log

class Supply(WestCommand):
    '''"west supply" command extension class.'''
    def __init__(self):
        super().__init__(
            'supply',
            'scripts must be run before building projects',
            dedent('''
                Neccessary patch for hpm_sdk
            '''))

    def do_add_parser(self, parser_adder):
        parser = parser_adder.add_parser(
            self.name,
            help=self.help,
            formatter_class=argparse.RawDescriptionHelpFormatter,
            description=self.description)

        # Add some example options using the standard argparse module API.
        parser.add_argument('-f', '--file', help='file including patch')
        parser.add_argument('-F', '--folder', help='folder including patch files')
        parser.add_argument('-p', '--path', help='repo path that requires patching')

        return parser

    def check_patch_applicable(self, file, path = utils.HPM_SDK_BASE):
        os.chdir(path)
        result = subprocess.run(['git', 'apply', '--check', file], capture_output=True, text=True)
        return result.returncode == 0
    
    def apply_patch(self, file, path = utils.HPM_SDK_BASE):
        os.chdir(path)
        result = subprocess.run(['git', 'apply', file], capture_output=True, text=True)
        if (result.returncode != 0):
            print(f"{file} patched in {path} failed, please check")
        return result.returncode == 0
    
    def am_patch(self, file, path = utils.HPM_SDK_BASE):
        os.chdir(path)
        result = subprocess.run(['git', 'am', file], capture_output=True, text=True)
        if (result.returncode != 0):
            print(f"{file} patched in {path} failed, please check")
        return result.returncode == 0

    def process_directory(self, dir, path = utils.HPM_SDK_BASE):
        for root, dirs, files in os.walk(dir):
            for file in files:
                file_path = os.path.join(root, file)
                if self.check_patch_applicable(file_path, path):
                    self.am_patch(file_path, path)
                else:
                    print(f"{file_path} patched in {path} failed")
    
    def get_sdk_version(self):
        version_file = os.path.join(utils.HPM_SDK_BASE, "VERSION")
        pattern = re.compile(r'(VERSION_MAJOR|VERSION_MINOR|PATCHLEVEL|VERSION_TWEAK|EXTRAVERSION)\s*=\s*(\d+)')
        matches = []
        
        version_info = {
            'VERSION_MAJOR': '0',
            'VERSION_MINOR': '0',
            'PATCHLEVEL': '0'
        }
        
        try:
            with open(version_file, "r") as version:   
                for line in version:
                    match = pattern.findall(line)
                    matches.extend(match)
        except Exception as e:
            print(f"error: {e}")
        
        for key, value in matches:
            version_info[key] = value
        
        version_string = f"{'v' + version_info['VERSION_MAJOR']}.{version_info['VERSION_MINOR']}.{version_info['PATCHLEVEL']}"
        return version_string

    
    def process_default(self):
        version = self.get_sdk_version()
        folder_version = "hpm_sdk" + "_" + version
        patch_path = os.path.join(utils.SCRIPT_BASE, "patch", folder_version)
        
        if os.path.exists(patch_path):
            self.process_directory(patch_path, utils.HPM_SDK_BASE)
        else:
            print(f"No corresponding patch for hpm_sdk {version}")
     
    def do_run(self, args, unknown_arguments):
        self.args = args
        file_path = None
        folder_path = None
        repo_path = utils.HPM_SDK_BASE
        
        self.get_sdk_version()

        if args.file:
            if os.path.isfile(args.file) :
                file_path = os.path.abspath(args.file)
            else:
                print(f"{args.file} missed, check file path or name")
                sys.exit(1)            
            
        if args.folder:
            if os.path.isdir(args.folder):
                folder_path = os.path.abspath(args.folder)
            else:
                print(f"{args.folder} missed, check folder path")
                sys.exit(1)
        
        if args.path:
            if os.path.isdir(args.path):
                repo_path = os.path.abspath(args.path)
            else:
                print(f"{args.path} missed, check repo path")
                sys.exit(1)            
        
        if file_path:
            if self.check_patch_applicable(file_path, repo_path):
                self.am_patch(file_path, repo_path)
            else:
                print(f"{file_path} patched in {repo_path} failed")
                sys.exit(1)
        
        if folder_path:
            self.process_directory(folder_path, repo_path)
        
        if all(var is None for var in [file_path, folder_path]):
            if repo_path == utils.HPM_SDK_BASE:
                self.process_default()
            else:
                print(f"{repo_path} need specify patch file")
                
        
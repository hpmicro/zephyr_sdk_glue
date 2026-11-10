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
        self._patch_failed = False

    def do_add_parser(self, parser_adder):
        parser = parser_adder.add_parser(
            self.name,
            help=self.help,
            formatter_class=argparse.RawDescriptionHelpFormatter,
            description=self.description)

        parser.add_argument('-f', '--file', help='file including patch')
        parser.add_argument('-F', '--folder', help='folder including patch files')
        parser.add_argument('-p', '--path', help='repo path that requires patching')
        parser.add_argument('--force', action='store_true', 
                            help='force re-apply patch (reverse old patch first)')

        return parser

    def _run_git_cmd(self, cmd_args, path):
        '''Run git without changing the process working directory.'''
        return subprocess.run(cmd_args, cwd=path, capture_output=True, text=True)

    def _run_git_apply(self, file, path, *options):
        '''Apply SDK-relative paths from the owning Git worktree root.'''
        patch_file = os.path.abspath(file)
        target = os.path.realpath(path)
        result = self._run_git_cmd(['git', 'rev-parse', '--show-toplevel'], target)
        if result.returncode != 0:
            return result

        root = os.path.realpath(result.stdout.strip())
        prefix = os.path.relpath(target, root).replace(os.sep, '/')
        command = ['git', 'apply', *options]
        if prefix != '.':
            command.append('--directory=' + prefix)
        command.extend(['--', patch_file])
        # Running in a subdirectory can silently skip every patch and still
        # return success. Always check and apply from the root, using the same
        # prefix for embedded SDKs as well as independent SDK repositories.
        return self._run_git_cmd(command, root)

    def check_patch_applicable(self, file, path=utils.HPM_SDK_BASE):
        '''Check if patch can be applied.'''
        result = self._run_git_apply(file, path, '--check')
        if result.returncode != 0:
            print(result.stderr.strip())
        return result.returncode == 0
    
    def check_patch_already_applied(self, file, path=utils.HPM_SDK_BASE):
        '''Check if patch has already been applied (can be reversed).'''
        result = self._run_git_apply(file, path, '--reverse', '--check')
        return result.returncode == 0
    
    def reverse_patch(self, file, path=utils.HPM_SDK_BASE):
        '''Reverse an already applied patch.'''
        result = self._run_git_apply(file, path, '--reverse')
        if result.returncode == 0:
            print(f"Reversed old patch: {os.path.basename(file)}")
        return result.returncode == 0
    
    def apply_patch(self, file, path=utils.HPM_SDK_BASE, force=False):
        '''Apply a patch file. Returns True on success, False on failure.'''
        filename = os.path.basename(file)
        
        if self.check_patch_already_applied(file, path):
            if force:
                if not self.reverse_patch(file, path):
                    print(f"Failed to reverse patch: {filename}")
                    return False
            else:
                print(f"Patch already applied, skipping: {filename}")
                return True
        
        if not self.check_patch_applicable(file, path):
            print(f"Patch not applicable: {filename} in {path}")
            return False
        
        result = self._run_git_apply(file, path)
        if result.returncode != 0:
            print(f"Failed to apply patch: {filename}")
            print(f"  Error: {result.stderr.strip()}")
            return False
        
        print(f"Successfully applied patch: {filename}")
        return True
    
    def am_patch(self, file, path=utils.HPM_SDK_BASE):
        result = self._run_git_cmd(['git', 'am', file], path)
        if result.returncode != 0:
            print(f"{file} patched in {path} failed, please check")
        return result.returncode == 0

    def process_directory(self, dir, path=utils.HPM_SDK_BASE, force=False):
        '''Process all patch files in directory. Returns True if all succeeded.'''
        all_success = True
        patch_files = []
        
        for root, dirs, files in os.walk(dir):
            dirs.sort()
            for file in sorted(files):
                if file.endswith('.patch') or file.endswith('.diff'):
                    patch_files.append(os.path.join(root, file))
        
        if not patch_files:
            print(f"No patch files found in {dir}")
            return True
        
        for file_path in patch_files:
            if not self.apply_patch(file_path, path, force):
                all_success = False
                self._patch_failed = True
        
        return all_success
    
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
        except FileNotFoundError:
            print(f"Error: VERSION file not found at {version_file}")
            return None
        except Exception as e:
            print(f"Error reading VERSION file: {e}")
            return None
        
        for key, value in matches:
            version_info[key] = value
        
        version_string = f"v{version_info['VERSION_MAJOR']}.{version_info['VERSION_MINOR']}.{version_info['PATCHLEVEL']}"
        return version_string

    def process_default(self, force=False):
        '''Process default patch directory based on SDK version. Returns True on success.'''
        version = self.get_sdk_version()
        if version is None:
            return False
            
        folder_version = "hpm_sdk" + "_" + version
        patch_path = os.path.join(utils.SCRIPT_BASE, "patch", folder_version)
        
        if os.path.exists(patch_path):
            return self.process_directory(patch_path, utils.HPM_SDK_BASE, force)
        else:
            print(f"No corresponding patch for hpm_sdk {version}")
            return True
     
    def do_run(self, args, unknown_arguments):
        self.args = args
        self._patch_failed = False
        file_path = None
        folder_path = None
        repo_path = utils.HPM_SDK_BASE
        force = getattr(args, 'force', False)

        if args.file:
            if os.path.isfile(args.file):
                file_path = os.path.abspath(args.file)
            else:
                print(f"Error: File not found: {args.file}")
                sys.exit(-1)
            
        if args.folder:
            if os.path.isdir(args.folder):
                folder_path = os.path.abspath(args.folder)
            else:
                print(f"Error: Folder not found: {args.folder}")
                sys.exit(-1)
        
        if args.path:
            if os.path.isdir(args.path):
                repo_path = os.path.abspath(args.path)
            else:
                print(f"Error: Repo path not found: {args.path}")
                sys.exit(-1)
        
        if file_path:
            if not self.apply_patch(file_path, repo_path, force):
                self._patch_failed = True
        
        if folder_path:
            if not self.process_directory(folder_path, repo_path, force):
                self._patch_failed = True
        
        if all(var is None for var in [file_path, folder_path]):
            if repo_path == utils.HPM_SDK_BASE:
                if not self.process_default(force):
                    self._patch_failed = True
            else:
                print(f"Error: {repo_path} needs a specified patch file or folder")
                sys.exit(-1)
        
        if self._patch_failed:
            print("\nSome patches failed to apply. Use --force to re-apply patches.")
            sys.exit(-1)
        
        print("\nAll patches applied successfully.")

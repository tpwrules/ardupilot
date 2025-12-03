#!/usr/bin/env python3
"""
Pymavlink Directory Uploader Script (with subdirectory support)

This script uses the MAVFTP class from the pymavlink library to recursively
upload the contents of a local directory to the /APM/scripts/ directory on a
vehicle running ArduPilot, preserving the subdirectory structure.

It automates the process of:
1. Connecting to the vehicle.
2. Traversing the local source directory.
3. Creating a corresponding directory structure on the remote vehicle.
4. Uploading each file to its correct remote location.
5. Optionally, sending a command to restart the scripting engine.
6. Terminating the connection gracefully.

Dependencies:
- Pymavlink (with mavftp module)

Example Usage:
1. Make the script executable (on Linux/macOS):
   chmod +x update_scripts.py

2. Sync a local 'scripts' directory and restart the scripting engine:
   ./update_scripts.py --connect udp:127.0.0.1:14550 --restart ./my_lua_scripts

3. Sync with verbose output without restarting:
   ./update_scripts.py --connect /dev/ttyACM0 -v /path/to/local/scripts
"""

import argparse
import os
import sys
import time
import traceback
from pymavlink import mavutil
from pymavlink.mavftp import MAVFTP, FtpError, DirectoryEntry

def ensure_remote_dir(ftp: MAVFTP, remote_path: str, verbose: bool = False):
    """
    Ensures a full directory path exists on the remote, creating parts as needed.
    """
    parts = remote_path.strip('/').split('/')
    current_remote_path = ""
    for part in parts:
        current_remote_path += f"/{part}"
        if not directory_exists(ftp, current_remote_path, verbose):
            # Clear the progress bar line before printing the status
            sys.stdout.write('\r' + ' ' * 120 + '\r')
            print(f" -> Creating remote directory '{current_remote_path}'")
            result = ftp.cmd_mkdir([current_remote_path])
            if result.error_code not in [FtpError.Success, FtpError.FileExists]:
                result.display_message()
                raise RuntimeError(f"Failed to create directory {current_remote_path}")

def sync_directory(connect_str: str, source_dir: str, verbose: bool = False, restart: bool = False):
    """
    Connects to a vehicle and recursively syncs a local directory to the
    vehicle's /APM/scripts/ directory.
    """
    if not os.path.isdir(source_dir):
        print(f"Error: Source directory not found at '{source_dir}'")
        sys.exit(1)

    # --- Step 1: Discover all files and their remote paths ---
    files_to_upload = []
    for dirpath, _, filenames in os.walk(source_dir):
        for filename in filenames:
            local_path = os.path.join(dirpath, filename)
            relative_path = os.path.relpath(local_path, source_dir)
            remote_path = f"/APM/scripts/{relative_path.replace(os.path.sep, '/')}"
            files_to_upload.append((local_path, remote_path))

    if not files_to_upload:
        print("\nSource directory is empty. No files to upload.")
        return

    master = None
    try:
        print(f"Connecting to vehicle on: {connect_str}")
        os.environ['MAVLINK20'] = '1'
        master = mavutil.mavlink_connection(connect_str, source_system=255)
        
        print("Waiting for heartbeat...")
        master.wait_heartbeat(timeout=10)
        print("Heartbeat received. Vehicle is connected.")

        ftp = MAVFTP(master, master.target_system, master.target_component)

    except Exception as e:
        print(f"Failed to connect or initialize FTP: {e}")
        print("Please check the connection string and ensure the vehicle is powered and ready.")
        sys.exit(1)

    try:
        # --- Step 2: Ensure all remote directories exist ---
        remote_dirs = sorted(list(set(os.path.dirname(remote) for _, remote in files_to_upload)))
        total_dirs = len(remote_dirs)
        print(f"\nChecking {total_dirs} remote directories...")

        for i, r_dir in enumerate(remote_dirs):
            progress = (i + 1) / total_dirs
            bar_length = 40
            filled_length = int(bar_length * progress)
            bar = '█' * filled_length + '-' * (bar_length - filled_length)
            percent_str = f"{progress * 100:.1f}%"
            dir_for_display = r_dir if len(r_dir) < 60 else f"...{r_dir[-57:]}"
            sys.stdout.write(f'\r[{bar}] {percent_str} | Checking {dir_for_display}')
            sys.stdout.flush()
            ensure_remote_dir(ftp, r_dir, verbose)
        sys.stdout.write('\n')

        # --- Step 3: Upload each file individually with a progress bar ---
        total_files = len(files_to_upload)
        print(f"\nUploading {total_files} files...")
        successful_uploads = 0
        for i, (local_path, remote_path) in enumerate(files_to_upload):
            filename = os.path.basename(local_path)
            
            progress = (i + 1) / total_files
            bar_length = 40
            filled_length = int(bar_length * progress)
            bar = '█' * filled_length + '-' * (bar_length - filled_length)
            percent_str = f"{progress * 100:.1f}%"
            relative_path_for_display = os.path.relpath(local_path, source_dir)
            sys.stdout.write(f'\r[{bar}] {percent_str} | Uploading {relative_path_for_display}...')
            sys.stdout.flush()

            try:
                ftp.cmd_put([local_path, remote_path])
                result = ftp.process_ftp_reply('put', timeout=600)
                if result.error_code == FtpError.Success:
                    successful_uploads += 1
                else:
                    sys.stdout.write('\n')
                    print(f"  [FAILURE] Could not upload file '{filename}'.")
                    result.display_message()

            except Exception as e:
                sys.stdout.write('\n')
                if verbose:
                    print("\n--- DETAILED EXCEPTION TRACE ---")
                    traceback.print_exc()
                    print("------------------------------------")
                print(f"\n  [FAILURE] An exception occurred during upload of '{filename}'. Reason: {e}")
                ftp.cmd_cancel()
                ftp.process_ftp_reply('cancel', timeout=5)
        
        sys.stdout.write('\n')

        # --- Step 4: Final Report ---
        print("\n--- Sync Complete ---")
        print(f"Successfully uploaded {successful_uploads} of {total_files} files.")
        if successful_uploads == total_files:
            print("[SUCCESS] All files were synced successfully.")
            
            # --- Step 5: Restart scripting engine if requested ---
            if restart:
                print("Sending command to restart scripting engine...")
                master.mav.command_int_send(
                    master.target_system,
                    master.target_component,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                    mavutil.mavlink.MAV_CMD_SCRIPTING,
                    0, 0,
                    mavutil.mavlink.SCRIPTING_CMD_STOP_AND_RESTART,
                    0, 0, 0, 0, 0, 0
                )

        else:
            print("[FAILURE] Some files failed to upload. Please review the log.")
            sys.exit(1)

    finally:
        if master and getattr(master, 'close', None):
            master.close()
            print("\nConnection closed.")

def directory_exists(ftp: MAVFTP, path: str, verbose: bool = False) -> bool:
    """Checks if a directory exists by listing its parent."""
    parent_dir, target_name = os.path.split(path)
    if not parent_dir:
        parent_dir = "/"
    if not target_name:
        return True

    if verbose:
        print(f"\n[DEBUG] Checking if directory '{target_name}' exists in '{parent_dir}'...")

    try:
        result = ftp.cmd_list([parent_dir])
        if result.error_code != FtpError.Success:
            if verbose:
                print(f"[DEBUG] Failed to list '{parent_dir}'. Assuming directory does not exist.")
                result.display_message()
            return False

        if verbose:
            print(f"[DEBUG] Directory listing for '{parent_dir}':")
            if not ftp.list_result:
                print("[DEBUG]   (empty listing)")
        
        for entry in ftp.list_result:
            entry_name = entry.name.strip('\x00').strip()
            if verbose:
                print(f"[DEBUG]   - Found entry: name='{entry.name}', stripped='{entry_name}', is_dir={entry.is_dir}")
            if entry_name == target_name and entry.is_dir:
                if verbose:
                    print(f"[DEBUG]   >>> Match found for '{target_name}'.")
                return True
        
        if verbose:
            print(f"[DEBUG] No match found for '{target_name}'.")
        return False
    except Exception as e:
        if verbose:
            print(f"[DEBUG] An exception occurred in directory_exists: {e}")
        return False

def main():
    """Parses command-line arguments and initiates the directory sync."""
    parser = argparse.ArgumentParser(
        description="Recursively upload files from a local directory to /APM/scripts/ on an ArduPilot vehicle.",
        formatter_class=argparse.RawTextHelpFormatter
    )
    parser.add_argument("--connect", required=True, help="Pymavlink connection string (e.g., udp:127.0.0.1:14550, /dev/ttyUSB0).")
    parser.add_argument("source_directory", help="Local directory of files to upload.")
    parser.add_argument("-v", "--verbose", action="store_true", help="Enable verbose debug output.")
    parser.add_argument("--restart", action="store_true", help="Restart scripting engine on successful upload.")
    args = parser.parse_args()
    sync_directory(args.connect, args.source_directory, args.verbose, args.restart)

if __name__ == "__main__":
    main()


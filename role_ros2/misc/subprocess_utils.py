"""
Subprocess utilities for running terminal commands
"""

import subprocess
import threading
import multiprocessing


def run_terminal_command(command):
    """Run a terminal command and return the process"""
    process = subprocess.Popen(
        command, stdout=subprocess.PIPE, stdin=subprocess.PIPE, 
        shell=True, executable="/bin/bash", encoding="utf8"
    )
    return process


def run_threaded_command(command, args=(), daemon=True):
    """Run a command in a separate thread"""
    thread = threading.Thread(target=command, args=args, daemon=daemon)
    thread.start()
    return thread


def run_multiprocessed_command(command, args=()):
    """Run a command in a separate process"""
    process = multiprocessing.Process(target=command, args=args)
    process.start()
    return process


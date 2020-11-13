#!/usr/bin/python3

import pathlib
import subprocess

currentPath = pathlib.Path(__file__).parent.absolute()
subprocess.Popen([str(currentPath / "server.py")])

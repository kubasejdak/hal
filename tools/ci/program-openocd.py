#!/usr/bin/python3

import subprocess
import sys

cmd = ["openocd", "-f", "/usr/share/openocd/scripts/board/{}.cfg".format(sys.argv[1]), "-c", "program {} verify reset exit".format(sys.argv[2])]
print(" ".join(cmd))

subprocess.Popen(cmd, stdout=subprocess.PIPE)

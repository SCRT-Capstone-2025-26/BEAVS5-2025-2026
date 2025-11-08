import subprocess as sp
from contextlib import chdir
import os
import sys

if sys.platform != "linux":
    print("The simulation is currently linux only")
    exit(1)

# If this is what the people want
with chdir(os.path.dirname(os.path.abspath(__file__))):
    res = sp.run(["make"])
    if res.returncode != 0:
        print("Maybe you want to run:")
        print("\tpip install pybind11")
        print("\tsudo apt-get install libboost-coroutine-dev")
        exit(1)

del sp
del chdir
del os
del sys

del res

from . import *  # noqa: F403 E402

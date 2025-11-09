import subprocess as sp
from contextlib import chdir
import os
import sys

if sys.platform != "linux":
    print("The simulation is currently linux only")
    exit(1)

# If this is what the people want
with chdir(os.path.join(os.path.dirname(os.path.abspath(__file__)), "Arduino")):
    res = sp.run(["make", "exp"])
    if res.returncode != 0:
        print("Maybe you want to run:")
        print("\tpip install pybind11")
        print("\tsudo apt-get install libboost-coroutine-dev")
        exit(1)

del sp
del chdir

del res

if "pytest" not in sys.modules or os.getcwd() != os.path.dirname(os.path.abspath(__file__)):
    # This is maybe stupid
    from . import beavs_sim  # noqa E402

    attr = None
    for attr in dir(beavs_sim):
        if not attr.startswith("__"):
            globals()[attr] = getattr(beavs_sim, attr)

    del attr
    del beavs_sim

del os
del sys

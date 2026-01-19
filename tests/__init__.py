import subprocess as sp
from contextlib import chdir
import os

with chdir(os.path.join(os.path.dirname(os.path.abspath(__file__)), "../Arduino")):
    sp.run(["make", "exp"])

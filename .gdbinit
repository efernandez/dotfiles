python
import sys
import os
sys.path.insert(0, os.getenv('HOME') + '/.gdb_printers/python')
from libstdcxx.v6.printers import register_libstdcxx_printers


import gzip
import sys
from pathlib import Path

Path(sys.argv[2]).write_bytes(gzip.compress(Path(sys.argv[1]).read_bytes(), mtime=0))

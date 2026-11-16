"""Write firmware/include/follower_config.h from config/follower-m12.json (M12 phase D)."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from bipe.m12.follow import write_header

if __name__ == "__main__":
    print(write_header())

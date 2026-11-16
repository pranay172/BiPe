"""Rebuild artifacts/m12/speed summary from per-case results already on disk."""
import json, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[5]))
from bipe.m12 import speed as S
import concurrent.futures as cf

class Done:
    def __init__(self, v): self.v = v
    def result(self): return self.v
class Pool:
    def __init__(self, *a, **k): pass
    def __enter__(self): return self
    def __exit__(self, *a): return False
    def submit(self, fn, x):
        case, out = x[4], Path(x[6])
        return Done((case["name"], json.loads((out / "results" / f"{case['name']}.json").read_text())))
S.ProcessPoolExecutor = Pool
S.as_completed = lambda fs: fs
sys.argv = ["speed"]
S.main()

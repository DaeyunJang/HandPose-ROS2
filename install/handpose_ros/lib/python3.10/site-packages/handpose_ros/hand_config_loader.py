# hand_config.py
import os, json
from functools import lru_cache
from typing import Any, Dict, List

_DEFAULT = {
    "wrist_index": 0,
    "joint_names": ["mcp", "pip", "dip", "tip"],
    "fingers": {
        "thumb":  [1, 2, 3, 4],
        "index":  [5, 6, 7, 8],
        "middle": [9, 10, 11, 12],
        "ring":   [13, 14, 15, 16],
        "pinky":  [17, 18, 19, 20]
    },
    "finger_color_rgba": {
        "thumb":  [1, 0, 0, 1],
        "index":  [0, 1, 0, 1],
        "middle": [0, 0, 1, 1],
        "ring":   [0, 1, 1, 1],
        "pinky":  [1, 0, 1, 1]
    }
}

def _find_config_path() -> str | None:
    env_p = os.environ.get("HAND_CONFIG_PATH")
    if env_p and os.path.isfile(env_p):
        return env_p
    here = os.path.abspath(os.path.dirname(__file__))
    cand = os.path.join(here, "finger_config.json")
    if os.path.isfile(cand):
        return cand
    cand = os.path.join(os.getcwd(), "finger_config.json")
    if os.path.isfile(cand):
        return cand
    return None

@lru_cache(maxsize=1)
def get_config() -> Dict[str, Any]:
    path = _find_config_path()
    print(f'=====================================================================================')
    print(f'config path: {path}')
    print(f'=====================================================================================')
    if not path:
        return _DEFAULT
    try:
        with open(path, "r", encoding="utf-8") as f:
            user = json.load(f)
    except Exception:
        return _DEFAULT
    # 얕은 병합
    cfg = {**_DEFAULT, **user}
    # 최소 보정
    if "joint_names" not in cfg or not cfg["joint_names"]:
        cfg["joint_names"] = _DEFAULT["joint_names"]
    return cfg

def build_finger_joint_map(cfg: Dict[str, Any]) -> Dict[str, Dict[str, int]]:
    joint_names: List[str] = list(cfg.get("joint_names", _DEFAULT["joint_names"]))
    fmap: Dict[str, Dict[str, int]] = {}
    for finger, indices in cfg["fingers"].items():
        n = min(len(indices), len(joint_names))
        fmap[finger] = {joint_names[i]: int(indices[i]) for i in range(n)}
    return fmap

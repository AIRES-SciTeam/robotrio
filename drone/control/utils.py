from dataclasses import dataclass
from typing import List


@dataclass
class DRONE_ModelConfig:
    world : str
    model : str
    cargo : str
    cargo_ids : List[int]


@dataclass
class DRONE_ConnConfig:
    type : str
    ip : str
    port : int


@dataclass 
class DRONE_TagConfig:
    family : str


@dataclass
class DRONE_FlyCommand:
    roll : int      # [-1000, 1000]
    pitch : int     # [-1000, 1000]
    yaw : int       # [-1000, 1000]
    throttle : int  # [0, 1000]

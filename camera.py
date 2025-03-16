from __future__ import annotations
from dataclasses import dataclass

@dataclass
class Camera:
    name: str
    x: float
    y: float
    alt: float
    heading: float
    pitch: float
    roll: float
    f: float
    px: float
    py: float
    k1: float
    k2: float
    k3: float
    k4: float
    t1: float
    t2: float
    
    def __init__(self, name: str, x: float, y: float, alt: float, heading: float, pitch: float, roll: float, f: float, px: float, py: float, k1: float, k2: float, k3: float, k4: float, t1: float, t2: float):
        self.name = name
        self.x = x
        self.y = y
        self.alt = alt
        self.heading = heading
        self.pitch = pitch
        self.roll = roll
        self.f = f
        self.px = px
        self.py = py
        self.k1 = k1
        self.k2 = k2
        self.k3 = k3
        self.k4 = k4
        self.t1 = t1
        self.t2 = t2

    @classmethod
    def from_parameters_file(cls, path: str):
        with open(path, "r") as file:
            lines = file.readlines()
            lines = [line[:-1] for line in lines[1:]] # Skip header and remove newline characters

            for line in lines:
                values = [float(value) for value in line.split(",")[1:]]
                yield cls(line.split(",")[0], *values)

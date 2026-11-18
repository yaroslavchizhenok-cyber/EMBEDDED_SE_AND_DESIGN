"""
fleet.py - swarm.dll seen from Python, through ctypes.

swarm.dll is the firmware's own fleet.c + pilot.c + craft.c, built for the PC
by test.bat with room for 64 drones (swarm.c gives them a C ABI).

    from fleet import Fleet
    f = Fleet(5, seed=1)            # loads out/swarm.dll and checks its layout
    f.place(0, -6.0, 0.0)           # drone 0 on its pad
    f.wind(3.0, math.radians(30))
    f.gps(*GPS["rtk"])              # or GPS["plain"]: lesson 14's receiver
    f.step(100)                     # one second: 100 ticks of 10 ms, every drone
    f.arm(0)
    f.target(0, (x, y, z), (vx, vy, vz))
    d = f.state(0)                  # d.x, d.ex, d.gx, d.mode ...

ctypes believes whatever it is told about a C struct.  Tell it a wrong layout
and nothing crashes: every number is read from the wrong place, and the drones
seem to fly strangely.  So before flying, this file asks the DLL for the size
and the offset of every field of swarm_drone_t and compares them with
Drone._fields_ below.  If one disagrees it refuses to run - edit both sides
together (../fleet.h's fleet_drone_t and Drone) and rebuild: test.bat.

Standard library only.
"""

import ctypes
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
VERSION = 2                       # must match SWARM_VERSION in swarm.c
TICK_MS = 10                      # one swarm_step(): AIR_HZ = 100
MODES = ("IDLE", "FOLLOW", "HOVER", "LAND")
IDLE, FOLLOW, HOVER, LAND = range(4)
GPS = {"plain": (0.40, 0.10), "rtk": (0.02, 0.02)}   # ../craft.h: GPS_PLAIN, GPS_RTK


class Drone(ctypes.Structure):
    """../fleet.h's fleet_drone_t, field for field and in the same order."""
    _fields_ = [
        ("t_ms", ctypes.c_uint32),
        ("mode", ctypes.c_uint8), ("why", ctypes.c_uint8),
        ("grounded", ctypes.c_uint8), ("fixes", ctypes.c_uint8),
        ("x", ctypes.c_float), ("y", ctypes.c_float), ("z", ctypes.c_float),
        ("vx", ctypes.c_float), ("vy", ctypes.c_float), ("vz", ctypes.c_float),
        ("ex", ctypes.c_float), ("ey", ctypes.c_float), ("ez", ctypes.c_float),
        ("gx", ctypes.c_float), ("gy", ctypes.c_float), ("gz", ctypes.c_float),
        ("roll", ctypes.c_float), ("pitch", ctypes.c_float), ("yaw", ctypes.c_float),
        ("lift", ctypes.c_float), ("charge", ctypes.c_float),
        ("air_x", ctypes.c_float), ("air_y", ctypes.c_float),
        ("hardest_landing", ctypes.c_float),
    ]


class LayoutError(RuntimeError):
    pass


def _library_path():
    names = {"win32": "swarm.dll", "darwin": "libswarm.dylib"}
    return os.path.join(HERE, "out", names.get(sys.platform, "libswarm.so"))


def load(path=None):
    """Load the library and check that it and this file agree on everything."""
    path = path or _library_path()
    if not os.path.exists(path):
        raise FileNotFoundError(f"{path} not found - run test.bat (Windows) or test.sh first")
    lib = ctypes.CDLL(path)
    f, i, u32, v = ctypes.c_float, ctypes.c_int, ctypes.c_uint32, None
    sig = {
        "swarm_version": ([], i), "swarm_drone_size": ([], i),
        "swarm_drone_layout": ([ctypes.POINTER(u32), i], i),
        "swarm_init": ([i, u32], i), "swarm_count": ([], i),
        "swarm_place": ([i, f, f], v), "swarm_wind": ([f, f], v), "swarm_gps": ([f, f], v),
        "swarm_arm": ([i], i), "swarm_land": ([i], i),
        "swarm_set_target": ([i, f, f, f, f, f, f], v),
        "swarm_step": ([], v), "swarm_run": ([i], v),
        "swarm_time_ms": ([], u32),
        "swarm_state": ([i, ctypes.POINTER(Drone)], i),
        "swarm_mode_name": ([i], ctypes.c_char_p),
    }
    for name, (args, res) in sig.items():
        fn = getattr(lib, name)
        fn.argtypes, fn.restype = args, res

    if lib.swarm_version() != VERSION:
        raise LayoutError(f"swarm.dll is version {lib.swarm_version()}, fleet.py expects {VERSION}: rebuild")
    if lib.swarm_drone_size() != ctypes.sizeof(Drone):
        raise LayoutError(f"swarm_drone_t is {lib.swarm_drone_size()} B in C, {ctypes.sizeof(Drone)} B here")
    want = [getattr(Drone, name).offset for name, _ in Drone._fields_]
    got = (u32 * 64)()
    n = lib.swarm_drone_layout(got, 64)
    if n != len(want):
        raise LayoutError(f"swarm_drone_t has {n} fields in C, {len(want)} here")
    for (name, _), w, g in zip(Drone._fields_, want, got):
        if w != g:
            raise LayoutError(f"field {name}: offset {g} in C, {w} here")
    return lib


class Fleet:
    """N drones on one clock."""

    def __init__(self, n, seed=1, path=None):
        self.lib = load(path)
        if self.lib.swarm_init(n, seed & 0xFFFFFFFF) != n:
            raise ValueError(f"swarm_init({n}) refused: 1..64 drones")
        self.n = n
        self._d = Drone()

    def place(self, i, x, y):
        self.lib.swarm_place(i, x, y)

    def wind(self, speed, toward_rad):
        self.lib.swarm_wind(speed, toward_rad)

    def gps(self, wander, jitter):
        """Every receiver's error, m: plain GPS (0.4, 0.1) or RTK (0.02, 0.02)."""
        self.lib.swarm_gps(wander, jitter)

    def arm(self, i):
        return self.lib.swarm_arm(i)

    def land(self, i):
        return self.lib.swarm_land(i)

    def target(self, i, p, v=(0.0, 0.0, 0.0)):
        self.lib.swarm_set_target(i, p[0], p[1], p[2], v[0], v[1], v[2])

    def step(self, ticks=1):
        if ticks == 1:
            self.lib.swarm_step()
        else:
            self.lib.swarm_run(ticks)

    @property
    def t_ms(self):
        return self.lib.swarm_time_ms()

    def state(self, i):
        """A fresh copy of drone i: safe to keep."""
        d = Drone()
        self.lib.swarm_state(i, ctypes.byref(d))
        return d

    def states(self):
        return [self.state(i) for i in range(self.n)]


if __name__ == "__main__":                  # python fleet.py: does it load, and agree?
    lib = load()
    print(f"swarm.dll v{lib.swarm_version()}: swarm_drone_t {lib.swarm_drone_size()} B, "
          f"{len(Drone._fields_)} fields, layout agrees")

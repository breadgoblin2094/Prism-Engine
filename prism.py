"""Python binding for the Prism C++ physics core (ctypes, no dependencies)."""
import ctypes as C, os, random
_lib = C.CDLL(os.path.join(os.path.dirname(os.path.abspath(__file__)), "libprism.so"))
d = C.c_double
_lib.prism_create.restype = C.c_void_p
for name, args, res in [
    ("prism_destroy", [C.c_void_p], None), ("prism_count", [C.c_void_p], C.c_int),
    ("prism_add_sphere", [C.c_void_p, d, d, d, d], C.c_int),
    ("prism_add_box", [C.c_void_p, d, d, d, d, d, d], C.c_int),
    ("prism_set_velocity", [C.c_void_p, C.c_int, d, d, d], None),
    ("prism_set_spin", [C.c_void_p, C.c_int, d, d, d], None),
    ("prism_step", [C.c_void_p, d], None),
    ("prism_get_pos", [C.c_void_p, C.c_int, C.POINTER(d)], None),
    ("prism_get_quat", [C.c_void_p, C.c_int, C.POINTER(d)], None)]:
    f = getattr(_lib, name); f.argtypes = args; f.restype = res

class World:
    def __init__(self): self._w = _lib.prism_create()
    def __del__(self): _lib.prism_destroy(self._w)
    def add_sphere(self, r, pos): return _lib.prism_add_sphere(self._w, r, *pos)
    def add_box(self, half, pos): return _lib.prism_add_box(self._w, *half, *pos)
    def set_velocity(self, i, v): _lib.prism_set_velocity(self._w, i, *v)
    def set_spin(self, i, w): _lib.prism_set_spin(self._w, i, *w)
    def step(self, dt=1/240): _lib.prism_step(self._w, dt)
    def pos(self, i):
        o = (d * 3)(); _lib.prism_get_pos(self._w, i, o); return tuple(o)
    def quat(self, i):
        o = (d * 4)(); _lib.prism_get_quat(self._w, i, o); return tuple(o)
    def __len__(self): return _lib.prism_count(self._w)

if __name__ == "__main__":
    random.seed(1); w = World()
    for _ in range(12):
        p = (random.uniform(-2, 2), random.uniform(3, 8), random.uniform(-2, 2))
        i = (w.add_box([random.uniform(.2, .5) for _ in range(3)], p) if random.random() < .55
             else w.add_sphere(random.uniform(.28, .55), p))
        w.set_spin(i, [random.uniform(-2, 2) for _ in range(3)])
    for t in range(240 * 6): w.step()
    ys = [w.pos(i)[1] for i in range(len(w))]
    print(f"{len(w)} bodies after 6 simulated seconds")
    print("lowest y: %.3f  highest y: %.3f" % (min(ys), max(ys)))
    print("all above floor:", all(y > 0.1 for y in ys), "| none exploded:", all(abs(c) < 6 for i in range(len(w)) for c in w.pos(i)))

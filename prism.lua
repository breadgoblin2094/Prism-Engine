-- Lua binding via LuaJIT FFI. Untested here: no LuaJIT in my sandbox.
local ffi = require("ffi")
ffi.cdef[[
void* prism_create();
int   prism_add_sphere(void* w, double r, double x, double y, double z);
int   prism_add_box(void* w, double hx, double hy, double hz, double x, double y, double z);
void  prism_step(void* w, double dt);
void  prism_get_pos(void* w, int id, double* out3);
]]
local P = ffi.load("./libprism.so")
local w = P.prism_create()
local id = P.prism_add_box(w, .4, .4, .4, 0, 5, 0)
for i = 1, 240 do P.prism_step(w, 1/240) end
local o = ffi.new("double[3]"); P.prism_get_pos(w, id, o)
print("box y after 1s:", o[1])

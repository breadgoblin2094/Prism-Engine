PRISM - tiny physics + rendering engine
=======================================
main.cpp    desktop app (SDL2 + OpenGL 3.3)
prism.cpp   C++ physics core (spheres + rotating boxes) with a C API
prism.py / Prism.cs / prism.lua   bindings for Python, C#, Lua

GET A WINDOWS .EXE (no coding):
1. Make a free GitHub account and a new repository.
2. Upload everything in this folder, keeping .github/workflows/build.yml in place.
3. Open the Actions tab, wait for the build, download "prism-windows" under Artifacts.

BUILD YOURSELF:
Linux:   g++ -std=c++17 -O2 main.cpp -o prism $(sdl2-config --cflags --libs) -lGL
Windows (MSYS2 MinGW64 shell): g++ -std=c++17 -O2 main.cpp -o prism.exe -static $(sdl2-config --cflags --static-libs) -lopengl32
Python demo (Linux): g++ -std=c++17 -O2 -shared -fPIC -o libprism.so prism.cpp && python3 prism.py

Controls: drag = orbit, wheel = zoom, SPACE = spawn, ESC = quit.

// C# binding (Unity / .NET). Put libprism.so (or prism.dll) next to your build. Untested here: no .NET runtime in my sandbox.
using System.Runtime.InteropServices;
public static class Prism {
    const string L = "prism";
    [DllImport(L)] public static extern System.IntPtr prism_create();
    [DllImport(L)] public static extern int  prism_add_sphere(System.IntPtr w, double r, double x, double y, double z);
    [DllImport(L)] public static extern int  prism_add_box(System.IntPtr w, double hx, double hy, double hz, double x, double y, double z);
    [DllImport(L)] public static extern void prism_step(System.IntPtr w, double dt);
    [DllImport(L)] public static extern void prism_get_pos(System.IntPtr w, int id, double[] out3);
}

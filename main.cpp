// Prism desktop app: SDL2 window + OpenGL 3.3 renderer + the C++ physics core.
// Build:  g++ -std=c++17 -O2 main.cpp -o prism $(sdl2-config --cflags --libs) -lGL     (Linux)
// Controls: drag = orbit, wheel = zoom, SPACE = spawn, ESC = quit.
#include <SDL2/SDL.h>
#include <SDL2/SDL_opengl.h>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <random>
#include <vector>
#include <array>
#include "prism.cpp"   // physics core (single translation unit keeps the build one command)

#define MAXB 24

// ---- minimal GL loader (so SDL2 is the only dependency) ----
#define GL_FUNCS \
  X(PFNGLCREATESHADERPROC, CreateShader) X(PFNGLSHADERSOURCEPROC, ShaderSource) X(PFNGLCOMPILESHADERPROC, CompileShader) \
  X(PFNGLGETSHADERIVPROC, GetShaderiv) X(PFNGLGETSHADERINFOLOGPROC, GetShaderInfoLog) X(PFNGLCREATEPROGRAMPROC, CreateProgram) \
  X(PFNGLATTACHSHADERPROC, AttachShader) X(PFNGLLINKPROGRAMPROC, LinkProgram) X(PFNGLUSEPROGRAMPROC, UseProgram) \
  X(PFNGLGETUNIFORMLOCATIONPROC, GetUniformLocation) X(PFNGLUNIFORM4FVPROC, Uniform4fv) X(PFNGLUNIFORM1IPROC, Uniform1i) \
  X(PFNGLUNIFORM2FPROC, Uniform2f) X(PFNGLUNIFORM3FPROC, Uniform3f) X(PFNGLGENVERTEXARRAYSPROC, GenVertexArrays) \
  X(PFNGLBINDVERTEXARRAYPROC, BindVertexArray)
#define X(T, N) static T gl_##N;
GL_FUNCS
#undef X

static const char* VS = R"GLSL(#version 330 core
void main(){ vec2 p=vec2((gl_VertexID<<1)&2, gl_VertexID&2); gl_Position=vec4(p*2.-1.,0.,1.); }
)GLSL";

static const char* FS = R"GLSL(#version 330 core
uniform vec4 uS[24]; uniform vec4 uC[24]; uniform vec4 uH[24]; uniform vec4 uX[24]; uniform vec4 uY[24]; uniform vec4 uZ[24];
uniform int uN; uniform vec3 uCam; uniform vec2 uRes;
out vec4 o;
float map(vec3 p, out float id){
  float d = p.y; id = 0.;
  for(int i=0;i<24;i++){ if(i>=uN) break;
    float s;
    if(uH[i].w<.5) s = length(p-uS[i].xyz)-uS[i].w;
    else { vec3 d3=p-uS[i].xyz; vec3 q=vec3(dot(d3,uX[i].xyz),dot(d3,uY[i].xyz),dot(d3,uZ[i].xyz));
      vec3 w=abs(q)-(uH[i].xyz-.04); s=length(max(w,0.))+min(max(w.x,max(w.y,w.z)),0.)-.04; }
    if(s<d){ d=s; id=float(i+1); } }
  return d; }
float mapD(vec3 p){ float i; return map(p,i); }
vec3 nrm(vec3 p){ vec2 e=vec2(.0015,0.);
  return normalize(vec3(mapD(p+e.xyy)-mapD(p-e.xyy), mapD(p+e.yxy)-mapD(p-e.yxy), mapD(p+e.yyx)-mapD(p-e.yyx))); }
float march(vec3 ro, vec3 rd, out float id){
  float t=.02; id=-1.;
  for(int i=0;i<110;i++){ float d=map(ro+rd*t,id); if(d<.0008*t){return t;} t+=d; if(t>60.) break; }
  id=-1.; return -1.; }
float shadow(vec3 ro, vec3 rd){
  float res=1., t=.03;
  for(int i=0;i<48;i++){ float h=mapD(ro+rd*t); res=min(res,12.*h/t); t+=clamp(h,.02,.6); if(res<.002||t>20.) break; }
  return clamp(res,0.,1.); }
float ao(vec3 p, vec3 n){
  float occ=0., sca=1.;
  for(int i=0;i<5;i++){ float h=.02+.12*float(i); occ+=(h-mapD(p+n*h))*sca; sca*=.85; }
  return clamp(1.-1.6*occ,0.,1.); }
const vec3 SUN = normalize(vec3(.6,.8,.35));
vec3 sky(vec3 rd){
  vec3 c = mix(vec3(.75,.85,1.), vec3(.18,.38,.85), clamp(rd.y*1.2,0.,1.));
  float s = max(dot(rd,SUN),0.);
  c += vec3(1.,.8,.55)*(pow(s,8.)*.25 + pow(s,400.)*6.);
  return c; }
float D_GGX(float nh,float a){ float a2=a*a; float d=nh*nh*(a2-1.)+1.; return a2/(3.14159*d*d); }
float G_S(float nv,float nl,float r){ float k=(r+1.)*(r+1.)/8.; return (nv/(nv*(1.-k)+k))*(nl/(nl*(1.-k)+k)); }
vec3 shade(vec3 p, vec3 n, vec3 rd, float id, out float F0r){
  vec3 alb; float metal=0., rough=.55;
  if(id<.5){ float ck=mod(floor(p.x)+floor(p.z),2.); alb=mix(vec3(.12),vec3(.62),ck); rough=.65; F0r=.04; }
  else { vec4 c=uC[int(id)-1]; alb=c.rgb; metal=c.a; rough=.22; F0r=mix(.04,.7,metal); }
  vec3 v=-rd, l=SUN, h=normalize(v+l);
  float nl=max(dot(n,l),0.), nv=max(dot(n,v),.001), nh=max(dot(n,h),0.), vh=max(dot(v,h),0.);
  vec3 f0=mix(vec3(.04),alb,metal);
  vec3 F=f0+(1.-f0)*pow(1.-vh,5.);
  float sh=shadow(p+n*.004,l);
  vec3 spec=D_GGX(nh,rough*rough)*G_S(nv,nl,rough)*F/(4.*nv*max(nl,.001)+.001);
  vec3 kd=(1.-F)*(1.-metal)*alb/3.14159;
  vec3 sunC=vec3(1.,.93,.8)*3.2;
  float a=ao(p,n);
  vec3 amb=mix(vec3(.12,.1,.09),vec3(.25,.4,.7),n.y*.5+.5)*a;
  return (kd+spec)*sunC*nl*sh + amb*(alb*(1.-metal)+f0*.6); }
void main(){
  vec2 uv=(gl_FragCoord.xy-.5*uRes)/uRes.y;
  vec3 ro=uCam, fw=normalize(vec3(0.,.8,0.)-ro), rt=normalize(cross(fw,vec3(0,1,0))), up=cross(rt,fw);
  vec3 rd=normalize(uv.x*rt+uv.y*up+1.6*fw);
  vec3 col=vec3(0.), thr=vec3(1.);
  for(int b=0;b<2;b++){
    float id; float t=march(ro,rd,id);
    if(t<0.){ col+=thr*sky(rd); break; }
    vec3 p=ro+rd*t, n=nrm(p); float f0;
    vec3 s=shade(p,n,rd,id,f0);
    float fr=f0+(1.-f0)*pow(1.-max(dot(n,-rd),0.),5.);
    float refl = id<.5 ? fr*.6 : fr;
    if(id>.5 && uC[int(id)-1].a>.5) refl=max(refl,.55);
    col+=thr*s*(1.-refl);
    thr*=refl; ro=p+n*.004; rd=reflect(rd,n);
    if(b==1) col+=thr*sky(rd);
  }
  float dist=length(uCam);
  col=mix(col,vec3(.55,.68,.9),1.-exp(-.0007*dist*dist));
  col*=.85; col=(col*(2.51*col+.03))/(col*(2.43*col+.59)+.14);
  col=pow(clamp(col,0.,1.),vec3(1./2.2));
  col*=1.-.25*dot(uv,uv);
  o=vec4(col,1.);
}
)GLSL";

static GLuint compile(GLenum type, const char* src) {
  GLuint s = gl_CreateShader(type); gl_ShaderSource(s, 1, &src, nullptr); gl_CompileShader(s);
  GLint ok = 0; gl_GetShaderiv(s, GL_COMPILE_STATUS, &ok);
  if (!ok) { char log[2048]; gl_GetShaderInfoLog(s, 2048, nullptr, log); std::fprintf(stderr, "Shader error:\n%s\n", log); std::exit(1); }
  return s;
}

static const float PAL[7][4] = {{1,.35,.6,0},{1,.75,.2,0},{.3,.9,.6,0},{.35,.65,1,0},{.75,.5,1,0},{.95,.95,1,1},{1,.8,.4,1}};

int main(int argc, char** argv) {
  (void)argc; (void)argv;
  if (SDL_Init(SDL_INIT_VIDEO) != 0) { std::fprintf(stderr, "SDL_Init: %s\n", SDL_GetError()); return 1; }
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
  SDL_Window* win = SDL_CreateWindow("Prism", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1280, 720,
                                     SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
  if (!win) { std::fprintf(stderr, "Window: %s\n", SDL_GetError()); return 1; }
  SDL_GLContext ctx = SDL_GL_CreateContext(win);
  if (!ctx) { std::fprintf(stderr, "GL context: %s\n", SDL_GetError()); return 1; }
  SDL_GL_SetSwapInterval(1);
#define X(T, N) gl_##N = (T)SDL_GL_GetProcAddress("gl" #N);
  GL_FUNCS
#undef X

  GLuint prog = gl_CreateProgram();
  gl_AttachShader(prog, compile(GL_VERTEX_SHADER, VS));
  gl_AttachShader(prog, compile(GL_FRAGMENT_SHADER, FS));
  gl_LinkProgram(prog); gl_UseProgram(prog);
  GLuint vao; gl_GenVertexArrays(1, &vao); gl_BindVertexArray(vao);
  GLint uS = gl_GetUniformLocation(prog, "uS"), uC = gl_GetUniformLocation(prog, "uC"), uH = gl_GetUniformLocation(prog, "uH"),
        uX = gl_GetUniformLocation(prog, "uX"), uY = gl_GetUniformLocation(prog, "uY"), uZ = gl_GetUniformLocation(prog, "uZ"),
        uN = gl_GetUniformLocation(prog, "uN"), uCam = gl_GetUniformLocation(prog, "uCam"), uRes = gl_GetUniformLocation(prog, "uRes");

  World W; std::vector<std::array<float,4>> colors; std::mt19937 rng(7);
  auto rnd = [&](double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); };
  auto spawn = [&]() {
    double x = rnd(-2.5, 2.5), y = rnd(4, 8), z = rnd(-2.5, 2.5);
    int id = rnd(0, 1) < .55 ? prism_add_box(&W, rnd(.2,.55), rnd(.2,.55), rnd(.2,.55), x, y, z) : prism_add_sphere(&W, rnd(.28,.58), x, y, z);
    prism_set_velocity(&W, id, rnd(-1,1), 0, rnd(-1,1)); prism_set_spin(&W, id, rnd(-2,2), rnd(-2,2), rnd(-2,2));
    Body& b = W.bodies[id]; double q[4] = {rnd(-1,1), rnd(-1,1), rnd(-1,1), rnd(-1,1)};
    double l = std::sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]); for (int i = 0; i < 4; i++) b.q[i] = q[i] / l; b.updateAxes();
    const float* c = PAL[(int)rnd(0, 6.999)]; colors.push_back({c[0], c[1], c[2], c[3]});
    if ((int)W.bodies.size() > MAXB) { W.bodies.erase(W.bodies.begin()); colors.erase(colors.begin()); }
  };
  for (int i = 0; i < 12; i++) spawn();

  float yaw = .6f, pitch = .35f, dist = 11.f; double acc = 0, fpsT = 0; int frames = 0;
  Uint32 last = SDL_GetTicks(); bool run = true;
  while (run) {
    SDL_Event e;
    while (SDL_PollEvent(&e)) {
      if (e.type == SDL_QUIT) run = false;
      else if (e.type == SDL_KEYDOWN) { if (e.key.keysym.sym == SDLK_ESCAPE) run = false; if (e.key.keysym.sym == SDLK_SPACE) { spawn(); spawn(); spawn(); } }
      else if (e.type == SDL_MOUSEMOTION && (e.motion.state & SDL_BUTTON_LMASK)) {
        yaw -= e.motion.xrel * .006f; pitch = std::fmax(.05f, std::fmin(1.3f, pitch + e.motion.yrel * .005f)); }
      else if (e.type == SDL_MOUSEWHEEL) dist = std::fmax(5.f, std::fmin(20.f, dist - e.wheel.y * .6f));
    }
    Uint32 now = SDL_GetTicks(); double dt = std::fmin((now - last) / 1000.0, .05); last = now;
    acc += dt; while (acc >= 1.0 / 240) { W.step(1.0 / 240); acc -= 1.0 / 240; }

    float S[MAXB*4] = {}, C[MAXB*4] = {}, H[MAXB*4] = {}, AX[MAXB*4] = {}, AY[MAXB*4] = {}, AZ[MAXB*4] = {};
    for (size_t i = 0; i < W.bodies.size(); i++) {
      const Body& b = W.bodies[i];
      S[i*4]=b.p.x; S[i*4+1]=b.p.y; S[i*4+2]=b.p.z; S[i*4+3]=b.r;
      H[i*4]=b.h.x; H[i*4+1]=b.h.y; H[i*4+2]=b.h.z; H[i*4+3]=b.box ? 1.f : 0.f;
      for (int k = 0; k < 4; k++) C[i*4+k] = colors[i][k];
      AX[i*4]=b.A[0].x; AX[i*4+1]=b.A[0].y; AX[i*4+2]=b.A[0].z;
      AY[i*4]=b.A[1].x; AY[i*4+1]=b.A[1].y; AY[i*4+2]=b.A[1].z;
      AZ[i*4]=b.A[2].x; AZ[i*4+1]=b.A[2].y; AZ[i*4+2]=b.A[2].z;
    }
    int w, h; SDL_GL_GetDrawableSize(win, &w, &h); glViewport(0, 0, w, h);
    gl_Uniform4fv(uS, MAXB, S); gl_Uniform4fv(uC, MAXB, C); gl_Uniform4fv(uH, MAXB, H);
    gl_Uniform4fv(uX, MAXB, AX); gl_Uniform4fv(uY, MAXB, AY); gl_Uniform4fv(uZ, MAXB, AZ);
    gl_Uniform1i(uN, (int)W.bodies.size());
    gl_Uniform3f(uCam, std::sin(yaw)*std::cos(pitch)*dist, .8f + std::sin(pitch)*dist, std::cos(yaw)*std::cos(pitch)*dist);
    gl_Uniform2f(uRes, (float)w, (float)h);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    SDL_GL_SwapWindow(win);

    frames++; fpsT += dt;
    if (fpsT >= .5) { char t[96]; std::snprintf(t, sizeof t, "Prism | %zu bodies | %.0f fps", W.bodies.size(), frames / fpsT); SDL_SetWindowTitle(win, t); frames = 0; fpsT = 0; }
  }
  SDL_GL_DeleteContext(ctx); SDL_DestroyWindow(win); SDL_Quit();
  return 0;
}

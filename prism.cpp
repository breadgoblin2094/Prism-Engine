// Prism physics core (C++17). Spheres + oriented boxes, sequential-impulse solver.
// Exposes a plain C API so Python / C# / Lua / anything with an FFI can drive it.
#include <cmath>
#include <vector>
#include <algorithm>

struct V3 { double x,y,z; };
static V3 operator+(V3 a,V3 b){return {a.x+b.x,a.y+b.y,a.z+b.z};}
static V3 operator-(V3 a,V3 b){return {a.x-b.x,a.y-b.y,a.z-b.z};}
static V3 operator*(V3 a,double s){return {a.x*s,a.y*s,a.z*s};}
static double dot(V3 a,V3 b){return a.x*b.x+a.y*b.y+a.z*b.z;}
static V3 cross(V3 a,V3 b){return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
static double len(V3 a){return std::sqrt(dot(a,a));}
static double at(const V3&a,int i){return i==0?a.x:i==1?a.y:a.z;}

struct Body {
  bool box=false; V3 p{},v{},w{},h{},iI{}; double q[4]={0,0,0,1}; V3 A[3]; double im=0, r=0;
  void updateAxes(){
    double x=q[0],y=q[1],z=q[2],s=q[3];
    A[0]={1-2*(y*y+z*z),2*(x*y+z*s),2*(x*z-y*s)};
    A[1]={2*(x*y-z*s),1-2*(x*x+z*z),2*(y*z+x*s)};
    A[2]={2*(x*z+y*s),2*(y*z-x*s),1-2*(x*x+y*y)}; }
  V3 invI(V3 v) const { V3 r{0,0,0}; for(int k=0;k<3;k++) r=r+A[k]*(dot(A[k],v)*at(iI,k)); return r; }
  V3 local(V3 pt) const { V3 d=pt-p; return {dot(d,A[0]),dot(d,A[1]),dot(d,A[2])}; }
};
struct Contact { Body *a,*b; V3 n,ra,rb; double jn=0,bias=0; };

struct World {
  std::vector<Body> bodies; std::vector<Contact> cs; Body stat;
  double G=-9.81, B=5.5, E=0.35, MU=0.6;
  World(){ stat.A[0]={1,0,0}; stat.A[1]={0,1,0}; stat.A[2]={0,0,1}; stat.iI={0,0,0}; }

  static V3 relV(const Contact&c){ return (c.b->v+cross(c.b->w,c.rb))-(c.a->v+cross(c.a->w,c.ra)); }
  static double den(const Contact&c,V3 d){
    return c.a->im+c.b->im+dot(d,cross(c.a->invI(cross(c.ra,d)),c.ra))+dot(d,cross(c.b->invI(cross(c.rb,d)),c.rb)); }
  static void imp(Contact&c,V3 P){
    c.a->v=c.a->v-P*c.a->im; c.a->w=c.a->w-c.a->invI(cross(c.ra,P));
    c.b->v=c.b->v+P*c.b->im; c.b->w=c.b->w+c.b->invI(cross(c.rb,P)); }
  void addC(Body*a,Body*b,V3 pt,V3 n,double pen){
    double s=a->im+b->im, k=std::max(pen-0.005,0.0)*0.25/s;
    a->p=a->p-n*(k*a->im); b->p=b->p+n*(k*b->im);
    Contact c{a,b,n,pt-a->p,pt-b->p}; double vn=dot(relV(c),n); c.bias=vn<-1?-E*vn:0; cs.push_back(c); }
  static void verts(const Body&b,V3*o){ int i=0;
    for(int x=-1;x<=1;x+=2)for(int y=-1;y<=1;y+=2)for(int z=-1;z<=1;z+=2)
      o[i++]=b.p+b.A[0]*(x*b.h.x)+b.A[1]*(y*b.h.y)+b.A[2]*(z*b.h.z); }
  static bool vertInBox(V3 v,const Body&bx,V3&n,double&pen){
    V3 l=bx.local(v); int k=-1; double best=1e9;
    for(int i=0;i<3;i++){ double p=at(bx.h,i)-std::fabs(at(l,i)); if(p<0) return false; if(p<best){best=p;k=i;} }
    n=bx.A[k]*(at(l,k)<0?-1.0:1.0); pen=best; return true; }

  void collect(){
    cs.clear();
    const V3 pn[5]={{0,1,0},{1,0,0},{-1,0,0},{0,0,1},{0,0,-1}}; const double pd[5]={0,-B,-B,-B,-B};
    for(auto&b:bodies) for(int i=0;i<5;i++){
      if(!b.box){ double pen=pd[i]+b.r-dot(b.p,pn[i]); if(pen>0) addC(&stat,&b,b.p-pn[i]*b.r,pn[i],pen); }
      else if(dot(b.p,pn[i])-pd[i]<b.r){ V3 vs[8]; verts(b,vs);
        for(auto&v:vs){ double pen=pd[i]-dot(v,pn[i]); if(pen>0) addC(&stat,&b,v,pn[i],pen); } } }
    for(size_t i=0;i<bodies.size();i++) for(size_t j=i+1;j<bodies.size();j++){
      Body&a=bodies[i]; Body&b=bodies[j]; V3 D=b.p-a.p; double dl=len(D);
      if(dl>a.r+b.r||dl<1e-6) continue;
      if(!a.box&&!b.box){ V3 n=D*(1/dl); addC(&a,&b,a.p+n*a.r,n,a.r+b.r-dl); }
      else if(a.box&&b.box){ V3 vs[8],n; double pen;
        verts(a,vs); for(auto&v:vs) if(vertInBox(v,b,n,pen)) addC(&a,&b,v,n*-1.0,pen);
        verts(b,vs); for(auto&v:vs) if(vertInBox(v,a,n,pen)) addC(&a,&b,v,n,pen); }
      else { Body&bx=a.box?a:b; Body&sp=a.box?b:a; V3 l=bx.local(sp.p);
        V3 c{std::clamp(l.x,-bx.h.x,bx.h.x),std::clamp(l.y,-bx.h.y,bx.h.y),std::clamp(l.z,-bx.h.z,bx.h.z)};
        V3 dv=l-c; double dd=len(dv); V3 out; double pen;
        if(dd>1e-6){ if(dd>=sp.r) continue; pen=sp.r-dd; out=bx.A[0]*(dv.x/dd)+bx.A[1]*(dv.y/dd)+bx.A[2]*(dv.z/dd); }
        else { int k=0; double m=1e9; for(int q=0;q<3;q++){ double f=at(bx.h,q)-std::fabs(at(l,q)); if(f<m){m=f;k=q;} }
          double s=at(l,k)<0?-1.0:1.0; out=bx.A[k]*s; pen=sp.r+m;
          double* pc=&c.x; pc[k]=s*at(bx.h,k); }
        V3 pt=bx.p+bx.A[0]*c.x+bx.A[1]*c.y+bx.A[2]*c.z;
        addC(&a,&b,pt,a.box?out:out*-1.0,pen); } } }

  void step(double dt){
    for(auto&b:bodies) b.v.y+=G*dt;
    collect();
    for(int it=0;it<10;it++) for(auto&c:cs){
      V3 vr=relV(c); double vn=dot(vr,c.n), dj=-(vn-c.bias)/den(c,c.n), nj=std::max(c.jn+dj,0.0);
      dj=nj-c.jn; c.jn=nj; imp(c,c.n*dj);
      vr=relV(c); vn=dot(vr,c.n); V3 vt=vr-c.n*vn; double ts=len(vt);
      if(ts>1e-6){ V3 t=vt*(1/ts); double jt=std::min(ts/den(c,t),MU*c.jn); imp(c,t*-jt); } }
    for(auto&b:bodies){
      b.v=b.v*(1-0.05*dt); b.w=b.w*(1-0.6*dt); b.p=b.p+b.v*dt;
      double x=b.q[0],y=b.q[1],z=b.q[2],s=b.q[3],h=dt*0.5; V3 o=b.w;
      double q[4]={x+h*(o.x*s+o.y*z-o.z*y),y+h*(o.y*s+o.z*x-o.x*z),z+h*(o.z*s+o.x*y-o.y*x),s-h*(o.x*x+o.y*y+o.z*z)};
      double l=std::sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]); for(int i=0;i<4;i++) b.q[i]=q[i]/l;
      b.updateAxes(); } }
};

// ---------------- C API (the "binding surface") ----------------
extern "C" {
void* prism_create(){ return new World(); }
void  prism_destroy(void*w){ delete (World*)w; }
int   prism_count(void*w){ return (int)((World*)w)->bodies.size(); }
int prism_add_sphere(void*w,double r,double x,double y,double z){
  Body b; b.r=r; b.h={r,r,r}; b.p={x,y,z}; double m=r*r*r*4.2; b.im=1/m; double i=2.5/(m*r*r); b.iI={i,i,i};
  b.updateAxes(); auto&W=*(World*)w; W.bodies.push_back(b); return (int)W.bodies.size()-1; }
int prism_add_box(void*w,double hx,double hy,double hz,double x,double y,double z){
  Body b; b.box=true; b.h={hx,hy,hz}; b.r=len(b.h); b.p={x,y,z}; double m=8*hx*hy*hz; b.im=1/m;
  b.iI={3/(m*(hy*hy+hz*hz)),3/(m*(hx*hx+hz*hz)),3/(m*(hx*hx+hy*hy))};
  b.updateAxes(); auto&W=*(World*)w; W.bodies.push_back(b); return (int)W.bodies.size()-1; }
void prism_set_velocity(void*w,int id,double x,double y,double z){ ((World*)w)->bodies[id].v={x,y,z}; }
void prism_set_spin(void*w,int id,double x,double y,double z){ ((World*)w)->bodies[id].w={x,y,z}; }
void prism_step(void*w,double dt){ ((World*)w)->step(dt); }
void prism_get_pos(void*w,int id,double*o){ auto&b=((World*)w)->bodies[id]; o[0]=b.p.x;o[1]=b.p.y;o[2]=b.p.z; }
void prism_get_quat(void*w,int id,double*o){ auto&b=((World*)w)->bodies[id]; for(int i=0;i<4;i++) o[i]=b.q[i]; }
}

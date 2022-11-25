//////////////////////////////////////////////////////////////////////////////////////// OCEAN
// https://www.shadertoy.com/view/Ms2SD1
/*
 * "Seascape" by Alexander Alekseev aka TDM - 2014
 * License Creative Commons Attribution-NonCommercial-ShareAlike 3.0 Unported License.
 * Contact: tdmaav@gmail.com
 */
#version 420 core

layout (std140) uniform CameraData{
	mat4 ViewMatrix;
	mat4 ProjectionMatrix;
	vec4 Position;
}Camera;

layout(binding=0) uniform sampler2D texColor;
layout(binding=1) uniform sampler2D texPosition;
layout(binding=2) uniform sampler2D texNormal;

uniform float mapHeight;
uniform float uTime;
uniform float aspectRatio;
uniform vec3 sunDir;
uniform ivec2 uResolution;

in vec2 TexCoords;
out vec4 FragColor;

const float infinity = 1.0 / 0.0;

const int NUM_STEPS = 4;
const float PI	 	= 3.141592;
const float EPSILON	= 1e-3;
#define EPSILON_NRM 0.1/uResolution.x//(0.1 / iResolution.x)
//#define AA

// sea
const int ITER_GEOMETRY = 2;
const int ITER_FRAGMENT = 3;
const float SEA_HEIGHT = 0.6;
const float SEA_CHOPPY = 4.0;
const float SEA_SPEED = 0.8;
const float SEA_FREQ = 0.16;
vec3 SEA_BASE = vec3(0.0,0.09,0.18);
const vec3 SEA_WATER_COLOR = vec3(0.8,0.9,0.6)*0.6;
#define SEA_TIME (1.0 + uTime * SEA_SPEED)
const mat2 octave_m = mat2(1.6,1.2,-1.2,1.6);

// math
mat3 fromEuler(vec3 ang) {
	vec2 a1 = vec2(sin(ang.x),cos(ang.x));
	vec2 a2 = vec2(sin(ang.y),cos(ang.y));
	vec2 a3 = vec2(sin(ang.z),cos(ang.z));
	mat3 m;
	m[0] = vec3(a1.y*a3.y+a1.x*a2.x*a3.x,a1.y*a2.x*a3.x+a3.y*a1.x,-a2.y*a3.x);
	m[1] = vec3(-a2.y*a1.x,a1.y*a2.y,a2.x);
	m[2] = vec3(a3.y*a1.x*a2.x+a1.y*a3.x,a1.x*a3.x-a1.y*a3.y*a2.x,a2.y*a3.y);
	return m;
}
float hash( vec2 p ) {
	float h = dot(p,vec2(127.1,311.7));	
	return fract(sin(h)*43758.5453123);
}
float noise( in vec2 p ) {
	vec2 i = floor( p );
	vec2 f = fract( p );
	vec2 u = f*f*(3.0-2.0*f);
	return -1.0+2.0*mix( mix( hash( i + vec2(0.0,0.0) ), 
					 hash( i + vec2(1.0,0.0) ), u.x),
				mix( hash( i + vec2(0.0,1.0) ), 
					 hash( i + vec2(1.0,1.0) ), u.x), u.y);
}

// lighting
float diffuse(vec3 n,vec3 l,float p) {
	return pow(dot(n,l) * 0.4 + 0.6,p);
}
float specular(vec3 n,vec3 l,vec3 e,float s) {    
	float nrm = (s + 8.0) / (PI * 8.0);
	return pow(max(dot(reflect(e,n),l),0.0),s) * nrm;
}

// sky
vec3 getSkyColor(vec3 e) {
	e.y = (max(e.y,0.0)*0.8+0.2)*0.8;
	return vec3(pow(1.0-e.y,2.0), 1.0-e.y, 0.6+(1.0-e.y)*0.4) * 1.1;
}

// sea
float sea_octave(vec2 uv, float choppy) {
	uv += noise(uv);        
	vec2 wv = 1.0-abs(sin(uv));
	vec2 swv = abs(cos(uv));    
	wv = mix(wv,swv,wv);
	return pow(1.0-pow(wv.x * wv.y,0.65),choppy);
}

float map(vec3 p) {
	float freq = SEA_FREQ;
	float amp = SEA_HEIGHT;
	float choppy = SEA_CHOPPY;
	vec2 uv = p.xz; uv.x *= 0.75;
	
	float d, h = 0.0;    
	for(int i = 0; i < ITER_GEOMETRY; i++) {        
		d = sea_octave((uv+SEA_TIME)*freq,choppy);
		d += sea_octave((uv-SEA_TIME)*freq,choppy);
		h += d * amp;        
		uv *= octave_m; freq *= 1.9; amp *= 0.22;
		choppy = mix(choppy,1.0,0.2);
	}
	return p.y - h;
}

float map_detailed(vec3 p) {
	float freq = SEA_FREQ;
	float amp = SEA_HEIGHT;
	float choppy = SEA_CHOPPY;
	vec2 uv = p.xz; uv.x *= 0.75;
	
	float d, h = 0.0;    
	for(int i = 0; i < ITER_FRAGMENT; i++) {        
		d = sea_octave((uv+SEA_TIME)*freq,choppy);
		d += sea_octave((uv-SEA_TIME)*freq,choppy);
		h += d * amp;        
		uv *= octave_m; freq *= 1.9; amp *= 0.22;
		choppy = mix(choppy,1.0,0.2);
	}
	return p.y - h;
}

vec3 lightColor = vec3(1.0,1.0,1.0);

vec3 getSeaColor(vec3 p, vec3 n, vec3 l, vec3 eye, vec3 dist) {  
	float fresnel = clamp(1.0 - dot(n,-eye), 0.0, 1.0);
	fresnel = pow(fresnel,3.0) * 0.5;
		
	vec3 reflected = (SEA_BASE.y/0.18+SEA_BASE.z/0.36)*vec3(0.37, 0.55, 1.0); //getSkyColor(reflect(eye,n));
	vec3 refracted = SEA_BASE + diffuse(n,l,80.0) * SEA_WATER_COLOR * 0.12;
	
	vec3 color = mix(refracted,reflected,fresnel);
	
	float atten = max(1.0 - dot(dist,dist) * 0.001, 0.0);
	color += SEA_WATER_COLOR * (p.y - SEA_HEIGHT) * 0.18 * atten;
	
	color += vec3(specular(n,l,eye,60.0))*lightColor;
	
	return color;
}

// tracing
vec3 getNormal(vec3 p, float eps) {
	vec3 n;
	n.y = map_detailed(p);    
	n.x = map_detailed(vec3(p.x+eps,p.y,p.z)) - n.y;
	n.z = map_detailed(vec3(p.x,p.y,p.z+eps)) - n.y;
	n.y = eps;
	return normalize(n);
}

float heightMapTracing(vec3 ori, vec3 dir, out vec3 p) {  
	float tm = 0.0;
	float tx = 10000.0;    
	float hx = map(ori + dir * tx);
	if(hx > 0.0) {
		p = ori + dir * tx;
		return tx;   
	}
	float hm = map(ori + dir * tm);    
	float tmid = 0.0;
	for(int i = 0; i < NUM_STEPS; i++) {
		tmid = mix(tm,tx, hm/(hm-hx));                   
		p = ori + dir * tmid;                   
		float hmid = map(p);
		if(hmid < 0.0) {
			tx = tmid;
			hx = hmid;
		} else {
			tm = tmid;
			hm = hmid;
		}
	}
	return tmid;
}
//////////////////////////////////////////////////////////////////////////////////////// OCEAN

//////////////////////////////////////////////////////////////////////////////////////// SKY

// The sun, the sky and the clouds. By StillTravelling
// https://www.shadertoy.com/view/tdSXzD
// Very much a messy hack sorry!!

// Many Thank yous go to the below for their amazing work
// Day and night sky cycle. By László Matuska (@BitOfGold)
// Creates a sky texture for a skydome
// https://www.shadertoy.com/view/ltlSWB

// Weather. By David Hoskins, May 2014.
// https://www.shadertoy.com/view/4dsXWn

// Edge of atmosphere
// created by dmytro rubalskyi (ruba)
// https://www.shadertoy.com/view/XlXGzB

// Auroras
// created by nimitz
// https://www.shadertoy.com/view/XtGGRt

// Sorry to those I've missed out!!

#define ORIG_CLOUD 0
#define ENABLE_RAIN 0 //enable rain drops on screen
#define NICE_HACK_SUN 1
#define SOFT_SUN 0
#define cloudy  0.5 //0.0 clear sky
#define haze  0.01 * (cloudy*20.)
#define rainmulti 5.0 // makes clouds thicker
#define rainy (10.0 -rainmulti)
#define t uTime
float fov = 0.0;
#define S(x, y, z) smoothstep(x, y, z)
#define cameraheight 5e1 //50.
#define mincloudheight 5e3 //5e3
#define maxcloudheight 8e3 //8e3
#define xaxiscloud t*5e2 //t*5e2 +t left -t right *speed
#define yaxiscloud 0. //0.
#define zaxiscloud t*6e2 //t*6e2 +t away from horizon -t towards horizon *speed
#define cloudnoise 2e-4 //2e-4

//#define cloud2


//Performance
const int steps = 8; //16 is fast, 128 or 256 is extreme high
const int stepss = 4; //16 is fast, 16 or 32 is high 

//Environment
const float R0 = 6360e3; //planet radius //6360e3 actual 6371km
const float Ra = 6380e3; //atmosphere radius //6380e3 troposphere 8 to 14.5km
const float I = 7.5; //sun light power, 10.0 is normal
const float SI = 5.; //sun intensity for sun
const float g = 0.45; //light concentration .76 //.45 //.6  .45 is normaL
const float g2 = g * g;

const float s = 0.999; //light concentration for sun
#if SOFT_SUN
const float s2 = s;
#else
const float s2 = s * s;
#endif
const float Hr = 8e3; //Rayleigh scattering top //8e3
const float Hm = 1.2e3; //Mie scattering top //1.3e3

vec3 bM = vec3(21e-6); //normal mie // vec3(21e-6)
//vec3 bM = vec3(50e-6); //high mie

//Rayleigh scattering (sky color, atmospheric up to 8km)
vec3 bR = vec3(5.8e-6, 13.5e-6, 33.1e-6); //normal earth
//vec3 bR = vec3(5.8e-6, 33.1e-6, 13.5e-6); //purple
//vec3 bR = vec3( 63.5e-6, 13.1e-6, 50.8e-6 ); //green
//vec3 bR = vec3( 13.5e-6, 23.1e-6, 115.8e-6 ); //yellow
//vec3 bR = vec3( 5.5e-6, 15.1e-6, 355.8e-6 ); //yeellow
//vec3 bR = vec3(3.5e-6, 333.1e-6, 235.8e-6 ); //red-purple

vec3 C = vec3(0., -R0, 0.); //planet center
vec3 Ds = normalize(vec3(0., 0., -1.)); //sun direction?

float cloudyhigh = 0.05; //if cloud2 defined

#if ORIG_CLOUD
float cloudnear = 1.0; //9e3 12e3  //do not render too close clouds on the zenith
float cloudfar = 1e3; //15e3 17e3
#else
float cloudnear = 1.0; //15e3 17e3
float cloudfar = 70e3; //160e3  //do not render too close clouds on the horizon 160km should be max for cumulus
#endif




//AURORA STUFF
mat2 mm2(in float a){
	float c = cos(a);
	float s = sin(a);
	return mat2(c,s,-s,c);
}

mat2 m2 = mat2(0.95534, 0.29552, -0.29552, 0.95534);

float tri(in float x){
	return clamp(abs(fract(x)-.5),0.01,0.49);
}

vec2 tri2(in vec2 p){
	return vec2(tri(p.x)+tri(p.y),tri(p.y+tri(p.x)));
}

float triNoise2d(in vec2 p, float spd)
{
	float z=1.8;
	float z2=2.5;
	float rz = 0.;
	p *= mm2(p.x*0.06);
	vec2 bp = p;
	for (float i=0.; i<5.; i++ )
	{
		vec2 dg = tri2(bp*1.85)*.75;
		dg *= mm2(t*spd);
		p -= dg/z2;

		bp *= 1.3;
		z2 *= 1.45;
		z *= .42;
		p *= 1.21 + (rz-1.0)*.02;
		
		rz += tri(p.x+tri(p.y))*z;
		p*= -m2;
	}
	return clamp(1./pow(rz*29., 1.3),0.,.55);
}


float hash21(in vec2 n){ return fract(sin(dot(n, vec2(12.9898, 4.1414))) * 43758.5453); }
vec4 aurora(vec3 ro, vec3 rd)
{
	vec4 col = vec4(0);
	vec4 avgCol = vec4(0);
	ro *= 1e-5;
	float mt = 10.;
	for(float i=0.;i<5.;i++)
	{
		float of = 0.006*hash21(gl_FragCoord.xy)*smoothstep(0.,15., i*mt);
		float pt = ((.8+pow((i*mt),1.2)*.001)-rd.y)/(rd.y*2.+0.4);
		pt -= of;
		vec3 bpos = (ro) + pt*rd;
		vec2 p = bpos.zx;
		//vec2 p = rd.zx;
		float rzt = triNoise2d(p, 0.1);
		vec4 col2 = vec4(0,0,0, rzt);
		col2.rgb = (sin(1.-vec3(2.15,-.5, 1.2)+(i*mt)*0.053)*(0.5*mt))*rzt;
		avgCol =  mix(avgCol, col2, .5);
		col += avgCol*exp2((-i*mt)*0.04 - 2.5)*smoothstep(0.,5., i*mt);

	}

	col *= (clamp(rd.y*15.+.4,0.,1.2));
	return col*2.8;
}

//END AURORA STUFF

// TODO find better noise function
float Noise( in vec3 x )
{
	return (noise(x.xy)*0.3+noise(x.yz)*0.3+noise(x.zx)*0.3)*10.0;
}

float fnoise( vec3 p, in float t )
{
	p *= .25;
	float f;

	f = 0.5000 * Noise(p); p = p * 3.02; p.y -= t*.1; //t*.05 speed cloud changes
	f += 0.2500 * Noise(p); p = p * 3.03; p.y += t*.06;
	f += 0.1250 * Noise(p); p = p * 3.01;
	f += 0.0625   * Noise(p); p =  p * 3.03;
	f += 0.03125  * Noise(p); p =  p * 3.02;
	f += 0.015625 * Noise(p);
	return f;
}

float cloud(vec3 p, in float t ) {
	float cld = fnoise(p*cloudnoise,t) + cloudy*0.1 ;
	cld = smoothstep(.4+.04, .6+.04, cld);
	cld *= cld * (5.0*rainmulti);
	return cld+haze;
}

void densities(in vec3 pos, out float rayleigh, out float mie) {
	float h = length(pos - C) - R0;
	rayleigh =  exp(-h/Hr);
	vec3 d = pos;
	d.y = 0.0;
	float dist = length(d);
	
	float cld = 0.;
	if (mincloudheight < h && h < maxcloudheight) {
		//cld = cloud(pos+vec3(t*1e3,0., t*1e3),t)*cloudy;
		cld = cloud(pos+vec3(xaxiscloud,yaxiscloud, zaxiscloud),t)*cloudy; //direction and speed the cloud movers
		cld *= sin(3.1415*(h-mincloudheight)/mincloudheight) * cloudy;
	}
	#ifdef cloud2
		float cld2 = 0.;
		if (12e3 < h && h < 15.5e3) {
			cld2 = fnoise(pos*3e-4,t)*cloud(pos*32.0+vec3(27612.3, 0.,-t*15e3), t);
			cld2 *= sin(3.1413*(h-12e3)/12e3) * cloudyhigh;
			cld2 = clamp(cld2,0.0,1.0);
		}
	
	#endif

	#if ORIG_CLOUD
	if (dist<cloudfar) {
		float factor = clamp(1.0-((cloudfar - dist)/(cloudfar-cloudnear)),0.0,1.0);
		cld *= factor;
	}
	#else

	if (dist>cloudfar) {

		float factor = clamp(1.0-((dist - cloudfar)/(cloudfar-cloudnear)),0.0,1.0);
		cld *= factor;
	}
	#endif

	mie = exp(-h/Hm) + cld + haze;
	#ifdef cloud2
		mie += cld2;
	#endif
	
}



float escape(in vec3 p, in vec3 d, in float R) {
	vec3 v = p - C;
	float b = dot(v, d);
	float c = dot(v, v) - R*R;
	float det2 = b * b - c;
	if (det2 < 0.) return -1.;
	float det = sqrt(det2);
	float t1 = -b - det, t2 = -b + det;
	return (t1 >= 0.) ? t1 : t2;
}

// this can be explained: http://www.scratchapixel.com/lessons/3d-advanced-lessons/simulating-the-colors-of-the-sky/atmospheric-scattering/
void scatter(vec3 o, vec3 d, out vec3 col, out vec3 scat, in float t) {
	
	float L = escape(o, d, Ra);	
	float mu = dot(d, Ds);
	float opmu2 = 1. + mu*mu;
	float phaseR = .0596831 * opmu2;
	float phaseM = .1193662 * (1. - g2) * opmu2 / ((2. + g2) * pow(1. + g2 - 2.*g*mu, 1.5));
	float phaseS = .1193662 * (1. - s2) * opmu2 / ((2. + s2) * pow(1. + s2 - 2.*s*mu, 1.5));
	
	float depthR = 0., depthM = 0.;
	vec3 R = vec3(0.), M = vec3(0.);
	
	float dl = L / float(steps);
	for (int i = 0; i < steps; ++i) {
		float l = float(i) * dl;
		vec3 p = (o + d * l);

		float dR, dM;
		densities(p, dR, dM);
		dR *= dl; dM *= dl;
		depthR += dR;
		depthM += dM;

		float Ls = escape(p, Ds, Ra);
		if (Ls > 0.) {
			float dls = Ls / float(stepss);
			float depthRs = 0., depthMs = 0.;
			for (int j = 0; j < stepss; ++j) {
				float ls = float(j) * dls;
				vec3 ps = ( p + Ds * ls );
				float dRs, dMs;
				densities(ps, dRs, dMs);
				depthRs += dRs * dls;
				depthMs += dMs * dls;
			}

			vec3 A = exp(-(bR * (depthRs + depthR) + bM * (depthMs + depthM)));
			R += (A * dR);
			M += A * dM ;
		} else {
		}
	}

	//col = (I) * (R * bR * phaseR + M * bM * (phaseM ));
	col = (I) *(M * bM * (phaseM )); // Mie scattering
	#if NICE_HACK_SUN
	col += (SI) *(M * bM *phaseS); //Sun
	#endif
	col += (I) *(R * bR * phaseR); //Rayleigh scattering
	scat = 0.1 *(bM*depthM);
	//scat = 0.0 + clamp(depthM*5e-7,0.,1.); 
}


vec3 hash33(vec3 p)
{
	p = fract(p * vec3(443.8975,397.2973, 491.1871));
	p += dot(p.zxy, p.yxz+19.27);
	return fract(vec3(p.x*p.y, p.z*p.x, p.y*p.z));
}

vec3 stars(in vec3 p)
{
	vec3 c = vec3(0.);
	float res = float(uResolution.x)*2.5;

	for (float i=0.;i<4.;i++)
	{
		vec3 q = fract(p*(.15*res))-0.5;
		vec3 id = floor(p*(.15*res));
		vec2 rn = hash33(id).xy;
		float c2 = 1.-smoothstep(0.,.6,length(q));
		c2 *= step(rn.x,.0005+i*i*0.001);
		c += c2*(mix(vec3(1.0,0.49,0.1),vec3(0.75,0.9,1.),rn.y)*0.1+0.9);
		p *= 1.3;
	}
	return c*c*.8;
}

//SIMPLE SUN STUFF
//const float PI = 3.14159265358979323846;
const float density = 0.5;
const float zenithOffset = 0.48;
const vec3 skyColor = vec3(0.37, 0.55, 1.0) * (1.0 + 0.0);

#define zenithDensity(x) density / pow(max(x - zenithOffset, 0.0035), 0.75)

float getSunPoint(vec2 p, vec2 lp){
return smoothstep(0.04*(fov/2.0), 0.026*(fov/2.0), distance(p, lp)) * 50.0;
}

float getMie(vec2 p, vec2 lp){
	float mytest = lp.y < 0.5 ? (lp.y+0.5)*pow(0.05,20.0):0.05;
	float disk = clamp(1.0 - pow(distance(p, lp), mytest), 0.0, 1.0);
	return disk*disk*(3.0 - 2.0 * disk) * 0.25 * PI;
}

vec3 getSkyAbsorption(vec3 x, float y){
	vec3 absorption = x * y;
	absorption = pow(absorption, 1.0 - (y + absorption) * 0.5) / x / y;
	return absorption;
}

vec3 jodieReinhardTonemap(vec3 c){
	float l = dot(c, vec3(0.2126, 0.7152, 0.0722));
	vec3 tc = c / (c + 1.0);
	return mix(c / (l + 1.0), tc, tc);
}

vec3 getAtmosphericScattering(vec2 p, vec2 lp){
	float zenithnew = zenithDensity(p.y);
	float sunPointDistMult =  clamp(length(max(lp.y + 0.1 - zenithOffset, 0.0)), 0.0, 1.0);
	vec3 absorption = getSkyAbsorption(skyColor, zenithnew);
	vec3 sunAbsorption = getSkyAbsorption(skyColor, zenithDensity(lp.y + 0.1));
	vec3 sun3 = getSunPoint(p, lp) * absorption;
	vec3 mie2 = getMie(p, lp) * sunAbsorption;
	vec3 totalSky = sun3; //+ mie2;
	totalSky *= sunAbsorption * 0.5 + 0.5 * length(sunAbsorption);
	vec3 newSky = jodieReinhardTonemap(totalSky);
	return newSky;
}
//END SIMPLE SUN STUFF

//RAIN STUFF
vec3 N31(float p) {
	//  3 out, 1 in... DAVE HOSKINS
   vec3 p3 = fract(vec3(p) * vec3(.1031,.11369,.13787));
   p3 += dot(p3, p3.yzx + 19.19);
   return fract(vec3((p3.x + p3.y)*p3.z, (p3.x+p3.z)*p3.y, (p3.y+p3.z)*p3.x));
}

float SawTooth(float t) {
	return cos(t+cos(t))+sin(2.*t)*.2+sin(4.*t)*.02;
}

float DeltaSawTooth(float t) {
	return 0.4*cos(2.*t)+0.08*cos(4.*t) - (1.-sin(t))*sin(t+cos(t));
}  

vec2 GetDrops(vec2 uv, float seed, float m) {
	
	float t2 = t+m;
	vec2 o = vec2(0.);

	#ifndef DROP_DEBUG
	uv.y += t2*.05;
	#endif
	
	uv *= vec2(10., 2.5)*2.;
	vec2 id = floor(uv);
	vec3 n = N31(id.x + (id.y+seed)*546.3524);
	vec2 bd = fract(uv);
	
	vec2 uv2 = bd;
	
	bd -= 0.5;
	
	bd.y*=4.;

	bd.x += (n.x-.5)*rainy;
	
	t2 += n.z * 6.28;
	float slide = SawTooth(t2);
	
	float ts = 1.5; //(cameraheight / 2.5e5); ?
	vec2 trailPos = vec2(bd.x*ts, (fract(bd.y*ts*2.-t2*2.)-.5)*.5);
	
	bd.y += slide*2.;								// make drops slide down
	
	#ifdef HIGH_QUALITY
	float dropShape = bd.x*bd.x;
	dropShape *= DeltaSawTooth(t);
	bd.y += dropShape;								// change shape of drop when it is falling
	#endif
	
	float d = length(bd);							// distance to main drop
	
	float trailMask = S(-.2, .2, bd.y);				// mask out drops that are below the main
	trailMask *= bd.y;								// fade dropsize
	float td = length(trailPos*max(.5, trailMask));	// distance to trail drops
	
	float mainDrop = S(.2, .1, d);
	float dropTrail = S(.1, .02, td);
	
	dropTrail *= trailMask;
	o = mix(bd*mainDrop, trailPos, dropTrail);		// mix main drop and drop trail
	
	#ifdef DROP_DEBUG
	if(uv2.x<.02 || uv2.y<.01) o = vec2(1.);
	#endif
	
	return o;
}
//END RAIN STUFF

//////////////////////////////////////////////////////////////////////////////////////// SKY

void main(){
	// water height is 0, checked with camPos.y just in case smn wants to use a different height
	
	vec3 geoWorldPos = texture(texPosition, TexCoords).xyz - vec3(0.0,mapHeight,0.0);
	vec4 col = texture(texColor, TexCoords);
	vec3 camPos = Camera.Position.xyz - vec3(0.0,mapHeight,0.0);
	
	// dont execute shader on above ocean terrain pixels
	if (geoWorldPos.y > 1.0 && camPos.y > 0.0) {
		FragColor = col;
		return;
	}
	
	vec2 uv = TexCoords;
	uv.x *= aspectRatio;
	uv.x -= (aspectRatio-1.0)*0.5;
	vec2 ndc = uv * 2.0 - vec2(1.0);
	
	vec3 camDir = (inverse(Camera.ViewMatrix)*vec4(0.0,0.0,-1.0,0.0)).xyz;
	vec3 camUp = (inverse(Camera.ViewMatrix)*vec4(0.0,1.0,0.0,0.0)).xyz;
	
	vec3 camSide = cross(camDir, camUp);
	float focus = Camera.ProjectionMatrix[1][1];

	vec3 rayDir = normalize(camSide*ndc.x + camUp*ndc.y + camDir*focus);
	
	float sDot = abs(dot(sunDir,vec3(0.0,1.0,0.0)));
	SEA_BASE *= sDot;
	
	//FragColor = raymarch(rayDir, camPos);
	
	float distScene = length(geoWorldPos - camPos);
	
	vec3 ori = camPos;
	vec3 dir = rayDir;
	// tracing
	vec3 p;
	float waterDist = heightMapTracing(ori,sign(camPos.y)*dir,p);
	vec3 dist = p - ori;
	vec3 n = getNormal(p, dot(dist,dist) * EPSILON_NRM);
	vec3 light = sunDir; //normalize(vec3(-.5,1.0,0.5));
	light.y *= sign(camPos.y);
	
	/////////////////////////////////////////////////////////////////////////// OCEAN Variables
	
	
	
	/////////////////////////////////////////////////////////////////////////// 
	fov = Camera.ProjectionMatrix[1][1];
	float M = 1.0;
	Ds = sunDir; //normalize(vec3(uvMouse.x-((0.5*AR)), uvMouse.y-0.5, (fov/-2.0)));
	
	
	vec3 O = vec3(0., camPos.y, 0.);
	vec3 D = dir;//normalize(vec3(uv, -(fov*M)));

	vec3 color = vec3(0.);
	vec3 scat = vec3(0.);

	//float scat = 0.;
	float att = 1.;
	float staratt = 1.;
	float scatatt = 1.;
	vec3 star = vec3(0.);
	vec4 aur = vec4(0.);

	float fade = smoothstep(0.,0.01,abs(D.y))*0.5+0.9;
	
	staratt = 1. -min(1.0,((dot(sunDir, vec3(0.0,1.0,0.0))+1.0)*0.5*2.0));
	scatatt = 1. -min(1.0,((dot(sunDir, vec3(0.0,1.0,0.0))+1.0)*0.5*2.2));

	float ts = 1.5;
	if (waterDist < 5000.0 && camPos.y > 0.0) { // reflection
		float L = - O.y / D.y;
		O = O + D * L;
		D.y = -D.y;
		//D = normalize(D+vec3(0,.003*sin(t+6.2831*noise(O.xz+vec2(0.,-t*1e3))),0.));
		att = .6;
		D = 0.85*D + 0.15*n;
		star = stars(D);
		aur = dot(sunDir, vec3(0.0,1.0,0.0)) < 0.0 ? smoothstep(0.0,2.5,aurora(O,D)) : aur;
	}
	else{ // night sky
		float L1 =  O.y / D.y;
		vec3 O1 = O + D * L1;

		vec3 D1 = vec3(1.);
		D1 = normalize(D+vec3(1.,0.0009*sin(t+6.2831*noise(O1.xz+vec2(0.,t*0.8))),0.));
		star = stars(D1);
		aur = dot(sunDir, vec3(0.0,1.0,0.0)) < 0.0 ? smoothstep(0.,1.5,aurora(O,D))*fade : aur;
	}

	star *= att;
	star *= staratt;

	scatter(O, D, color, scat, t);
	color *= att;
	scat *=  att;
	scat *= scatatt;
	
	color += scat;
	color += star;
	color += aur.rgb*scatatt;
	
	
	#if ENABLE_RAIN
	vec2 drops = vec2(0.);
	if (rainmulti > 1.0){
	drops = GetDrops(uv/2.0, 1., 1.);

	color +=drops.x+drops.y;
	}
	#endif

	vec4 sscSky = vec4(pow(color, vec3(1.0/2.2)), 1.); //gamma correct
	///////////////////////////////////////////////////////////////////////////
	
	/////////////////////////////////////////////////////////////////////////// OCEAN
	
	lightColor = lightColor*sDot + (1.0-sDot)*vec3(1.0,0.584,0.0);
	// color
	vec3 oceanCol = mix(sscSky.xyz, getSeaColor(sign(camPos.y)*p,sign(camPos.y)*n,sign(camPos.y)*light,sign(camPos.y)*dir,dist), pow(smoothstep(0.0,-0.02,sign(camPos.y)*dir.y),0.2));
	float weight = 1.0/(waterDist);
	float distDiff = clamp(0.2*log(((1.0-weight)*(-geoWorldPos.y/(mapHeight*0.5))+weight*(distScene - waterDist)/mapHeight))+1.0,0.0,1.0);
	if (camPos.y > 0.0) {
		// blending depending on water depth
	
		oceanCol = oceanCol*distDiff + col.xyz*(1.0-distDiff);
		
		// fake SSR
		if (distDiff > 0.0 && camPos.y > 0.0 && waterDist < 5000.0) {
			// interpolate plane normal with ocean normal
			float u = abs(dot(camDir,vec3(0.0,1.0,0.0)));
			float quadinterp = pow(u-0.5,2)*2.0+0.5;
			vec3 normal = (quadinterp)*vec3(0.0,1.0,0.0) + (1.0-quadinterp)*n;

			//normal = vec3(0.0,1.0,0.0); // enable to view actual reflection
		
			// angle of reflection
			float ref = dot(normal,camDir);
			ref *= 1.0+(pow(abs(ref),4))*1.5; // correct view angle
		
			vec2 coord = vec2(TexCoords.x, 1.0-(ref*focus+TexCoords.y));
			vec3 scnRef = vec3(0.0);
			if (coord.x > 0.0 && coord.x < 1.0 && coord.y > 0.0 && coord.y < 1.0 && coord.y > TexCoords.y) // only sample reflection above water
			{
				scnRef = texture(texColor, coord).xyz;
			}

			vec3 skyRef = sscSky.xyz;
			skyRef = length(scnRef) == 0.0 ? skyRef : max(scnRef,0.1);// + distDiffBlend*max(scnRef,0.1);
			//oceanCol = skyRef;
			oceanCol = oceanCol + skyRef*0.5*sDot;
		}
	} else { // underwater
		vec2 coord = refract(dir,n,0.680678).xy*0.5;
		weight = max(0.0,1.0/(1.0-waterDist*0.01));
		
		vec2 refractNoise = clamp(coord*(0.035+0.005*(1.0-weight))+TexCoords*(0.965-0.005*(weight)),0.0,1.0);
		
		float time = uTime*0.1; //cos(uTime*0.5)*10.0+20.0;
		vec2 waterNoise = TexCoords + vec2(sin(uTime)*noise(TexCoords*5.0+time)*0.01,cos(uTime)*noise(TexCoords.yx*5.0+time)*0.01)*(distDiff);
		refractNoise = clamp(refractNoise,0.0,1.0);
		waterNoise  = clamp(waterNoise,0.0,1.0);
		vec3 scnCol = abs(waterDist) < abs(distScene) ? texture(texColor, refractNoise).xyz : texture(texColor, waterNoise).xyz;
		scnCol = length(scnCol) == 0.0 ? sscSky.xyz : scnCol;
		
		//distDiff = distDiff + max(0.0,(1.0-weight)-distDiff);
		
		oceanCol = abs(waterDist) < abs(distScene)
		           ? oceanCol*(1.0-weight)+(weight)*scnCol
		           : scnCol*(1.0-distDiff)+(distDiff)*(SEA_BASE);
	}

	FragColor = vec4(oceanCol,0.0);
}
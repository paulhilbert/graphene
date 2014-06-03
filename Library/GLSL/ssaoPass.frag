#version 330

const int MAX_KERNEL_SIZE = 128;

uniform float radius = 1.5;
uniform float power = 2.0;

uniform int numSamples = 12;
uniform vec3 samples[MAX_KERNEL_SIZE];
uniform float noise[4*4*2];


uniform sampler2D mapPos;
uniform sampler2D mapNrm;
//uniform sampler2D mapDepth;

uniform mat4 mvM;
uniform mat4 prM;
uniform mat3 nmM;


in VertexData {
	smooth vec2 tc;
	noperspective vec3 viewRay;
} vertexIn;

out vec4 light;


float linearize(in float depth);
vec3 viewPos(in vec2 tc);
float ssao(in mat3 local, in vec3 pos, in float r);

void main (void) {
	vec3 pos = texture(mapPos, vertexIn.tc).xyz;
	vec3 nrm = texture(mapNrm, vertexIn.tc).xyz;
	if (dot(nrm,nrm) < 0.8) {
		light = vec4(1.0, 1.0, 1.0, 1.0);
		return;
	}
	ivec2 idx = ivec2(vertexIn.tc * textureSize(mapPos,0)) % 4;
	vec3 n = vec3(noise[idx.x*4*2 + idx.y*2 + 0], noise[idx.x*4*2 + idx.y*2 + 1], 0.f);
	vec3 tangent = normalize(n - nrm * dot(n,nrm));
	vec3 bitangent = cross(nrm, tangent);
	mat3 local = mat3(tangent, bitangent, nrm);

	float lightFactor = ssao(local, pos, radius);
	light = vec4(lightFactor, lightFactor, lightFactor, 1.0);
}

float linearize(in float depth) {
	return prM[3][2] / (depth - prM[2][2]);
}

vec3 viewPos(vec2 tc) {
	vec3 pos = texture(mapPos, tc).xyz;
	vec4 p = prM * mvM * vec4(pos, 1.0);
	p /= p.w;
	return p.xyz;
}

float ssao(in mat3 local, in vec3 pos, in float r) {
	vec3 vPos = viewPos(vertexIn.tc);
	float occlusion = 0.f;
	vec3 sample;
	for (int i=0; i<numSamples; ++i) {
		sample = r*(local * samples[i]) + pos;

		vec4 s = prM * mvM * vec4(sample, 1.0);
		s /= s.w;
		s.xy = s.xy * 0.5 + 0.5;
		vec3 vSample = viewPos(s.xy);

		vec3 sPos = texture(mapPos, s.xy).xyz;

		float rangeCheck = length(sPos - pos) > r ? 0.0 : 1.0;
		occlusion += rangeCheck * (vSample.z < s.z ? 1.0 : 0.0);
	}
	occlusion = 1.0 - (occlusion / float(numSamples));
	return pow(occlusion, power);
}

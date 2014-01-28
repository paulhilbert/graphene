#version 330

uniform float samples[12*3];
uniform float noise[4*4*2];
uniform int width;
uniform int height;

uniform sampler2D mapPos;
uniform sampler2D mapNrm;
uniform sampler2D mapDepth;

uniform mat4 mvM;
uniform mat4 prM;
uniform mat3 nmM;

in vec2 tc;

out vec4 light;

void main (void) {
	vec3 pos = texture(mapPos, tc).xyz;
	pos = (mvM * vec4(pos.xyz, 1.0)).xyz;
	vec3 nrm = nmM * texture(mapNrm, tc).xyz;

	int ni = int(tc.x * width) % 4;
	int nj = int(tc.y * height) % 4;
	vec3 n = vec3(noise[ni*4*2 + nj*2 + 0], noise[ni*4*2 + nj*2 + 1], 0.f);
	vec3 tangent = normalize(n - nrm * dot(n,nrm));
	vec3 bitangent = cross(nrm, tangent);
	mat3 local = mat3(tangent, bitangent, nrm);

	float occlusion = 0.f;
	for (int i=0; i<12; ++i) {
		vec3 sample = local * vec3(samples[i*3+0], samples[i*3+1], samples[i*3+2]);
		sample = sample * 0.5f + pos;
		vec4 origin = prM * vec4(pos, 1.f);
		origin.xy /= origin.w;
		origin.xy = origin.xy * 0.5 + 0.5;

		vec4 offset = prM * vec4(sample.xyz, 1.f);
		offset.xy /= offset.w;
		offset.xy = offset.xy * 0.5 + 0.5;
		float depth = texture(mapDepth, offset.xy).r;
		float odepth = texture(mapDepth, origin.xy).r;
		depth = prM[3][2] / (depth - prM[2][2]);
		odepth = prM[3][2] / (odepth - prM[2][2]);
		float rangeCheck = abs(odepth - depth) < 0.5f ? 1.0 : 0.0;
		occlusion += (depth <= odepth ? 1.0 : 0.0) * rangeCheck;
	}
	light = vec4(1.0 - (occlusion / 12.0));
}

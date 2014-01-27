#version 140

uniform float offsets[9];
uniform float weights[9];

uniform sampler2D mapBlur;
uniform sampler2D mapBloom;

in vec2 tc;

out vec4 blur;
out vec4 bloom;

void main (void) {
	int i;
	blur = vec4(0.0, 0.0, 0.0, 1.0);
	bloom = vec4(0.0, 0.0, 0.0, 1.0);

	for(i=0; i<9; i++) {
		vec2 coord = tc + vec2(0.0, offsets[i]);
		if (coord.x < 0.f || coord.y < 0.f || coord.x >= 1.f || coord.y >= 1.f) continue;
		blur += texture2D(mapBlur, coord) * weights[i];
		bloom += texture2D(mapBloom, coord) * weights[i];
	}
}

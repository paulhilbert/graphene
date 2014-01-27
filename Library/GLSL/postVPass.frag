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
		blur += texture2D(mapBlur, tc + vec2(0.0, offsets[i])) * weights[i];
		bloom += texture2D(mapBloom, tc + vec2(0.0, offsets[i])) * weights[i];
	}
}

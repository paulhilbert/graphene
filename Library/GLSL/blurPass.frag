#version 140

uniform float weights[9*9];
uniform float offsetsH[9];
uniform float offsetsV[9];

uniform sampler2D mapCol;
uniform sampler2D mapOcclusion;

in vec2 tc;

out vec4 blur;
out vec4 bloom;
out vec4 ssao;

void main (void) {
	int i, j;
	blur = vec4(0.0, 0.0, 0.0, 1.0);
	bloom = vec4(0.0, 0.0, 0.0, 1.0);
	ssao = vec4(0.0, 0.0, 0.0, 1.0);

	for(i=0; i<9; i++) {
		for(j=0; j<9; j++) {
			vec2 coord = tc + vec2(offsetsH[i], offsetsV[j]);
			if (coord.x < 0.f || coord.y < 0.f || coord.x >= 1.f || coord.y >= 1.f) continue;
			vec4 col = texture2D(mapCol, coord);
			blur += col * weights[i*9+j];
			bloom += (col.r > 1.0 || col.g > 1.0 || col.b > 1.0) ? col*weights[i*9+j] : vec4(0.0);
			ssao += texture2D(mapOcclusion, coord) * weights[i*9+j];
		}
	}
}

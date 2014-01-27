#version 330

const float MIDDLE_GREY = 0.72;
const float FUDGE = 0.001;
const float L_WHITE = 1.5;

uniform float exposure;

uniform sampler2D mapCol;

in vec2 tc;
out vec4 fragColor;


vec4 toneMap(in vec4 col, in float exposure);

void main(void) {
	vec4 diffCol = texture2D(mapCol, tc);

	fragColor = toneMap(diffCol, exposure);
}

vec4 toneMap(in vec4 col, in float exposure) {
	col.rgb *= (MIDDLE_GREY / (FUDGE + exposure));
	col.rgb *= (1.0 + col.rgb / L_WHITE);
	col.rgb /= (1.0 + col.rgb);

	return col;
}

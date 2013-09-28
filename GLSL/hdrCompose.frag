#version 140

const float MIDDLE_GREY = 0.72;
const float FUDGE = 0.001;
const float L_WHITE = 1.5;

uniform float factor;
uniform float exposure;

uniform sampler2D Tex0;
uniform sampler2D Tex1;

in vec2 tc;
out vec4 fragColor;

vec4 toneMap(in vec4 col, in float exposure);

void main(void) {
	vec4 colorMap = texture2D(Tex0, tc);
	vec4 bloomMap = texture2D(Tex1, tc);

	//fragColor = toneMap(colorMap + factor*abs(bloomMap), exposure);//colorMap + factor * (bloomMap - colorMap);
	fragColor = toneMap(colorMap + factor*(bloomMap-colorMap), exposure);//colorMap + factor * (bloomMap - colorMap);
}

vec4 toneMap(in vec4 col, in float exposure) {
	col.rgb *= (MIDDLE_GREY / (FUDGE + exposure));
	col.rgb *= (1.0 + col.rgb / L_WHITE);
	col.rgb /= (1.0 + col.rgb);

	return col;
}

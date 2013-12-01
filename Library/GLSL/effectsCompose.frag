#version 140

const float MIDDLE_GREY = 0.72;
const float FUDGE = 0.001;
const float L_WHITE = 1.5;

uniform int   ortho;
uniform float ratio;
uniform float focalPoint;
uniform float focalArea;
uniform float near;
uniform float far;
uniform float exposure;
uniform int   tonemap;

uniform sampler2D Tex0;
uniform sampler2D Tex1;
uniform sampler2D Tex2;

in vec2 tc;
out vec4 fragColor;

vec4 toneMap(in vec4 col, in float exposure);

void main(void) {
	vec4 colorMap = texture2D(Tex0, tc);
	vec4 blurMap = texture2D(Tex1, tc);
	float depth = texture2D(Tex2, tc).r;
	float z;
	if (ortho == 0) {
		float r = far - near;
		z = near * far / ((depth * r) - far);
		z = (z+near) / (-r);
	} else {
		z = depth;
	}

	//fragColor = toneMap(colorMap + factor*(bloomMap-colorMap), exposure);//colorMap + factor * (bloomMap - colorMap);
	float blur = abs(z - focalPoint) / focalArea;
	blur = ratio * clamp(blur, 0.f, 1.f);
	vec4 hdrColor = (1.f-blur) * colorMap + blur * blurMap;
	if (tonemap != 0) fragColor = toneMap(hdrColor, exposure);
}

vec4 toneMap(in vec4 col, in float exposure) {
	col.rgb *= (MIDDLE_GREY / (FUDGE + exposure));
	col.rgb *= (1.0 + col.rgb / L_WHITE);
	col.rgb /= (1.0 + col.rgb);

	return col;
}

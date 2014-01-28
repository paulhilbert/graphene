#version 330

const float MIDDLE_GREY = 0.72;
const float FUDGE = 0.001;
const float L_WHITE = 1.5;

uniform int   ortho;
uniform float ratio;
uniform float bloom;
uniform float focalPoint;
uniform float focalArea;
uniform float near;
uniform float far;
uniform float exposure;
uniform int   debugSSAO = 0;

uniform sampler2D mapCol;
uniform sampler2D mapDepth;
uniform sampler2D mapBlur;
uniform sampler2D mapBloom;
uniform sampler2D mapSSAO;

in VertexData {
	smooth vec2 tc;
	noperspective vec3 viewRay;
} vertexIn;
out vec4 fragColor;


vec4 toneMap(in vec4 col, in float exposure);

void main(void) {
	vec4 diffCol = texture2D(mapCol, vertexIn.tc);
	vec4 blurCol = texture2D(mapBlur, vertexIn.tc);
	vec4 bloomCol = texture2D(mapBloom, vertexIn.tc);
	float depth  = texture2D(mapDepth, vertexIn.tc).r;
	float light = texture2D(mapSSAO, vertexIn.tc).r;

	float z;
	if (ortho == 0) {
		float r = far - near;
		z = near * far / ((depth * r) - far);
		z = (z+near) / (-r);
	} else {
		z = depth;
	}
	float blur = abs(z - focalPoint) / focalArea;
	blur = ratio * clamp(blur, 0.f, 1.f);

	fragColor = toneMap((1.f - blur) * light * diffCol + light * blur * blurCol + light * blur * bloom * (bloomCol * blurCol), exposure);
	if (debugSSAO != 0) fragColor = vec4(light, light, light, 1.0);
}

vec4 toneMap(in vec4 col, in float exposure) {
	col.rgb *= (MIDDLE_GREY / (FUDGE + exposure));
	col.rgb *= (1.0 + col.rgb / L_WHITE);
	col.rgb /= (1.0 + col.rgb);

	return col;
}

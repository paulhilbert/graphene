#version 330

in vec3 nrm;
in vec3 refl;
in vec4 col;
out vec4 fragmentColor;

uniform sampler2D Tex0;
uniform sampler2D Tex1;
uniform float specularity;
uniform float alpha;
uniform float exposure;
uniform int tonemap;

const float pi = 3.14159265358979;
const float piInv = 0.318309886183791;

const vec4 ambient = vec4(0.2, 0.2, 0.2, 1.0);
const float MIDDLE_GREY = 0.72;
const float FUDGE = 0.001;
const float L_WHITE = 1.5;

vec2 dirToUV(in vec3 dir);
vec4 toneMap(in vec4 col, in float exposure);

void main() {
	vec4 diff = texture2D(Tex0, dirToUV(nrm));
	vec4 spec = texture2D(Tex1, dirToUV(refl));

	fragmentColor = (ambient+diff)*col + specularity*spec;//toneMap((ambient+diff)*material + specularity*spec, exposure);
	if (tonemap != 0) {
		fragmentColor = toneMap(fragmentColor, exposure);
	}
	fragmentColor[3] = alpha;
}


vec2 dirToUV(in vec3 dir) {
	if (dot(dir, vec3(0,0,1)) >= 0.999) return vec2(1.0, 0.0);
	if (dot(dir, vec3(0,0,1)) <= -0.999) return vec2(1.0, 1.0);
	return clamp(vec2(0.5f*(1.f + atan(dir.x,-dir.y) / pi), acos(dir.z) / pi), 0.f, 1.f);
}

vec4 toneMap(in vec4 col, in float exposure) {
	col.rgb *= (MIDDLE_GREY / (FUDGE + exposure));
	col.rgb *= (1.0 + col.rgb / L_WHITE);
	col.rgb /= (1.0 + col.rgb);

	return col;
}

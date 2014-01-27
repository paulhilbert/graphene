#version 330

const float MIDDLE_GREY = 0.72;
const float FUDGE = 0.001;
const float L_WHITE = 1.5;

uniform float exposure;
uniform float specularity;
uniform float refrIndex;

uniform vec3      camPos;
uniform sampler2D mapPos;
uniform sampler2D mapCol;
uniform sampler2D mapNrm;
uniform sampler2D mapDiff;
uniform sampler2D mapSpec;

in vec2 tc;
out vec4 fragColor;

const float pi = 3.14159265358979;
const float piInv = 0.318309886183791;


vec2 dirToUV(in vec3 dir);
vec4 toneMap(in vec4 col, in float exposure);

void main(void) {
	vec3 pos = texture2D(mapPos, tc).xyz;
	vec4 diffCol = texture2D(mapCol, tc);
	vec3 nrm = texture2D(mapNrm, tc).xyz;

	if (dot(nrm, nrm) < 0.9f) {
		fragColor = diffCol;
		return;
	}

	vec3 viewDir = normalize(pos - camPos);

	vec3 refl = normalize(reflect(viewDir, nrm));
	vec4 diff = texture2D(mapDiff, dirToUV(nrm));
	vec4 ambient = 0.2 * diff + 0.2 * diffCol;
	vec4 spec = texture2D(mapSpec, dirToUV(refl));

	float R = (1.f - refrIndex) / (refrIndex + 1.f);
	R = R * R;
	float ds = R + (1.f - R)*pow(1.f - dot(nrm, -viewDir), 5);

	vec4 fragmentColor;
	fragmentColor = (ambient + diff) * diffCol + specularity * ds * spec;
	fragmentColor.a = 1.f;

	fragColor = toneMap(fragmentColor, exposure);
	//fragColor.xyz = pos;
}

vec2 dirToUV(in vec3 dir) {
	//if (dot(dir, vec3(0,0,1)) >= 0.999) return vec2(1.0, 0.0);
	//if (dot(dir, vec3(0,0,1)) <= -0.999) return vec2(1.0, 1.0);
	return clamp(vec2(0.5f*(1.f + atan(dir.x,-dir.y) / pi), acos(dir.z) / pi), 0.f, 1.f);
}

vec4 toneMap(in vec4 col, in float exposure) {
	col.rgb *= (MIDDLE_GREY / (FUDGE + exposure));
	col.rgb *= (1.0 + col.rgb / L_WHITE);
	col.rgb /= (1.0 + col.rgb);

	return col;
}

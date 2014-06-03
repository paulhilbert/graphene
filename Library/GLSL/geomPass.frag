#version 330

uniform float specularity = 0.0;
uniform float refrIndex = 1.3;

uniform int debugNormals = 0;

uniform vec3      camPos;
uniform sampler2D mapDiff;
uniform sampler2D mapSpec;

const float pi = 3.14159265358979;
const float piInv = 0.318309886183791;

in vec3 fsPos;
in vec4 fsCol;
in vec3 fsNrm;

out vec3 outPos;
out vec4 outCol;
out vec3 outNrm;

vec2 dirToUV(in vec3 dir);

void main(void) {
	outPos = fsPos;

	if (debugNormals != 0) {
		if (isinf(fsNrm.x) || isnan(fsNrm.x) || isinf(fsNrm.y) || isnan(fsNrm.y) || isinf(fsNrm.z) || isnan(fsNrm.z)) {
			outCol = vec4(1.0, 1.0, 1.0, 1.0);
		} else if (dot(fsNrm, fsNrm) < 0.1f) {
			outCol = vec4(0.0, 0.0, 0.0, 1.0);
		} else {
			outCol = vec4(abs(normalize(fsNrm)), 1.0);
		}
		return;
	}

	if (dot(fsNrm, fsNrm) < 0.1f) {
		outCol = fsCol;
		outNrm = vec3(0.f, 0.f, 0.f);
		return;
	}
	outNrm = normalize(fsNrm);


	vec3 viewDir = normalize(fsPos - camPos);
	vec3 refl = normalize(reflect(viewDir, outNrm));
	vec4 diff = texture2D(mapDiff, dirToUV(outNrm));
	vec4 ambient = 0.2 * diff + 0.2 * fsCol;
	vec4 spec = texture2D(mapSpec, dirToUV(refl));

	float R = (1.f - refrIndex) / (refrIndex + 1.f);
	R = R * R;
	float ds = (1.0 - abs(dot(outNrm, -viewDir))) * (R + (1.f - R)*pow(1.f - dot(outNrm, -viewDir), 5));

	float alpha = fsCol.a;
	outCol = (ambient + diff) * fsCol + specularity * ds * spec;
	outCol.a = 1.0; //alpha;
}

vec2 dirToUV(in vec3 dir) {
	if (dot(dir, vec3(0,0,1)) >=  0.9999999) return vec2(1.0, 0.0);
	if (dot(dir, vec3(0,0,1)) <= -0.9999999) return vec2(1.0, 1.0);
	return clamp(vec2(0.5f*(1.f + atan(dir.x,-dir.y) / pi), acos(dir.z) / pi), 0.f, 1.f);
}


#version 140

in vec3 nrm;
in vec3 refl;
in vec4 fColor;
out vec4 fragmentColor;

uniform sampler2D Tex0;
uniform sampler2D Tex1;
uniform float specularity;

const float pi = 3.14159265358979;
const float piInv = 0.318309886183791;

const vec4 ambient = vec4(0.2, 0.2, 0.2, 1.0);

vec2 dirToUV(in vec3 dir);

void main() {
	vec4 diff = texture2D(Tex0, dirToUV(nrm));
	vec4 spec = texture2D(Tex1, dirToUV(refl));

	float alpha = fColor.a;
	fragmentColor = (ambient+diff)*fColor + specularity*spec;
	fragmentColor.a = alpha;
}

vec2 dirToUV(in vec3 dir) {
	if (dot(dir, vec3(0,0,1)) >= 0.999) return vec2(1.0, 0.0);
	if (dot(dir, vec3(0,0,1)) <= -0.999) return vec2(1.0, 1.0);
	return clamp(vec2(0.5f*(1.f + atan(dir.x,-dir.y) / pi), acos(dir.z) / pi), 0.f, 1.f);
}

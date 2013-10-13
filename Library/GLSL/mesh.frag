#version 140

in vec3 mvsNormal;
in vec3 lightNormal;
out vec4 fragmentColor;

uniform vec4 ambient;
uniform vec4 diffuse;

void main() {
	vec3 normalized_normal = normalize(mvsNormal);
	vec3 normalized_lightDir = normalize(lightNormal);

	float att = clamp(dot(mvsNormal, normalized_lightDir), 0.0, 1.0);

	fragmentColor = ambient + diffuse * att;
}

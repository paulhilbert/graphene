#version 120

uniform vec4 ambient;
uniform vec4 diffuse;

varying vec3 mvsNormal;
varying vec3 lightNormal;

void main() {
	vec3 normalized_normal = normalize(mvsNormal);
	vec3 normalized_lightDir = normalize(lightNormal);

	float att = clamp(dot(normalized_normal, normalized_lightDir), 0.0, 1.0);

	gl_FragColor = ambient + diffuse * att;
}

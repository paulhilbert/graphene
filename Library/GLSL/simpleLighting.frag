#version 140

in vec3 mvsNormal;
in vec3 lightNormal;
in vec4 fColor;
out vec4 fragmentColor;


void main() {
	vec3 normalized_normal = normalize(mvsNormal);
	vec3 normalized_lightDir = normalize(lightNormal);

	float att = clamp(dot(normalized_normal, normalized_lightDir), 0.0, 1.0);

	vec4 diffuse = fColor;
	vec4 ambient = 0.2*fColor;
	fragmentColor = ambient + diffuse * att;
}

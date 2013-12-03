#version 140

in vec3 position;
in vec3 normal;
in vec4 color;

uniform mat4 mvM;
uniform mat4 prM;
uniform vec3 viewDir;

out vec3 nrm;
out vec3 refl;
out vec4 fColor;


void main() {
	gl_Position  = prM * mvM * vec4(position.xyz, 1.0);
	nrm          = normal;
	refl         = normalize(reflect(viewDir, normal));
	fColor       = color;
}

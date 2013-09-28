#version 330

in vec3 position;
in vec3 normal;
in vec4 color;

uniform mat4 mvM;
uniform mat4 prM;
uniform vec3 viewDir;
uniform vec3 clipNormal;
uniform float clipDistance;

out vec3 nrm;
out vec3 refl;
out vec4 col;


void main() {
	gl_Position  = prM * mvM * vec4(position.xyz, 1.0);
	gl_ClipDistance[0] = -dot(position.xyz, clipNormal) + clipDistance;
	nrm = normal;
	refl = normalize(reflect(viewDir,normal));
	col = color;
}

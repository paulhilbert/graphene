#version 330

uniform mat4 mvM;
uniform mat4 prM;

uniform vec3 clipNormal;
uniform float clipDistance;

in vec3 position;
in vec3 normal;
in vec4 color;

out vec3 fsPos;
out vec4 fsCol;
out vec3 fsNrm;


void main(void) {
	gl_Position = prM * mvM * vec4(position.xyz, 1.0);
	fsPos = position;
	fsCol = color;
	fsNrm = normal;

	gl_ClipDistance[0] = -dot(position.xyz, clipNormal) + clipDistance;
}

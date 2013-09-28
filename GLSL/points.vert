#version 330

in vec3 position;
in vec4 icolor;

out vec4 color;

uniform mat4 mvM;
uniform mat4 prM;


void main() {
	gl_Position  = prM * mvM * vec4(position.xyz, 1.0);
	color = icolor;
}

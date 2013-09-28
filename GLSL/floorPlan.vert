#version 140

uniform mat4 mvM;
uniform mat4 prM;
uniform mat4 planeTransformM;

in vec3 position;
in vec2 texcoord;

out vec2 tc;

void main() {
	gl_Position = prM * mvM * planeTransformM * vec4(position, 1.0);
	tc = texcoord;
}

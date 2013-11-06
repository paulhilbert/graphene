#version 140

in vec3 position;
in vec3 normal;
in vec4 color;

uniform mat4 mvM;
uniform mat4 prM;
uniform mat3 nmM;
uniform vec3 lightDir;

out vec3 mvsNormal;
out vec3 lightNormal;
out vec4 fColor;


void main() {
	gl_Position  = prM * mvM * vec4(position.xyz, 1.0);
	mvsNormal    = nmM * normal;
	lightNormal  = nmM * lightDir;
	fColor       = color;
}

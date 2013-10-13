#version 120

uniform vec3 lightDir;
uniform mat4 mvM;
uniform mat4 prM;
uniform mat3 nmM;

varying vec3 mvsNormal;
varying vec3 lightNormal;

void main() {
	gl_Position  = prM * mvM * gl_Vertex;
	mvsNormal    = nmM * gl_Normal;
	lightNormal  = nmM * lightDir;
}

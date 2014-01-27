#version 330

in vec3 position;

uniform vec4 clearColor;

out vec3 outPos;
out vec4 outCol;
out vec3 outNrm;

void main(void) {
	outPos = vec3(0.f, 0.f, 0.f);
	outCol = clearColor;
	outNrm = vec3(0.f, 0.f, 0.f);
}

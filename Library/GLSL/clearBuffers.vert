#version 330

in vec3 position;
out vec2 tc;

void main(void) {
	gl_Position  = vec4(position.xyz, 1.0);
	tc = clamp(0.5f * (position.xy + 1.f), 0.f, 1.f);
}

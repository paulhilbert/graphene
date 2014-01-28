#version 330

in vec3 position;
uniform float aspectRatio;
uniform float tanHalfFov;

out VertexData {
	smooth vec2 tc;
	noperspective vec3 viewRay;
} vertexOut;

void main(void) {
	gl_Position  = vec4(position.xyz, 1.0);
	vertexOut.tc = clamp(0.5f * (position.xy + 1.f), 0.f, 1.f);

	vertexOut.viewRay = vec3(
		position.x * tanHalfFov * aspectRatio,
		position.y * tanHalfFov,
		1.0
	);
}

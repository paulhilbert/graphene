#version 330

in vec3 position;
//in vec2 texCoord;
out vec2 tc;

//uniform mat4 mvM;
//uniform mat4 prM;

void main(void) {
	//gl_Position  = prM * mvM * vec4(position.xyz, 1.0);
	gl_Position  = vec4(position.xyz, 1.0);
	tc = clamp(0.5f * (position.xy + 1.f), 0.f, 1.f);
}

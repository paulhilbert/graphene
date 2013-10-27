#version 140

uniform float offsets[9];
uniform float weights[9];
uniform sampler2D Tex0;

in vec2 tc;
out vec4 fragColor;

void main (void)
{
	int i;
	vec4 color = vec4(0.0, 0.0, 0.0, 1.0);

	for(i=0; i<9; i++) {
		color += (texture2D(Tex0, tc + vec2(0.0, offsets[i])) * weights[i]);
	}

	fragColor = color;
}

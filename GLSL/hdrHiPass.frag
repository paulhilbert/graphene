#version 140

uniform sampler2D Tex0;

in vec2 tc;
out vec4 fragColor;

void main(void)
{
	vec4 colorMap = texture2D(Tex0, tc);

//	float Y = dot(vec4(0.30, 0.59, 0.11, 0.0), colorMap);
//	if (Y > 1.0) {
	if (colorMap.r > 1.0 || colorMap.g > 1.0 || colorMap.b > 1.0) {
		fragColor = colorMap;
	}
	else {
		fragColor = vec4(0.0);
	}
}


#version 330 compatibility
layout(lines) in;
layout(line_strip, max_vertices=6) out;

// pass through color
in  vec4 vcolor[];
out vec4 fcolor;

void main() {
	float angleFactor = 0.4;
	float lengthFactor = 0.2;
	vec4 unicol = vcolor[0]; // color of base vertex is used for all vertices
	vec4 verts[6];
	verts[0] = gl_in[0].gl_Position;
	verts[3] = gl_in[1].gl_Position;

	// determine hook end points
	angleFactor = 0.4;
	vec4 v = gl_in[1].gl_Position - gl_in[0].gl_Position;
	vec4 p = gl_in[0].gl_Position + (1.0 - lengthFactor) * v;
	vec2 v2d = v.xy;
	v2d = normalize(v2d);
	vec2 v1 = vec2(-v2d.y, v2d.x);
	vec2 v2 = vec2(v2d.y, -v2d.x);
	float arrowLength = length(v.xy) * lengthFactor;

	verts[1] = p;
	verts[2] = p;
	verts[4] = p;
	verts[5] = p;
	verts[2].xy += angleFactor * arrowLength * v1;
	verts[4].xy += angleFactor * arrowLength * v2;

	// generate
	for (int i=0; i < 6; ++i) {
		gl_Position = verts[i];
		fcolor = unicol;
		EmitVertex();
	}
	EndPrimitive();
	/*
	for (int i=0; i<3; ++i) {
		gl_Position = gl_in[1].gl_Position;
		fcolor = unicol;
		EmitVertex();

		gl_Position = ends[i];
		fcolor = unicol;
		EmitVertex();

		EndPrimitive();
	}
	*/
}  

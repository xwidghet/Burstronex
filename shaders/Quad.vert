#version 460 core
layout (location = 0) in vec3 Pos;
out vec2 QuadCoordinate;

void main()
{
	// OpenGL rendering is upside-down :))
	QuadCoordinate = vec2(Pos.x, 1.0 - Pos.y);

	// Full-screen quad.
	vec2 DrawPosition = (Pos.xy*2.0) - 1.0;
	gl_Position = vec4(DrawPosition.x, DrawPosition.y, Pos.z, 1.0f);
}

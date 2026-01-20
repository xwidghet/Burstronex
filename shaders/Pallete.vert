#version 460 core
layout (location = 0) in vec3 Pos;
out vec2 QuadCoordinate;

void main()
{
	vec3 DebugPos = vec3(Pos.x*2 - 1, Pos.y, Pos.z);
	// OpenGL rendering is upside-down :))
	QuadCoordinate = vec2(Pos.x, 1.0 - Pos.y);
	gl_Position = vec4(DebugPos.x, DebugPos.y, DebugPos.z, 1.0f);
}

#pragma once

#include "glad/glad.h"

#include <string>
#include <vector>

class Shader {
	unsigned int mShaderProgram;

public:
	Shader(const std::string& VertexPath, const std::string& FragmentPath);

	void Use();

private:
	void CreateAndLinkProgram(unsigned int VertexProgram, unsigned int FragmentProgram);

	void CompileShader(const std::string& PathToShader, GLenum Type, unsigned int& ShaderStorage);

	std::vector<char> ReadShader(const std::string& PathToShader);
};
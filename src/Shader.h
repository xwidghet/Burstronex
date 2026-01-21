#pragma once

#include "glad/glad.h"

#include <string>
#include <vector>

class Shader {
	unsigned int mShaderProgram;
	std::string mVertexPath;
	std::string mFragmentPath;

public:
	Shader(const std::string& VertexPath, const std::string& FragmentPath);

	void Use();

	void Reload();

private:
	void CreateAndLinkProgram();

	void CompileShader(const std::string& PathToShader, GLenum Type, unsigned int& ShaderStorage);

	std::vector<char> ReadShader(const std::string& PathToShader);
};

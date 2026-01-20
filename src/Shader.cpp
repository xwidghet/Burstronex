#include "Shader.h"

#include "Logger.h"

#include <fstream>

Shader::Shader(const std::string& VertexPath, const std::string& FragmentPath)
{
    unsigned int VertexShader = 0;
    unsigned int FragmentShader = 0;
    CompileShader(VertexPath, GL_VERTEX_SHADER, VertexShader);
    CompileShader(FragmentPath, GL_FRAGMENT_SHADER, FragmentShader);

    CreateAndLinkProgram(VertexShader, FragmentShader);
}

void Shader::Use()
{
    glUseProgram(mShaderProgram);
}

void Shader::CreateAndLinkProgram(unsigned int VertexShader, unsigned int FragmentShader)
{
    mShaderProgram = glCreateProgram();

    glAttachShader(mShaderProgram, VertexShader);
    glAttachShader(mShaderProgram, FragmentShader);

    glLinkProgram(mShaderProgram);

    int Success;
    glGetProgramiv(mShaderProgram, GL_LINK_STATUS, &Success);
    if (!Success) {
        int LogLength;
        glGetProgramiv(mShaderProgram, GL_INFO_LOG_LENGTH, &LogLength);

        std::vector<GLchar> Log(LogLength);
        glGetProgramInfoLog(mShaderProgram, LogLength, &LogLength, Log.data());

        mLog->Log(ELOGGING_SOURCES::RENDERER, ELOGGING_MODE::ERROR, "Link Program Error: {0}\n", std::string_view(Log.data(), Log.size()));
    }

    glUseProgram(mShaderProgram);

    glDeleteShader(VertexShader);
    glDeleteShader(FragmentShader);
}

void Shader::CompileShader(const std::string& PathToShader, GLenum Type, unsigned int& ShaderStorage)
{
    std::vector<char> ShaderFile = ReadShader(PathToShader);
    ShaderStorage = glCreateShader(Type);

    const GLchar* ShaderPtr = (GLchar*)(ShaderFile.data());
    const GLint ShaderSize = ShaderFile.size();
    glShaderSource(ShaderStorage, 1, &ShaderPtr, &ShaderSize);
    glCompileShader(ShaderStorage);

    int Success;
    glGetShaderiv(ShaderStorage, GL_COMPILE_STATUS, &Success);
    if (!Success)
    {
        int LogLength;
        glGetProgramiv(ShaderStorage, GL_INFO_LOG_LENGTH, &LogLength);

        std::vector<GLchar> Log(LogLength);
        glGetProgramInfoLog(ShaderStorage, LogLength, &LogLength, Log.data());

        mLog->Log(ELOGGING_SOURCES::RENDERER, ELOGGING_MODE::ERROR, "Compile Shader Error: {0}\n", std::string_view(Log.data(), Log.size()));
    }
}

std::vector<char> Shader::ReadShader(const std::string& PathToShader)
{
    std::ifstream ShaderFile(PathToShader, std::ios::binary | std::ios::ate);

    if (!ShaderFile)
    {
        mLog->Log(ELOGGING_SOURCES::RENDERER, ELOGGING_MODE::ERROR, "Shader not found at {0}\n", PathToShader);
        return std::vector<char>();
    }

    std::streamsize Size = ShaderFile.tellg();
    ShaderFile.seekg(0, std::ios::beg);

    std::vector<char> FileBuffer(Size);
    if (!ShaderFile.read(FileBuffer.data(), Size))
    {
        mLog->Log(ELOGGING_SOURCES::RENDERER, ELOGGING_MODE::ERROR, "Failed to read shader file at {0}\n", PathToShader);
    }

    mLog->Log(ELOGGING_SOURCES::RENDERER, ELOGGING_MODE::INFO, "Shader File at {0}, Size {1}\n", PathToShader, Size);

    return FileBuffer;
}
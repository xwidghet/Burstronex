#pragma once

#include "Shader.h"

#include "../include/glad/glad.h"

#include <atomic>
#include <functional>
#include <memory>
#include <string>

struct GLFWwindow;
enum class EControllerButtonMasks;

class Renderer {
    GLFWwindow* mWindow = nullptr;
    std::function<void()> mShutdownFunction;
    bool mbShowDebugWindow = true;

    std::atomic<uint8_t> mController1 = 0;
    std::atomic<uint8_t> mController2 = 0;

    unsigned int mQuadVBO = 0;
    unsigned int mQuadVAO = 0;
    std::unique_ptr<Shader> mQuadShader;

public:
    ~Renderer();

    // Not Thread Safe, should only be used prior to starting the thread.
    void Init(std::function<void()> ShutdownFunction);

    // Not Thread Safe, should only be used to start the thread.
    void Tick();

    // Thread Safe
    uint8_t GetController1();

    // Thread Safe
    uint8_t GetController2();

private:
    void InitDrawData();

    void CompileShaders();

    void CompileShader(const std::string& PathToShader, GLenum Type, unsigned int& ShaderStorage);

    std::vector<char> ReadShader(const std::string& PathToShader);

    void UpdateInputs(const bool bController2, const EControllerButtonMasks Button, const uint8_t bPressed);

    static void KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);

    void RenderFrame();

    void RenderDebug();
};

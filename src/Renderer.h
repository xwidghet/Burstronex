#pragma once

#include "Shader.h"

#include "../include/glad/glad.h"

#include <array>
#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

struct GLFWwindow;
enum class EControllerButtonMasks;

class Renderer {
    GLFWwindow* mWindow = nullptr;
    std::function<void()> mShutdownFunction;
    std::function<void()> mThrottleFunction;

    bool mbShowDebugWindow = true;
    bool mbShowPallete = false;

    std::atomic<uint8_t> mController1 = 0;
    std::atomic<uint8_t> mController2 = 0;

    unsigned int mQuadVBO = 0;
    unsigned int mQuadVAO = 0;
    std::unique_ptr<Shader> mQuadShader;
    std::unique_ptr<Shader> mPalleteShader;

    unsigned int mSharedPPUMemorySSBO = 0;

    struct SharedPPUMemory {
        SharedPPUMemory();

        std::unique_ptr<std::mutex> mMutex;

        GLint mPPUCTRL;
        std::array<GLint, 16384> mPPUMemory;
        std::array<GLint, 61696> mBackgroundDrawData;
        std::array<GLint, 8192> mChrMemory;
        std::array<GLint, 32> mPalleteMemory;
        std::array<GLint, 256> mObjectAttributeMemory;

        const uint32_t Size = (1 + 16384 + 61696 + 8192 + 32 + 256) * sizeof(GLint);
    } mSharedPPUMemory;

public:
    Renderer();
    ~Renderer();

    // Not Thread Safe, should only be used prior to starting the thread.
    void Init(std::function<void()> ShutdownFunction, std::function<void()> ThrottleFunction);

    // Not Thread Safe, should only be used to start the thread.
    void Tick();

    // Thread Safe
    uint8_t GetController1();

    // Thread Safe
    uint8_t GetController2();

    // Thread Safe
    void CopyPPUMemory(const uint8_t PPUCTRL, const std::array<uint16_t, 61696>& BackgroundDrawData, const std::array<uint8_t, 8192>& ChrMemory, const std::array<uint8_t, 16384>& PPUMemory, const std::array<uint8_t, 32>& PalleteMemory, const std::array<uint8_t, 256>& ObjectAttributeMemory);

private:
    void InitDrawData();

    void CompileShaders();

    void CreateDataBuffers();

    void UpdateSharedPPUMemorySSBO();

    void UpdateInputs(const bool bController2, const EControllerButtonMasks Button, const uint8_t bPressed);

    static void KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);

    void RenderFrame();

    static void FramebufferSizeCallback(GLFWwindow* Window, int Width, int Height);

    static void ApplyIntegerScaling(GLFWwindow* Window, int Width, int Height);

    void RenderDebug();

    void DrawPallete();
};

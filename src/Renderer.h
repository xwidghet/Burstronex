#pragma once

#include <atomic>
#include <functional>

struct GLFWwindow;
enum class EControllerButtonMasks;

class Renderer {
    GLFWwindow* mWindow = nullptr;
    std::function<void()> mShutdownFunction;
    bool mbShowDebugWindow = true;

    std::atomic<uint8_t> mController1 = 0;
    std::atomic<uint8_t> mController2 = 0;

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
    void UpdateInputs(const bool bController2, const EControllerButtonMasks Button, const uint8_t bPressed);

    static void KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);

    void RenderFrame();

    void RenderDebug();
};

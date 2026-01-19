#pragma once

class GLFWwindow;

class Renderer {
    GLFWwindow* mWindow;
    bool mbShowDebugWindow = true;

public:
    void Tick();

private:
    void RenderFrame();

    void RenderDebug();
};

#include "Renderer.h"

#include "Input.h"
#include "Logger.h"

#include "../include/glad/glad.h"

#include "../thirdparty/glfw/include/GLFW/glfw3.h"

#include "../thirdparty/imgui/imgui.h"
#include "../thirdparty/imgui/backends/imgui_impl_glfw.h"
#include "../thirdparty/imgui/backends/imgui_impl_opengl3.h"
#include <atomic>

Renderer::~Renderer()
{
    // External force has shutdown the Emulator, cleanup GLFW to avoid leaving a stale window open.
    if (!glfwWindowShouldClose(mWindow))
    {
        glfwTerminate();
    }
}

void Renderer::Init(std::function<void()> ShutdownFunction)
{
    mShutdownFunction = std::move(ShutdownFunction);
}

void Renderer::Tick()
{
    if (!glfwInit())
    {
        mLog->Log(ELOGGING_SOURCES::RENDERER, ELOGGING_MODE::ERROR, "Failed to initialize GLFW\n");

        return;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    mWindow = glfwCreateWindow(1920, 1080, "GNES", NULL, NULL);
    if (mWindow == nullptr)
    {
        mLog->Log(ELOGGING_SOURCES::RENDERER, ELOGGING_MODE::ERROR, "Failed to create window\n");
        glfwTerminate();

        return;
    }

    glfwMakeContextCurrent(mWindow);

    glfwSetWindowUserPointer(mWindow, this);
    glfwSetKeyCallback(mWindow, &KeyCallback);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        mLog->Log(ELOGGING_SOURCES::RENDERER, ELOGGING_MODE::ERROR, "Failed to initialize GLAD\n");

        return;
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui_ImplGlfw_InitForOpenGL(mWindow, true);
    ImGui_ImplOpenGL3_Init();

    while(!glfwWindowShouldClose(mWindow))
    {
        glfwPollEvents();

        glClear(GL_COLOR_BUFFER_BIT);

        RenderFrame();
        RenderDebug();

        glfwSwapBuffers(mWindow);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwTerminate();

    // Notify GNES that we're done.
    mShutdownFunction();
}

void Renderer::UpdateInputs(bool bController2, EControllerButtonMasks Button, bool bPressed)
{
    uint8_t ButtonMask = static_cast<uint8_t>(Button);

    if (bController2)
    {
        mController2.fetch_and(~ButtonMask, std::memory_order_relaxed);
        mController2.fetch_or(ButtonMask & bPressed, std::memory_order_relaxed);
    }
    else
    {
        mController1.fetch_and(~ButtonMask, std::memory_order_relaxed);
        mController1.fetch_or(ButtonMask & bPressed, std::memory_order_relaxed);
    }
}

void Renderer::KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (action == GLFW_REPEAT)
        return;

    Renderer* RendererPtr = static_cast<Renderer*>(glfwGetWindowUserPointer(window));
    if (RendererPtr)
    {
        bool bButtonState = action == GLFW_PRESS;

        switch(key)
        {
            case GLFW_KEY_ESCAPE:
                glfwSetWindowShouldClose(RendererPtr->mWindow, true);
                break;
            case GLFW_KEY_LEFT:
                RendererPtr->UpdateInputs(false, EControllerButtonMasks::LEFT, bButtonState);
                break;
            case GLFW_KEY_RIGHT:
                RendererPtr->UpdateInputs(false, EControllerButtonMasks::RIGHT, bButtonState);
                break;
            case GLFW_KEY_UP:
                RendererPtr->UpdateInputs(false, EControllerButtonMasks::UP, bButtonState);
                break;
            case GLFW_KEY_DOWN:
                RendererPtr->UpdateInputs(false, EControllerButtonMasks::DOWN, bButtonState);
                break;
            case GLFW_KEY_ENTER:
                RendererPtr->UpdateInputs(false, EControllerButtonMasks::START, bButtonState);
                break;
            case GLFW_KEY_Z:
                RendererPtr->UpdateInputs(false, EControllerButtonMasks::B, bButtonState);
                break;
            case GLFW_KEY_X:
                RendererPtr->UpdateInputs(false, EControllerButtonMasks::A, bButtonState);
                break;
        }

        switch(mods)
        {
            case GLFW_MOD_SHIFT:
                RendererPtr->UpdateInputs(false, EControllerButtonMasks::SELECT, bButtonState);
                break;
        }
    }
}

void Renderer::RenderFrame()
{

}

void Renderer::RenderDebug()
{
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    if (mbShowDebugWindow)
    {
        ImGui::Begin("Debug", &mbShowDebugWindow);
        ImGui::Text("Test Post Please Ignore");
        ImGui::End();
    }

    ImGui::EndFrame();

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

uint8_t Renderer::GetController1()
{
    return mController1.load(std::memory_order_relaxed);
}

uint8_t Renderer::GetController2()
{
    return mController1.load(std::memory_order_relaxed);
}

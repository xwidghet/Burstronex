#include "Renderer.h"

#include "Logger.h"

#include "../include/glad/glad.h"

#include "../thirdparty/glfw/include/GLFW/glfw3.h"

#include "../thirdparty/imgui/imgui.h"
#include "../thirdparty/imgui/backends/imgui_impl_glfw.h"
#include "../thirdparty/imgui/backends/imgui_impl_opengl3.h"

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
        glClear(GL_COLOR_BUFFER_BIT);

        RenderFrame();
        RenderDebug();

        glfwSwapBuffers(mWindow);
        glfwPollEvents();
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwTerminate();
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

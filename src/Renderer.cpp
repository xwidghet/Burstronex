#include "Renderer.h"

#include "Input.h"
#include "Logger.h"

#include "../thirdparty/glfw/include/GLFW/glfw3.h"

#include "../thirdparty/imgui/imgui.h"
#include "../thirdparty/imgui/backends/imgui_impl_glfw.h"
#include "../thirdparty/imgui/backends/imgui_impl_opengl3.h"

#include <atomic>
#include <fstream>

Renderer::Renderer()
{

}

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

    mWindow = glfwCreateWindow(1440, 1080, "GNES", NULL, NULL);
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

    InitDrawData();

    int Width, Height;
    glfwGetFramebufferSize(mWindow, &Width, &Height);

    ApplyIntegerScaling(mWindow, Width, Height);
    glfwSetFramebufferSizeCallback(mWindow, FramebufferSizeCallback);

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

void Renderer::InitDrawData()
{
    float QuadVertices[] = {
        0.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        1.0, 0.0, 0.0,
        1.0, 0.0, 0.0,
        1.0, 1.0, 0.0,
        0.0, 1.0, 0.0
    };

    glGenBuffers(1, &mQuadVBO);
    glGenVertexArrays(1, &mQuadVAO);

    glBindVertexArray(mQuadVAO);
    glBindBuffer(GL_ARRAY_BUFFER, mQuadVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(QuadVertices), QuadVertices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    CompileShaders();
    CreateDataBuffers();
}

void Renderer::CompileShaders()
{
    // Windows compiles to build/builttype/binary.exe
    // Linux compiles to build/binary.exe
    //
    // TODO: fix this at some point...
    std::string ShaderFolderPath = "../shaders/";
#ifdef _WIN64
    ShaderFolderPath = "../../shaders/"
#endif

    mQuadShader = std::make_unique<Shader>(ShaderFolderPath + "Quad.vert", ShaderFolderPath + "Quad.frag");
    mPalleteShader = std::make_unique<Shader>(ShaderFolderPath + "Pallete.vert", ShaderFolderPath + "Pallete.frag");
}

void Renderer::CreateDataBuffers()
{
    glGenBuffers(1, &mSharedPPUMemorySSBO);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, mSharedPPUMemorySSBO);
    glBufferData(GL_SHADER_STORAGE_BUFFER, mSharedPPUMemory.Size, nullptr, GL_DYNAMIC_DRAW);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, mSharedPPUMemorySSBO);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
}

void Renderer::UpdateSharedPPUMemorySSBO()
{
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, mSharedPPUMemorySSBO);

    uint32_t Offset = 0;
    uint32_t DataSize = 1 * sizeof(GLint);
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, Offset, DataSize, &mSharedPPUMemory.mPPUCTRL);
    Offset += DataSize;

    DataSize = 16384 * sizeof(GLint);
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, Offset, DataSize, mSharedPPUMemory.mPPUMemory.data());
    Offset += DataSize;

    DataSize = 32 * sizeof(GLint);
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, Offset, DataSize, mSharedPPUMemory.mPalleteMemory.data());
    Offset += DataSize;

    DataSize = 256 * sizeof(GLint);
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, Offset, DataSize, mSharedPPUMemory.mObjectAttributeMemory.data());
}

void Renderer::UpdateInputs(const bool bController2, const EControllerButtonMasks Button, const uint8_t bPressed)
{
    uint8_t ButtonMask = static_cast<uint8_t>(Button);
    mLog->Log(ELOGGING_SOURCES::RENDERER, ELOGGING_MODE::INFO, "Renderer: Recieved Button Input {0}, {1}!!\n", ButtonMask, bPressed);
    if (bController2)
    {
        uint8_t ControllerState = mController2.load(std::memory_order_acquire);
        ControllerState &= ~ButtonMask;
        ControllerState |= ButtonMask * bPressed;

        mController2.store(ControllerState, std::memory_order_release);
    }
    else
    {
        uint8_t ControllerState = mController1.load(std::memory_order_acquire);
        ControllerState &= ~ButtonMask;
        ControllerState |= ButtonMask * bPressed;

        mController1.store(ControllerState, std::memory_order_release);
    }
}

void Renderer::KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (action == GLFW_REPEAT)
        return;

    Renderer* RendererPtr = static_cast<Renderer*>(glfwGetWindowUserPointer(window));
    if (RendererPtr)
    {
        uint8_t ButtonState = action == GLFW_PRESS;

        switch(key)
        {
            case GLFW_KEY_ESCAPE:
                glfwSetWindowShouldClose(RendererPtr->mWindow, true);
                break;
            case GLFW_KEY_F1:
                if (ButtonState)
                    RendererPtr->mbShowDebugWindow = !RendererPtr->mbShowDebugWindow;
                break;
            case GLFW_KEY_LEFT:
                RendererPtr->UpdateInputs(false, EControllerButtonMasks::LEFT, ButtonState);
                break;
            case GLFW_KEY_RIGHT:
                RendererPtr->UpdateInputs(false, EControllerButtonMasks::RIGHT, ButtonState);
                break;
            case GLFW_KEY_UP:
                RendererPtr->UpdateInputs(false, EControllerButtonMasks::UP, ButtonState);
                break;
            case GLFW_KEY_DOWN:
                RendererPtr->UpdateInputs(false, EControllerButtonMasks::DOWN, ButtonState);
                break;
            case GLFW_KEY_ENTER:
                RendererPtr->UpdateInputs(false, EControllerButtonMasks::START, ButtonState);
                break;
            case GLFW_KEY_RIGHT_SHIFT:
                RendererPtr->UpdateInputs(false, EControllerButtonMasks::SELECT, ButtonState);
                break;
            case GLFW_KEY_Z:
                RendererPtr->UpdateInputs(false, EControllerButtonMasks::B, ButtonState);
                break;
            case GLFW_KEY_X:
                RendererPtr->UpdateInputs(false, EControllerButtonMasks::A, ButtonState);
                break;
        }
    }
}

void Renderer::RenderFrame()
{
    mQuadShader->Use();

    UpdateSharedPPUMemorySSBO();

    glBindVertexArray(mQuadVAO);
    glDrawArrays(GL_TRIANGLES, 0, 6);
}

void Renderer::RenderDebug()
{
    if (mbShowPallete)
    {
        DrawPallete();
    }

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    if (mbShowDebugWindow)
    {
        ImGui::Begin("Debug", &mbShowDebugWindow);
        ImGui::Text("Test Post Please Ignore");

        if (ImGui::Button("Display Pallete"))
        {
            mbShowPallete = !mbShowPallete;
        }

        if (ImGui::Button("Recompile Shaders"))
        {
            mQuadShader->Reload();
            mPalleteShader->Reload();
        }

        if (ImGui::Button("Verbose Logging"))
        {
            mLog->SetLogMode(ELOGGING_MODE::VERBOSE);
        }


        ImGui::End();
    }

    ImGui::EndFrame();

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void Renderer::FramebufferSizeCallback(GLFWwindow* Window, int Width, int Height)
{
    ApplyIntegerScaling(Window, Width, Height);
}

void Renderer::ApplyIntegerScaling(GLFWwindow* Window, int Width, int Height)
{
    // Seems to be a GLFW bug that I *must* set viewport only on linux to display the correct area on window creation.
    // Theory: On Wayland before a frame has been displayed the window isn't displayed, which probably breaks it.
    //
    // While I'm here, might as well do aspect ratio correction to ensure pixels are square.
    GLsizei ScalerTarget = Height <= Width ? Height : Width;

    // Todo: The output has some non-square pixels, so there's an issue here or in the shader (Pallete).
    GLsizei TargetScaleRatio = ScalerTarget / 262;

    GLsizei TargetHeight = 262*TargetScaleRatio;
    GLsizei TargetWidth = 262*TargetScaleRatio;

    GLint XOffset = ((Width - TargetWidth)*0.5f);
    GLint YOffset = ((Height - TargetHeight)*0.5f);
    glViewport(XOffset, YOffset, TargetWidth, TargetHeight);
}

void Renderer::DrawPallete()
{
    mPalleteShader->Use();

    glBindVertexArray(mQuadVAO);
    glDrawArrays(GL_TRIANGLES, 0, 6);
}

uint8_t Renderer::GetController1()
{
    return mController1.load(std::memory_order_acquire);
}

uint8_t Renderer::GetController2()
{
    return mController2.load(std::memory_order_acquire);
}

void Renderer::CopyPPUMemory(const uint8_t PPUCTRL, const std::array<uint8_t, 16384>& PPUMemory, const std::array<uint8_t, 32>& PalleteMemory, const std::array<uint8_t, 256>& ObjectAttributeMemory)
{
    std::lock_guard<std::mutex> lock(*mSharedPPUMemory.mMutex);

    mSharedPPUMemory.mPPUCTRL = PPUCTRL;
    std::copy(PPUMemory.begin(), PPUMemory.end(), mSharedPPUMemory.mPPUMemory.begin());
    std::copy(PalleteMemory.begin(), PalleteMemory.end(), mSharedPPUMemory.mPalleteMemory.begin());
    std::copy(ObjectAttributeMemory.begin(), ObjectAttributeMemory.end(), mSharedPPUMemory.mObjectAttributeMemory.begin());
}

Renderer::SharedPPUMemory::SharedPPUMemory()
{
    mMutex = std::make_unique<std::mutex>();
}

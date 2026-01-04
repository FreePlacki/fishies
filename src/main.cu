#define GLFW_INCLUDE_NONE
#include "boids.cu"
#include "boids_gpu.cu"
#include "params.cu"
#include "shaders.cu"
#include <GLFW/glfw3.h>
#include <assert.h>
#include <cuda_runtime.h>
#include <glad/glad.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifdef _WIN32
#include <windows.h>
#endif

static GLuint compile_shader(GLenum type, const char *src) {
    GLuint s = glCreateShader(type);
    glShaderSource(s, 1, &src, NULL);
    glCompileShader(s);

    GLint ok = 0;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[1024];
        glGetShaderInfoLog(s, sizeof(log), NULL, log);
        fprintf(stderr, "Shader compile error:\n%s\n", log);
        exit(1);
    }
    return s;
}

static GLuint create_shader_program(void) {
    GLuint vs = compile_shader(GL_VERTEX_SHADER, vs_src);
    GLuint fs = compile_shader(GL_FRAGMENT_SHADER, fs_src);

    GLuint prog = glCreateProgram();
    glAttachShader(prog, vs);
    glAttachShader(prog, fs);
    glLinkProgram(prog);

    GLint ok = 0;
    glGetProgramiv(prog, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[1024];
        glGetProgramInfoLog(prog, sizeof(log), NULL, log);
        fprintf(stderr, "Program link error:\n%s\n", log);
        exit(1);
    }

    glDeleteShader(vs);
    glDeleteShader(fs);
    return prog;
}

static void framebuffer_size_callback(GLFWwindow *window, int w, int h) {
    glViewport(0, 0, w, h);
}

static void draw_ui(BoidsParams *p) {
    ImGui::Begin("Boids");

    ImGui::SeparatorText("Cohesion");
    ImGui::SliderFloat("Radius##cohesion", &p->cohesion_r, 0.01f, 0.2f);
    ImGui::SliderFloat("Strength##1", &p->cohesion_strength, -1.0f, 1.0f);
    ImGui::SeparatorText("Separation");
    ImGui::SliderFloat("Radius##2", &p->separation_r, 0.01f, 0.2f);
    ImGui::SliderFloat("Strength##2", &p->separation_strength, -1.0f, 1.0f);
    ImGui::SeparatorText("Alignment");
    ImGui::SliderFloat("Radius##3", &p->alignment_r, 0.01f, 0.2f);
    ImGui::SliderFloat("Strength##3", &p->alignment_strength, -1.0f, 1.0f);
    ImGui::SeparatorText("Other");
    ImGui::SliderFloat("Min Speed", &p->min_speed, 0.0f, 1.0f);
    ImGui::SliderFloat("Max Speed", &p->max_speed, p->min_speed, 3.0f);

    ImGui::End();
}

void usage(char *pname) {
    fprintf(stderr, "Usage\t%s\n", pname);
    exit(1);
}

int main(int argc, char **argv) {
    srand((unsigned)time(NULL));

    if (cudaSetDevice(0) != cudaSuccess) {
        fprintf(stderr, "CUDA device not available\n");
    }

    if (argc != 1)
        usage(argv[0]);

    if (!glfwInit()) {
        fprintf(stderr, "GLFW init failed\n");
        return 1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow *window = glfwCreateWindow(800, 600, "Fishies", NULL, NULL);
    if (!window) {
        glfwTerminate();
        return 1;
    }

    glfwMakeContextCurrent(window);
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        fprintf(stderr, "GLAD init failed\n");
        return 1;
    }
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();

    ImGuiIO &io = ImGui::GetIO();
    (void)io;

    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    glfwSwapInterval(1);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    int w, h;
    glfwGetFramebufferSize(window, &w, &h);
    glViewport(0, 0, w, h);

    InitialConfig cfg;
    if (!config_parse("config.ini", &cfg))
        printf("Couldn't parse config, using defaults\n");

    Boids boids;
    boids_init(&boids, &cfg);

    RenderBoid *render_boids =
        (RenderBoid *)malloc(boids.count * sizeof(RenderBoid));

    GLuint vao;
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    /* triangle mesh (per-vertex) */
    GLuint tri_vbo;
    glGenBuffers(1, &tri_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, tri_vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(tri_vertices), tri_vertices,
                 GL_STATIC_DRAW);

    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, (void *)0);
    glEnableVertexAttribArray(0);

    /* instance buffer (per-boid) */
    GLuint inst_vbo;
    glGenBuffers(1, &inst_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, inst_vbo);
    glBufferData(GL_ARRAY_BUFFER, boids.count * sizeof(RenderBoid), NULL,
                 GL_DYNAMIC_DRAW);

    /* position */
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(RenderBoid),
                          (void *)0);
    glEnableVertexAttribArray(1);
    glVertexAttribDivisor(1, 1);

    /* velocity */
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(RenderBoid),
                          (void *)(2 * sizeof(float)));
    glEnableVertexAttribArray(2);
    glVertexAttribDivisor(2, 1);

    /* type */
    glVertexAttribIPointer(3, 1, GL_UNSIGNED_BYTE, sizeof(RenderBoid),
                          (void *)(4 * sizeof(float)));
    glEnableVertexAttribArray(3);
    glVertexAttribDivisor(3, 1);

    glBindVertexArray(0);

    GLuint shader_program = create_shader_program();

    BoidsParams params = {
        .cohesion_r = 0.07f,
        .cohesion_strength = 1.0f,
        .separation_r = 0.02f,
        .separation_strength = 1.0f,
        .alignment_r = 0.10f,
        .alignment_strength = 1.0f,
        .min_speed = 0.05f,
        .max_speed = 1.0f,
    };

    double last_time = glfwGetTime();
    double last_fps_update = last_time;
    int n_frames = 0;
    while (!glfwWindowShouldClose(window)) {
        double now = glfwGetTime();
        float dt = (float)(now - last_time);
        last_time = now;
        n_frames++;

        // Update FPS every 0.25 seconds
        if (now - last_fps_update >= 0.25) {
            double fps = n_frames / (now - last_fps_update);

            char title[128];
            snprintf(title, sizeof(title), "Fishies (%.1f FPS)", fps);
            glfwSetWindowTitle(window, title);

            n_frames = 0;
            last_fps_update = now;
        }
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        draw_ui(&params);

        boids_update_gpu(&boids, &params, dt);
        boids_pack_positions(&boids, render_boids);

        glBindBuffer(GL_ARRAY_BUFFER, inst_vbo);
        glBufferSubData(GL_ARRAY_BUFFER, 0, boids.count * sizeof(RenderBoid),
                        render_boids);

        glClearColor(0.05f, 0.05f, 0.08f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        glUseProgram(shader_program);
        glBindVertexArray(vao);
        glDrawArraysInstanced(GL_TRIANGLES, 0, 3, boids.count);

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glDeleteProgram(shader_program);
    glDeleteBuffers(1, &inst_vbo);
    glDeleteBuffers(1, &tri_vbo);
    glDeleteVertexArrays(1, &vao);

    free(render_boids);
    boids_free(&boids);

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}

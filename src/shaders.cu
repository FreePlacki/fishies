const char *vs_src = "#version 330 core\n"
                     "layout (location = 0) in vec2 aLocalPos;\n"
                     "layout (location = 1) in vec2 aBoidPos;\n"
                     "layout (location = 2) in vec2 aBoidVel;\n"
                     "void main() {\n"
                     "    vec2 dir = normalize(aBoidVel);\n"
                     "    vec2 perp = vec2(-dir.y, dir.x);\n"
                     "    mat2 rot = mat2(dir, perp);\n"
                     "    vec2 worldPos = rot * aLocalPos + aBoidPos;\n"
                     "    gl_Position = vec4(worldPos, 0.0, 1.0);\n"
                     "}\n";

const char *fs_src = "#version 330 core\n"
                     "out vec4 FragColor;\n"
                     "void main() {\n"
                     "    FragColor = vec4(1.0, 1.0, 1.0, 1.0);\n"
                     "}\n";

#define TRIANGLE_SIZE 0.003f

static const float tri_vertices[6] = {
    TRIANGLE_SIZE * 2.0f,  0.0f,  // tip
    -TRIANGLE_SIZE, TRIANGLE_SIZE, // back top
    -TRIANGLE_SIZE, -TRIANGLE_SIZE // back bottom
};

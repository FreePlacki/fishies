const char *vs_src = "#version 330 core\n"
                     "layout (location = 0) in vec2 aLocalPos;\n"
                     "layout (location = 1) in vec2 aBoidPos;\n"
                     "layout (location = 2) in vec2 aBoidVel;\n"
                     "layout (location = 3) in uint aBoidType;\n"
                     "flat out uint v_type;\n"
                     "void main() {\n"
                     "    vec2 dir = normalize(aBoidVel);\n"
                     "    vec2 perp = vec2(-dir.y, dir.x);\n"
                     "    mat2 rot = mat2(dir, perp);\n"
                     "    vec2 worldPos = rot * aLocalPos + aBoidPos;\n"
                     "    gl_Position = vec4(worldPos, 0.0, 1.0);\n"
                     "    v_type = aBoidType;\n"
                     "}\n";

const char *fs_src = "#version 330 core\n"
                     "flat in uint v_type;\n"
                     "out vec4 FragColor;\n"
                     "vec3 hsv2rgb(vec3 c) {\n"
                     "    vec4 K = vec4(1.0, 2.0/3.0, 1.0/3.0, 3.0);\n"
                     "    vec3 p = abs(fract(c.xxx + K.xyz) * 6.0 - K.www);\n"
                     "    return c.z * mix(K.xxx, clamp(p - K.xxx, 0.0, 1.0), c.y);\n"
                     "}\n"
                     "void main() {\n"
                     "    float h = fract(float(v_type) * 0.61803398875);\n"
                     "    vec3 color = hsv2rgb(vec3(h, 0.8, 0.95));\n"
                     "    FragColor = vec4(color, 1.0);\n"
                     "}\n";

#define TRIANGLE_SIZE 0.002f
static const float tri_vertices[6] = {
    TRIANGLE_SIZE * 2.0f,  0.0f,  // tip
    -TRIANGLE_SIZE, TRIANGLE_SIZE, // back top
    -TRIANGLE_SIZE, -TRIANGLE_SIZE // back bottom
};

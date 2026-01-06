#ifndef UI_H
#define UI_H

#include "params.cu"
#include <imgui.h>

static void type_color(uint32_t type, float *r, float *g, float *b) {
    float h = fmodf(type * 0.61803398875f, 1.0f);

    float s = 0.8f;
    float v = 0.95f;

    float c = v * s;
    float x = c * (1.0f - fabsf(fmodf(h * 6.0f, 2.0f) - 1.0f));
    float m = v - c;

    float rr, gg, bb;

    int sector = (int)(h * 6.0f);
    switch (sector) {
    case 0:
        rr = c;
        gg = x;
        bb = 0;
        break;
    case 1:
        rr = x;
        gg = c;
        bb = 0;
        break;
    case 2:
        rr = 0;
        gg = c;
        bb = x;
        break;
    case 3:
        rr = 0;
        gg = x;
        bb = c;
        break;
    case 4:
        rr = x;
        gg = 0;
        bb = c;
        break;
    default:
        rr = c;
        gg = 0;
        bb = x;
        break;
    }

    *r = rr + m;
    *g = gg + m;
    *b = bb + m;
}

void draw_ui(BoidsParams *p) {
    ImGui::Begin("Params");

    for (int t = 0; t < p->type_count; ++t) {
        float r, g, b;
        type_color(t, &r, &g, &b);

        ImGui::PushID(t);

        char header[64];
        snprintf(header, sizeof(header), "Fish Type %d", t);

        if (ImGui::CollapsingHeader(header, ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::SameLine();
            ImGui::ColorButton("##color", ImVec4(r, g, b, 1.0f),
                               ImGuiColorEditFlags_NoTooltip |
                                   ImGuiColorEditFlags_NoDragDrop,
                               ImVec2(18, 18));

            BoidTypeParams *tp = &p->type[t];

            ImGui::SeparatorText("Cohesion");
            ImGui::SliderFloat("Radius##coh", &tp->cohesion_r, 0.0f, 0.2f);
            ImGui::SliderFloat("Strength##coh", &tp->cohesion_strength, -1.0f,
                               1.0f);

            ImGui::SeparatorText("Separation");
            ImGui::SliderFloat("Radius##sep", &tp->separation_r, 0.0f, 0.2f);
            ImGui::SliderFloat("Strength##sep", &tp->separation_strength, -1.0f,
                               1.0f);

            ImGui::SeparatorText("Alignment");
            ImGui::SliderFloat("Radius##ali", &tp->alignment_r, 0.0f, 0.2f);
            ImGui::SliderFloat("Strength##ali", &tp->alignment_strength, -1.0f,
                               1.0f);

            ImGui::SeparatorText("Speed");
            ImGui::SliderFloat("Min Speed", &tp->min_speed, 0.01f, 1.0f);
            ImGui::SliderFloat("Max Speed", &tp->max_speed, tp->min_speed,
                               3.0f);
        }

        ImGui::PopID();
    }

    ImGui::End();
}

#endif /* UI_H */

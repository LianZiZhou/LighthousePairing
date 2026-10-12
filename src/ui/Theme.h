#pragma once
#include "imgui.h"

namespace lhp::ui {

struct Palette {
    ImVec4 bg, panel, panelAlt, card, cardHover, cardActive, border, text, textDim, textFaint;
    ImVec4 accent, accentHover, accentActive, accentSoft;
    ImVec4 success, warning, danger, info, purple;
    ImVec4 plotImu, plotUsb, plotGap, plotJitter;
    bool dark = true;
};

struct Fonts {
    ImFont* ui = nullptr;
    ImFont* bold = nullptr;
    ImFont* mono = nullptr;
    float base = 15.0f;
    float iconOffsetY = 0.0f;
};

const Palette& Pal();
Fonts& GetFonts();

/// theme: 0 dark, 1 light, 2 follow system
void ApplyTheme(int theme, float dpiScale, bool translucent);
bool SystemPrefersDark();
/// (Re)build the font atlas. `iconOffsetY` shifts the merged icon font vertically (base-size px).
void LoadFonts(float iconOffsetY);
/// Must be called inside a frame. Vertical correction needed to optically centre icons on CJK text.
float MeasureIconMisalignment();

inline ImU32 Col(const ImVec4& c, float alphaMul = 1.0f) { return ImGui::ColorConvertFloat4ToU32(ImVec4(c.x, c.y, c.z, c.w * alphaMul)); }
ImVec4 Mix(const ImVec4& a, const ImVec4& b, float t);

}  // namespace lhp::ui

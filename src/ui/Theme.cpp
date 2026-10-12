#include "ui/Theme.h"

#include <windows.h>

#include "implot.h"

namespace lhp::ui {

static Palette g_pal;
static Fonts g_fonts;

const Palette& Pal() { return g_pal; }
Fonts& GetFonts() { return g_fonts; }

static ImVec4 Hex(unsigned rgb, float a = 1.0f) {
    return ImVec4(((rgb >> 16) & 0xFF) / 255.f, ((rgb >> 8) & 0xFF) / 255.f, (rgb & 0xFF) / 255.f, a);
}

ImVec4 Mix(const ImVec4& a, const ImVec4& b, float t) {
    return ImVec4(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t, a.w + (b.w - a.w) * t);
}

bool SystemPrefersDark() {
    DWORD v = 1, size = sizeof v;
    if (RegGetValueW(HKEY_CURRENT_USER, L"Software\\Microsoft\\Windows\\CurrentVersion\\Themes\\Personalize", L"AppsUseLightTheme", RRF_RT_REG_DWORD,
                     nullptr, &v, &size) == ERROR_SUCCESS)
        return v == 0;
    return true;
}

static void BuildPalette(bool dark, bool translucent) {
    Palette p;
    p.dark = dark;
    if (dark) {
        p.bg = Hex(0x111318, translucent ? 0.60f : 1.0f);
        p.panel = Hex(0x171A20, translucent ? 0.78f : 1.0f);
        p.panelAlt = Hex(0x1D2128, translucent ? 0.85f : 1.0f);
        p.card = Hex(0x20242C, 0.92f);
        p.cardHover = Hex(0x272C36, 0.96f);
        p.cardActive = Hex(0x2B3242, 1.0f);
        p.border = Hex(0xFFFFFF, 0.075f);
        p.text = Hex(0xE8EAED);
        p.textDim = Hex(0x9AA3AF);
        p.textFaint = Hex(0x6B7280);
        p.accent = Hex(0x4F8CFF);
        p.accentHover = Hex(0x6A9DFF);
        p.accentActive = Hex(0x3B74E0);
        p.accentSoft = Hex(0x4F8CFF, 0.16f);
        p.success = Hex(0x3DDC97);
        p.warning = Hex(0xFFB547);
        p.danger = Hex(0xFF5D5D);
        p.info = Hex(0x56C2FF);
        p.purple = Hex(0xB18CFF);
        p.plotImu = Hex(0x4F8CFF);
        p.plotUsb = Hex(0x3DDC97);
        p.plotGap = Hex(0xFF5D5D);
        p.plotJitter = Hex(0xFFB547);
    } else {
        p.bg = Hex(0xF3F4F7, translucent ? 0.55f : 1.0f);
        p.panel = Hex(0xFAFBFC, translucent ? 0.80f : 1.0f);
        p.panelAlt = Hex(0xFFFFFF, translucent ? 0.88f : 1.0f);
        p.card = Hex(0xFFFFFF, 0.95f);
        p.cardHover = Hex(0xF2F6FF, 1.0f);
        p.cardActive = Hex(0xE6EEFF, 1.0f);
        p.border = Hex(0x000000, 0.09f);
        p.text = Hex(0x1C1F24);
        p.textDim = Hex(0x5B6472);
        p.textFaint = Hex(0x8A93A0);
        p.accent = Hex(0x2F6FEB);
        p.accentHover = Hex(0x4A82F0);
        p.accentActive = Hex(0x235BCB);
        p.accentSoft = Hex(0x2F6FEB, 0.12f);
        p.success = Hex(0x14A86B);
        p.warning = Hex(0xD98A00);
        p.danger = Hex(0xE03E3E);
        p.info = Hex(0x1C8FD6);
        p.purple = Hex(0x7C4DDB);
        p.plotImu = Hex(0x2F6FEB);
        p.plotUsb = Hex(0x14A86B);
        p.plotGap = Hex(0xE03E3E);
        p.plotJitter = Hex(0xD98A00);
    }
    g_pal = p;
}

void ApplyTheme(int theme, float dpiScale, bool translucent) {
    bool dark = theme == 0 || (theme == 2 && SystemPrefersDark());
    BuildPalette(dark, translucent);
    const Palette& p = g_pal;

    ImGuiStyle& s = ImGui::GetStyle();
    s = ImGuiStyle();
    // Close to Dear ImGui's own proportions: compact, small radii, information-dense.
    s.WindowPadding = ImVec2(8, 7);
    s.FramePadding = ImVec2(6, 3);
    s.CellPadding = ImVec2(5, 2);
    s.ItemSpacing = ImVec2(7, 4);
    s.ItemInnerSpacing = ImVec2(4, 4);
    s.IndentSpacing = 16;
    s.ScrollbarSize = 11;
    s.GrabMinSize = 9;
    s.WindowBorderSize = 0;
    s.ChildBorderSize = 1;
    s.PopupBorderSize = 1;
    s.FrameBorderSize = 0;
    s.TabBorderSize = 0;
    s.WindowRounding = 4;
    s.ChildRounding = 3;
    s.FrameRounding = 3;
    s.PopupRounding = 4;
    s.ScrollbarRounding = 6;
    s.GrabRounding = 3;
    s.TabRounding = 3;
    s.WindowTitleAlign = ImVec2(0.0f, 0.5f);
    s.SeparatorTextBorderSize = 1;
    s.DockingSeparatorSize = 2;
    s.SelectableTextAlign = ImVec2(0, 0.5f);
    s.ScaleAllSizes(dpiScale);
    s.FontScaleDpi = dpiScale;
    s.FontSizeBase = g_fonts.base;

    ImVec4* c = s.Colors;
    c[ImGuiCol_Text] = p.text;
    c[ImGuiCol_TextDisabled] = p.textFaint;
    c[ImGuiCol_WindowBg] = p.panel;
    c[ImGuiCol_ChildBg] = ImVec4(0, 0, 0, 0);
    c[ImGuiCol_PopupBg] = dark ? Hex(0x1E222A, 0.98f) : Hex(0xFFFFFF, 0.98f);
    c[ImGuiCol_Border] = p.border;
    c[ImGuiCol_BorderShadow] = ImVec4(0, 0, 0, 0);
    c[ImGuiCol_FrameBg] = dark ? Hex(0xFFFFFF, 0.055f) : Hex(0x000000, 0.045f);
    c[ImGuiCol_FrameBgHovered] = dark ? Hex(0xFFFFFF, 0.085f) : Hex(0x000000, 0.07f);
    c[ImGuiCol_FrameBgActive] = p.accentSoft;
    c[ImGuiCol_TitleBg] = p.panel;
    c[ImGuiCol_TitleBgActive] = p.panel;
    c[ImGuiCol_TitleBgCollapsed] = p.panel;
    c[ImGuiCol_MenuBarBg] = ImVec4(0, 0, 0, 0);
    c[ImGuiCol_ScrollbarBg] = ImVec4(0, 0, 0, 0);
    c[ImGuiCol_ScrollbarGrab] = dark ? Hex(0xFFFFFF, 0.12f) : Hex(0x000000, 0.15f);
    c[ImGuiCol_ScrollbarGrabHovered] = dark ? Hex(0xFFFFFF, 0.2f) : Hex(0x000000, 0.25f);
    c[ImGuiCol_ScrollbarGrabActive] = p.accent;
    c[ImGuiCol_CheckMark] = p.accent;
    c[ImGuiCol_SliderGrab] = p.accent;
    c[ImGuiCol_SliderGrabActive] = p.accentHover;
    c[ImGuiCol_Button] = dark ? Hex(0xFFFFFF, 0.07f) : Hex(0x000000, 0.055f);
    c[ImGuiCol_ButtonHovered] = dark ? Hex(0xFFFFFF, 0.12f) : Hex(0x000000, 0.09f);
    c[ImGuiCol_ButtonActive] = dark ? Hex(0xFFFFFF, 0.17f) : Hex(0x000000, 0.13f);
    c[ImGuiCol_Header] = p.accentSoft;
    c[ImGuiCol_HeaderHovered] = dark ? Hex(0xFFFFFF, 0.07f) : Hex(0x000000, 0.05f);
    c[ImGuiCol_HeaderActive] = Mix(p.accentSoft, p.accent, 0.25f);
    c[ImGuiCol_Separator] = p.border;
    c[ImGuiCol_SeparatorHovered] = p.accent;
    c[ImGuiCol_SeparatorActive] = p.accent;
    c[ImGuiCol_ResizeGrip] = ImVec4(0, 0, 0, 0);
    c[ImGuiCol_ResizeGripHovered] = p.accentSoft;
    c[ImGuiCol_ResizeGripActive] = p.accent;
    c[ImGuiCol_InputTextCursor] = p.accent;
    c[ImGuiCol_Tab] = ImVec4(0, 0, 0, 0);
    c[ImGuiCol_TabHovered] = dark ? Hex(0xFFFFFF, 0.08f) : Hex(0x000000, 0.06f);
    c[ImGuiCol_TabSelected] = dark ? Hex(0xFFFFFF, 0.075f) : Hex(0xFFFFFF, 1.0f);
    c[ImGuiCol_TabSelectedOverline] = p.accent;
    c[ImGuiCol_TabDimmed] = ImVec4(0, 0, 0, 0);
    c[ImGuiCol_TabDimmedSelected] = dark ? Hex(0xFFFFFF, 0.05f) : Hex(0xFFFFFF, 0.8f);
    c[ImGuiCol_TabDimmedSelectedOverline] = ImVec4(0, 0, 0, 0);
    c[ImGuiCol_DockingPreview] = Mix(p.accent, ImVec4(0, 0, 0, 0), 0.4f);
    c[ImGuiCol_DockingEmptyBg] = ImVec4(0, 0, 0, 0);
    c[ImGuiCol_PlotLines] = p.accent;
    c[ImGuiCol_PlotLinesHovered] = p.accentHover;
    c[ImGuiCol_PlotHistogram] = p.accent;
    c[ImGuiCol_PlotHistogramHovered] = p.accentHover;
    c[ImGuiCol_TableHeaderBg] = dark ? Hex(0xFFFFFF, 0.04f) : Hex(0x000000, 0.035f);
    c[ImGuiCol_TableBorderStrong] = p.border;
    c[ImGuiCol_TableBorderLight] = p.border;
    c[ImGuiCol_TableRowBg] = ImVec4(0, 0, 0, 0);
    c[ImGuiCol_TableRowBgAlt] = dark ? Hex(0xFFFFFF, 0.022f) : Hex(0x000000, 0.02f);
    c[ImGuiCol_TextLink] = p.accent;
    c[ImGuiCol_TextSelectedBg] = Mix(p.accentSoft, p.accent, 0.3f);
    c[ImGuiCol_DragDropTarget] = p.accent;
    c[ImGuiCol_NavCursor] = p.accent;
    c[ImGuiCol_ModalWindowDimBg] = dark ? Hex(0x000000, 0.55f) : Hex(0x000000, 0.25f);

    if (ImPlot::GetCurrentContext()) {
        ImPlotStyle& ps = ImPlot::GetStyle();
        ps = ImPlotStyle();
        if (dark) ImPlot::StyleColorsDark();
        else ImPlot::StyleColorsLight();
        ps.Colors[ImPlotCol_FrameBg] = ImVec4(0, 0, 0, 0);
        ps.Colors[ImPlotCol_PlotBg] = dark ? Hex(0x000000, 0.18f) : Hex(0x000000, 0.025f);
        ps.Colors[ImPlotCol_PlotBorder] = p.border;
        ps.Colors[ImPlotCol_LegendBg] = dark ? Hex(0x1A1D23, 0.9f) : Hex(0xFFFFFF, 0.9f);
        ps.Colors[ImPlotCol_LegendBorder] = p.border;
        ps.Colors[ImPlotCol_AxisGrid] = dark ? Hex(0xFFFFFF, 0.06f) : Hex(0x000000, 0.06f);
        ps.PlotPadding = ImVec2(6 * dpiScale, 6 * dpiScale);
        ps.PlotBorderSize = 0;
        ps.MinorAlpha = 0.5f;
    }
}

float MeasureIconMisalignment() {
    // Compare the vertical centre of a symmetric icon (Info circle, U+E946) with a symmetric
    // CJK glyph (中). Returns the correction in unscaled font units (px at base size).
    ImFontBaked* fb = ImGui::GetFontBaked();
    if (!fb) return 0;
    const ImFontGlyph* gi = fb->FindGlyph((ImWchar)0xE946);
    const ImFontGlyph* gc = fb->FindGlyph((ImWchar)0x4E2D);
    if (!gi || !gc || gi->Y1 <= gi->Y0 || gc->Y1 <= gc->Y0) return 0;
    float deltaPx = (gc->Y0 + gc->Y1) * 0.5f - (gi->Y0 + gi->Y1) * 0.5f;
    float pxPerUnit = fb->Size / g_fonts.base;
    return pxPerUnit > 0 ? deltaPx / pxPerUnit : 0;
}

void LoadFonts(float iconOffsetY) {
    g_fonts.iconOffsetY = iconOffsetY;
    ImGuiIO& io = ImGui::GetIO();
    io.Fonts->Clear();
    const float size = g_fonts.base;
    auto exists = [](const char* p) { return GetFileAttributesA(p) != INVALID_FILE_ATTRIBUTES; };
    const char* iconFont = exists("C:\\Windows\\Fonts\\SegoeIcons.ttf") ? "C:\\Windows\\Fonts\\SegoeIcons.ttf" : "C:\\Windows\\Fonts\\segmdl2.ttf";

    auto addFamily = [&](const char* latin, const char* cjk, int cjkIndex) -> ImFont* {
        ImFontConfig cfg;
        cfg.OversampleH = 1;
        ImFont* f = nullptr;
        if (exists(latin)) f = io.Fonts->AddFontFromFileTTF(latin, size, &cfg);
        if (!f) f = io.Fonts->AddFontDefaultVector();
        if (exists(cjk)) {
            ImFontConfig m;
            m.MergeMode = true;
            m.FontNo = cjkIndex;
            io.Fonts->AddFontFromFileTTF(cjk, size, &m);
        }
        if (exists(iconFont)) {
            ImFontConfig ic;
            ic.MergeMode = true;
            ic.GlyphOffset = ImVec2(0, iconOffsetY);   // calibrated at startup (see MeasureIconMisalignment)
            ic.GlyphMinAdvanceX = size;
            io.Fonts->AddFontFromFileTTF(iconFont, size * 0.92f, &ic);
        }
        return f;
    };
    g_fonts.ui = addFamily("C:\\Windows\\Fonts\\segoeui.ttf", "C:\\Windows\\Fonts\\msyh.ttc", 0);
    g_fonts.bold = addFamily("C:\\Windows\\Fonts\\seguisb.ttf", "C:\\Windows\\Fonts\\msyhbd.ttc", 0);
    const char* mono = exists("C:\\Windows\\Fonts\\CascadiaMono.ttf") ? "C:\\Windows\\Fonts\\CascadiaMono.ttf" : "C:\\Windows\\Fonts\\consola.ttf";
    g_fonts.mono = addFamily(mono, "C:\\Windows\\Fonts\\msyh.ttc", 0);
    io.FontDefault = g_fonts.ui;
}

}  // namespace lhp::ui

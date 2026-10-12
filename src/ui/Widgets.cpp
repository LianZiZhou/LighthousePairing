#include "ui/Widgets.h"

#include "backend/SystemInfo.h"
#include "imgui_internal.h"
#include "ui/Icons.h"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>

namespace lhp::ui {

std::string Fmt(const char* fmt, ...) {
    char buf[1024];
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    return buf;
}

float Animate(ImGuiID id, float target, float speed) {
    ImGuiStorage* st = ImGui::GetStateStorage();
    float* v = st->GetFloatRef(id, target);
    float dt = ImGui::GetIO().DeltaTime;
    *v += (target - *v) * (1.0f - std::exp(-speed * dt));
    if (std::fabs(*v - target) < 0.001f) *v = target;
    return *v;
}

float AnimateStr(const char* key, float target, float speed) { return Animate(ImGui::GetID(key), target, speed); }

float Pulse(float hz) { return 0.5f + 0.5f * std::sin((float)ImGui::GetTime() * hz * 6.2831853f); }

void StatusDot(const ImVec4& color, float radius, bool pulse) {
    ImDrawList* dl = ImGui::GetWindowDrawList();
    ImVec2 p = ImGui::GetCursorScreenPos();
    float h = ImGui::GetTextLineHeight();
    ImVec2 c(p.x + radius, p.y + h * 0.5f);
    (void)pulse;   // kept for API compatibility; no glow/pulse (keeps the plain ImGui look)
    float cy = p.y + h * 0.5f + (TextOpticalCenter() - h * 0.5f);
    dl->AddCircleFilled(ImVec2(c.x, cy), radius, Col(color));
    ImGui::Dummy(ImVec2(radius * 2 + 2, h));
}

float TextOpticalCenter() {
    // Vertical centre of a CJK ideograph relative to the top of a text line, measured from the
    // real glyph box. Aligning shapes to this (rather than to line height / 2) is what makes
    // icons, dots and tags look centred next to Chinese text.
    ImFontBaked* fb = ImGui::GetFontBaked();
    if (fb)
        if (const ImFontGlyph* g = fb->FindGlyph((ImWchar)0x4E2D); g && g->Y1 > g->Y0) return (g->Y0 + g->Y1) * 0.5f;
    return ImGui::GetFontSize() * 0.5f;
}

ImVec2 Chip(const char* text, const ImVec4& color, bool filled) {
    // The tag occupies one frame-height slot (same as buttons / AlignTextToFramePadding text)
    // and is vertically centred inside it, so tags line up with whatever is next to them.
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const float fs = ImGui::GetFontSize();
    const float slotH = ImGui::GetFrameHeight();
    ImVec2 ts = ImGui::CalcTextSize(text);
    float padX = fs * 0.4f;
    float chipH = std::min(slotH, fs * 1.25f);
    ImVec2 p = ImGui::GetCursorScreenPos();
    float y0 = IM_ROUND(p.y + (slotH - chipH) * 0.5f);
    ImVec2 a(p.x, y0), b(p.x + ts.x + padX * 2, y0 + chipH);
    float r = ImGui::GetStyle().FrameRounding;
    dl->AddRectFilled(a, b, Col(color, filled ? 0.85f : 0.14f), r);
    // Optical centring: glyph centre (not line box centre) on the tag centre.
    float textY = IM_ROUND(y0 + chipH * 0.5f - TextOpticalCenter());
    dl->AddText(ImVec2(a.x + padX, textY), filled ? IM_COL32(255, 255, 255, 255) : Col(color), text);
    ImVec2 size(b.x - a.x, slotH);
    ImGui::Dummy(size);
    return size;
}

void ChipSameLine(const char* text, const ImVec4& color, bool filled) {
    ImGui::SameLine(0, ImGui::GetStyle().ItemInnerSpacing.x);
    Chip(text, color, filled);
}

bool IconButton(const char* icon, const char* tooltip, const ImVec2& size) {
    // Square, transparent button whose icon is centred on its actual glyph bounds
    // (icon fonts carry their own baseline/offset, so text-style layout leaves them off-centre).
    const float fh = ImGui::GetFrameHeight();
    ImVec2 sz(size.x > 0 ? size.x : fh, size.y > 0 ? size.y : fh);
    ImGui::PushID(tooltip ? tooltip : icon);
    ImVec2 p = ImGui::GetCursorScreenPos();
    bool pressed = ImGui::InvisibleButton("##ib", sz);
    bool hovered = ImGui::IsItemHovered(), held = ImGui::IsItemActive();
    float hov = Animate(ImGui::GetID("hov"), hovered ? 1.f : 0.f, 18.f);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImGuiStyle& st = ImGui::GetStyle();
    if (hov > 0.01f || held) {
        ImVec4 c = held ? st.Colors[ImGuiCol_ButtonActive] : st.Colors[ImGuiCol_ButtonHovered];
        c.w *= held ? 1.f : hov;
        dl->AddRectFilled(p, ImVec2(p.x + sz.x, p.y + sz.y), ImGui::ColorConvertFloat4ToU32(c), st.FrameRounding);
    }
    unsigned int cp = 0;
    ImTextCharFromUtf8(&cp, icon, nullptr);
    ImFontBaked* fb = ImGui::GetFontBaked();
    const ImFontGlyph* g = fb ? fb->FindGlyph((ImWchar)cp) : nullptr;
    ImVec2 textPos;
    if (g && g->X1 > g->X0) {
        textPos = ImVec2(p.x + sz.x * 0.5f - (g->X0 + g->X1) * 0.5f, p.y + sz.y * 0.5f - (g->Y0 + g->Y1) * 0.5f);
    } else {
        ImVec2 ts = ImGui::CalcTextSize(icon);
        textPos = ImVec2(p.x + (sz.x - ts.x) * 0.5f, p.y + (sz.y - ts.y) * 0.5f);
    }
    textPos = ImVec2(IM_ROUND(textPos.x), IM_ROUND(textPos.y));
    ImVec4 tc = st.Colors[ImGuiCol_Text];
    if (!hovered && !held) tc = Mix(tc, Pal().textDim, 0.35f);
    dl->AddText(textPos, ImGui::ColorConvertFloat4ToU32(tc), icon);
    if (tooltip && ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) ImGui::SetTooltip("%s", tooltip);
    ImGui::PopID();
    return pressed;
}

static bool ColoredButton(const char* label, const ImVec2& size, const ImVec4& base, const ImVec4& hover, const ImVec4& active, ImU32 text) {
    ImGui::PushStyleColor(ImGuiCol_Button, base);
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, hover);
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, active);
    ImGui::PushStyleColor(ImGuiCol_Text, ImGui::ColorConvertU32ToFloat4(text));
    bool r = ImGui::Button(label, size);
    ImGui::PopStyleColor(4);
    return r;
}

bool AccentButton(const char* label, const ImVec2& size) {
    const auto& p = Pal();
    return ColoredButton(label, size, p.accent, p.accentHover, p.accentActive, IM_COL32_WHITE);
}

bool DangerButton(const char* label, const ImVec2& size) {
    const auto& p = Pal();
    ImVec4 b = p.danger;
    b.w = 0.16f;
    ImVec4 h = p.danger;
    h.w = 0.28f;
    ImVec4 a = p.danger;
    a.w = 0.4f;
    return ColoredButton(label, size, b, h, a, Col(p.danger));
}

bool GhostButton(const char* label, const ImVec2& size) {
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0, 0, 0, 0));
    bool r = ImGui::Button(label, size);
    ImGui::PopStyleColor();
    return r;
}

void SectionTitle(const char* icon, const char* title, const char* subtitle) {
    const auto& p = Pal();
    ImGui::PushFont(GetFonts().bold, 0.0f);
    if (icon) {
        ImGui::PushStyleColor(ImGuiCol_Text, p.accent);
        ImGui::TextUnformatted(icon);
        ImGui::PopStyleColor();
        ImGui::SameLine();
    }
    ImGui::TextUnformatted(title);
    ImGui::PopFont();
    if (subtitle) {
        ImGui::SameLine();
        ImGui::PushStyleColor(ImGuiCol_Text, p.textDim);
        ImGui::TextUnformatted(subtitle);
        ImGui::PopStyleColor();
    }
}

void HelpMarker(const char* text) {
    ImGui::SameLine();
    ImGui::PushStyleColor(ImGuiCol_Text, Pal().textFaint);
    ImGui::TextUnformatted(ICON_INFO);
    ImGui::PopStyleColor();
    if (ImGui::BeginItemTooltip()) {
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 32.0f);
        ImGui::TextUnformatted(text);
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }
}

void TextDim(const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    ImGui::PushStyleColor(ImGuiCol_Text, Pal().textDim);
    ImGui::TextV(fmt, ap);
    ImGui::PopStyleColor();
    va_end(ap);
}

void TextColored2(const ImVec4& c, const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    ImGui::PushStyleColor(ImGuiCol_Text, c);
    ImGui::TextV(fmt, ap);
    ImGui::PopStyleColor();
    va_end(ap);
}

void BoldText(const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    ImGui::PushFont(GetFonts().bold, 0.0f);
    ImGui::TextV(fmt, ap);
    ImGui::PopFont();
    va_end(ap);
}

void BigText(float scale, const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    ImGui::PushFont(GetFonts().bold, ImGui::GetStyle().FontSizeBase * BigScale(scale));
    ImGui::TextV(fmt, ap);
    ImGui::PopFont();
    va_end(ap);
}

bool BeginPropTable(const char* id, float labelWidth) {
    if (!ImGui::BeginTable(id, 2, ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_RowBg | ImGuiTableFlags_PadOuterX)) return false;
    float lw = labelWidth > 0 ? labelWidth : ImGui::GetFontSize() * 8.5f;
    ImGui::TableSetupColumn("k", ImGuiTableColumnFlags_WidthFixed, lw);
    ImGui::TableSetupColumn("v", ImGuiTableColumnFlags_WidthStretch);
    return true;
}

void EndPropTable() { ImGui::EndTable(); }

void PropRow(const char* label, const std::string& value, bool copyable, const ImVec4* color) {
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::PushStyleColor(ImGuiCol_Text, Pal().textDim);
    ImGui::TextUnformatted(label);
    ImGui::PopStyleColor();
    ImGui::TableNextColumn();
    if (color) ImGui::PushStyleColor(ImGuiCol_Text, *color);
    ImGui::TextWrapped("%s", value.empty() ? "—" : value.c_str());
    if (color) ImGui::PopStyleColor();
    if (copyable && !value.empty()) {
        if (ImGui::IsItemHovered()) ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
        if (ImGui::IsItemClicked()) {
            sys::CopyToClipboard(value);
            ImGui::SetTooltip("已复制");
        } else if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayNormal)) {
            ImGui::SetTooltip("点击复制");
        }
    }
}

void MetricTile(const char* id, const char* caption, const char* value, const char* unit, const ImVec4& accent, float width, const char* hint) {
    // Compact stat box: caption line + value line, thin accent bar on the left.
    const auto& p = Pal();
    const float fs = ImGui::GetFontSize();
    const ImGuiStyle& st = ImGui::GetStyle();
    float big = fs * 1.3f;
    float h = st.FramePadding.y * 2 + fs + 2 + big + st.FramePadding.y;
    ImGui::PushID(id);
    ImVec2 pos = ImGui::GetCursorScreenPos();
    ImDrawList* dl = ImGui::GetWindowDrawList();
    ImGui::InvisibleButton("tile", ImVec2(width, h));
    bool hov = ImGui::IsItemHovered();
    dl->AddRectFilled(pos, ImVec2(pos.x + width, pos.y + h), Col(p.card), st.ChildRounding);
    dl->AddRect(pos, ImVec2(pos.x + width, pos.y + h), Col(p.border), st.ChildRounding);
    dl->AddRectFilled(ImVec2(pos.x, pos.y), ImVec2(pos.x + 2.0f * st.FontScaleDpi, pos.y + h), Col(accent), st.ChildRounding, ImDrawFlags_RoundCornersLeft);
    float px = st.FramePadding.x + 3 * st.FontScaleDpi;
    float y = pos.y + st.FramePadding.y * 1.5f;
    dl->AddText(ImVec2(pos.x + px, y), Col(p.textDim), caption);
    y += fs + 2;
    ImFont* bold = GetFonts().bold;
    ImVec2 vs = bold->CalcTextSizeA(big, FLT_MAX, 0, value);
    dl->AddText(bold, big, ImVec2(pos.x + px, y), Col(p.text), value);
    // Unit sits on the value's baseline (smaller font, aligned bottoms).
    if (unit && *unit) dl->AddText(ImVec2(pos.x + px + vs.x + fs * 0.3f, y + big - fs - fs * 0.05f), Col(p.textDim), unit);
    if (hint && hov) ImGui::SetTooltip("%s", hint);
    ImGui::PopID();
}

bool BeginCard(const char* id, const ImVec2& size, bool hovered, bool selected, float padding) {
    const auto& p = Pal();
    ImVec4 bg = selected ? p.cardActive : hovered ? p.cardHover : p.card;
    ImGui::PushStyleColor(ImGuiCol_ChildBg, bg);
    ImGui::PushStyleColor(ImGuiCol_Border, selected ? Mix(p.border, p.accent, 0.7f) : p.border);
    float pad = padding >= 0 ? padding : ImGui::GetStyle().WindowPadding.x;
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(pad, pad * 0.8f));
    ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, ImGui::GetStyle().ChildRounding);
    bool open = ImGui::BeginChild(id, size, ImGuiChildFlags_Borders | ImGuiChildFlags_AlwaysUseWindowPadding | (size.y == 0 ? ImGuiChildFlags_AutoResizeY : 0),
                                  ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
    return open;
}

void EndCard() {
    ImGui::EndChild();
    ImGui::PopStyleVar(2);
    ImGui::PopStyleColor(2);
}

void ProgressRing(float radius, float thickness, float progress, const ImVec4& color) {
    ImDrawList* dl = ImGui::GetWindowDrawList();
    ImVec2 p = ImGui::GetCursorScreenPos();
    ImVec2 c(p.x + radius, p.y + radius);
    dl->PathArcTo(c, radius - thickness * 0.5f, 0, IM_PI * 2, 48);
    dl->PathStroke(Col(color, 0.15f), thickness, ImDrawFlags_Closed);
    float start = -IM_PI * 0.5f;
    if (progress < 0) {
        float t = (float)ImGui::GetTime();
        start = t * 4.0f;
        dl->PathArcTo(c, radius - thickness * 0.5f, start, start + IM_PI * (0.6f + 0.4f * std::sin(t * 2.0f)), 32);
    } else {
        dl->PathArcTo(c, radius - thickness * 0.5f, start, start + IM_PI * 2 * std::clamp(progress, 0.f, 1.f), 48);
    }
    dl->PathStroke(Col(color), thickness, 0);
    ImGui::Dummy(ImVec2(radius * 2, radius * 2));
}

void Meter(float fraction, const ImVec4& color, const ImVec2& size) {
    ImDrawList* dl = ImGui::GetWindowDrawList();
    ImVec2 p = ImGui::GetCursorScreenPos();
    float r = size.y * 0.5f;
    dl->AddRectFilled(p, ImVec2(p.x + size.x, p.y + size.y), Col(Pal().textFaint, 0.25f), r);
    float w = size.x * std::clamp(fraction, 0.f, 1.f);
    if (w > 1) dl->AddRectFilled(p, ImVec2(p.x + std::max(w, size.y), p.y + size.y), Col(color), r);
    ImGui::Dummy(size);
}

void SignalBars(int bars, const ImVec4& color, float height) {
    ImDrawList* dl = ImGui::GetWindowDrawList();
    ImVec2 p = ImGui::GetCursorScreenPos();
    float lh = ImGui::GetTextLineHeight();
    float bw = height * 0.22f, gap = height * 0.12f;
    float base = p.y + (lh + height) * 0.5f;
    for (int i = 0; i < 4; ++i) {
        float bh = height * (0.35f + 0.65f * (i / 3.0f));
        ImVec2 a(p.x + i * (bw + gap), base - bh), b(a.x + bw, base);
        dl->AddRectFilled(a, b, i < bars ? Col(color) : Col(Pal().textFaint, 0.3f), bw * 0.3f);
    }
    ImGui::Dummy(ImVec2(4 * (bw + gap), lh));
}

}  // namespace lhp::ui

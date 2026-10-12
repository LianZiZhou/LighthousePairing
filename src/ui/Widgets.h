#pragma once
#include "imgui.h"
#include "ui/Theme.h"

#include <string>
#include <string_view>

namespace lhp::ui {

/// Exponential smoothing animation keyed by ImGui id; returns current value.
float Animate(ImGuiID id, float target, float speed = 12.0f);
float AnimateStr(const char* key, float target, float speed = 12.0f);

/// Pulsing 0..1 value (for "pairing" breathing light).
float Pulse(float hz = 1.0f);

void StatusDot(const ImVec4& color, float radius, bool pulse = false);
/// Optical (glyph-box) vertical centre of CJK text, relative to the top of a text line.
float TextOpticalCenter();
/// Effective scale used by BigText (headings are kept modest).
inline float BigScale(float s) { return 1.0f + (s - 1.0f) * 0.45f; }
/// Small rounded pill with text. Returns its size.
ImVec2 Chip(const char* text, const ImVec4& color, bool filled = false);
void ChipSameLine(const char* text, const ImVec4& color, bool filled = false);

bool IconButton(const char* icon, const char* tooltip, const ImVec2& size = ImVec2(0, 0));
bool AccentButton(const char* label, const ImVec2& size = ImVec2(0, 0));
bool DangerButton(const char* label, const ImVec2& size = ImVec2(0, 0));
bool GhostButton(const char* label, const ImVec2& size = ImVec2(0, 0));

void SectionTitle(const char* icon, const char* title, const char* subtitle = nullptr);
void HelpMarker(const char* text);
void TextDim(const char* fmt, ...);
void TextColored2(const ImVec4& c, const char* fmt, ...);
void BoldText(const char* fmt, ...);
void BigText(float scale, const char* fmt, ...);

/// Two-column property row inside a table (label, value[, copy button]).
void PropRow(const char* label, const std::string& value, bool copyable = false, const ImVec4* color = nullptr);
bool BeginPropTable(const char* id, float labelWidth = 0);
void EndPropTable();

/// Metric tile: big number + caption + optional accent bar.
void MetricTile(const char* id, const char* caption, const char* value, const char* unit, const ImVec4& accent, float width, const char* hint = nullptr);

/// Begin/End a rounded "card" child with background.
bool BeginCard(const char* id, const ImVec2& size, bool hovered = false, bool selected = false, float padding = -1);
void EndCard();

/// Circular progress ring (0..1). If progress < 0 draws an indeterminate spinner.
void ProgressRing(float radius, float thickness, float progress, const ImVec4& color);

/// Simple horizontal bar 0..1
void Meter(float fraction, const ImVec4& color, const ImVec2& size);

/// Signal quality 0..4 bars.
void SignalBars(int bars, const ImVec4& color, float height);

std::string Fmt(const char* fmt, ...);

}  // namespace lhp::ui

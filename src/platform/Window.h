#pragma once
#include <windows.h>

#include <d3d11.h>
#include <dxgi1_2.h>

#include <string>

struct IDCompositionDevice;
struct IDCompositionTarget;
struct IDCompositionVisual;

namespace lhp {

/// Win32 window + D3D11 swap chain. When `mica` is requested the swap chain is a
/// premultiplied-alpha DirectComposition swap chain so the Windows 11 Mica backdrop
/// shows through translucent ImGui backgrounds.
class Window {
public:
    bool Create(const wchar_t* title, int width, int height, bool mica, bool startHidden);
    void Destroy();

    /// Pumps messages; returns false when the app should quit.
    bool PumpMessages();
    /// Blocks until a message arrives or the timeout expires (used to idle when unfocused).
    void WaitForEvents(DWORD timeoutMs);

    bool BeginFrame(const float clearPremultiplied[4]);   // false when minimized/occluded
    void Present(bool vsync);
    /// Save the current back buffer (call after rendering, before Present). Alpha is composited over `bg`.
    bool SaveScreenshot(const std::wstring& bmpPath, const float bg[3]);

    void SetDarkTitleBar(bool dark);
    bool Focused() const { return GetForegroundWindow() == hwnd; }

    HWND hwnd = nullptr;
    ID3D11Device* device = nullptr;
    ID3D11DeviceContext* context = nullptr;
    float dpiScale = 1.0f;
    bool dpiChanged = false;
    bool composition = false;
    bool micaActive = false;
    UINT width = 0, height = 0;

    LRESULT HandleMessage(UINT msg, WPARAM wp, LPARAM lp);

private:
    bool CreateDevice();
    bool CreateSwapChain();
    void CreateRenderTarget();
    void CleanupRenderTarget();

    IDXGISwapChain1* swapChain_ = nullptr;
    ID3D11RenderTargetView* rtv_ = nullptr;
    IDCompositionDevice* dcompDevice_ = nullptr;
    IDCompositionTarget* dcompTarget_ = nullptr;
    IDCompositionVisual* dcompVisual_ = nullptr;
    UINT pendingW_ = 0, pendingH_ = 0;
    bool occluded_ = false;
    bool minimized_ = false;
    bool recreating_ = false;
};

}  // namespace lhp

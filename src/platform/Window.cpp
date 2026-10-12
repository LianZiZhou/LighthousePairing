#include "platform/Window.h"

#include <dcomp.h>
#include <dwmapi.h>
#include <dxgi1_3.h>

#include <cstdio>
#include <fstream>
#include <vector>

#include "imgui.h"
#include "imgui_impl_win32.h"

extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

#ifndef DWMWA_USE_IMMERSIVE_DARK_MODE
#define DWMWA_USE_IMMERSIVE_DARK_MODE 20
#endif
#ifndef DWMWA_SYSTEMBACKDROP_TYPE
#define DWMWA_SYSTEMBACKDROP_TYPE 38
#endif
#ifndef DWMWA_WINDOW_CORNER_PREFERENCE
#define DWMWA_WINDOW_CORNER_PREFERENCE 33
#endif

namespace lhp {

template <typename T>
static void SafeRelease(T*& p) {
    if (p) {
        p->Release();
        p = nullptr;
    }
}

static LRESULT CALLBACK StaticWndProc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp) {
    if (ImGui_ImplWin32_WndProcHandler(hwnd, msg, wp, lp)) return true;
    Window* self = reinterpret_cast<Window*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));
    if (msg == WM_NCCREATE) {
        auto cs = reinterpret_cast<CREATESTRUCTW*>(lp);
        SetWindowLongPtrW(hwnd, GWLP_USERDATA, (LONG_PTR)cs->lpCreateParams);
        self = static_cast<Window*>(cs->lpCreateParams);
        self->hwnd = hwnd;
    }
    if (self) return self->HandleMessage(msg, wp, lp);
    return DefWindowProcW(hwnd, msg, wp, lp);
}

LRESULT Window::HandleMessage(UINT msg, WPARAM wp, LPARAM lp) {
    switch (msg) {
        case WM_SIZE:
            minimized_ = wp == SIZE_MINIMIZED;
            if (!minimized_) {
                pendingW_ = LOWORD(lp);
                pendingH_ = HIWORD(lp);
            }
            return 0;
        case WM_GETMINMAXINFO: {
            auto* mmi = reinterpret_cast<MINMAXINFO*>(lp);
            mmi->ptMinTrackSize.x = (LONG)(900 * dpiScale);
            mmi->ptMinTrackSize.y = (LONG)(560 * dpiScale);
            return 0;
        }
        case WM_DPICHANGED: {
            dpiScale = HIWORD(wp) / 96.0f;
            dpiChanged = true;
            const RECT* r = reinterpret_cast<RECT*>(lp);
            SetWindowPos(hwnd, nullptr, r->left, r->top, r->right - r->left, r->bottom - r->top, SWP_NOZORDER | SWP_NOACTIVATE);
            return 0;
        }
        case WM_SYSCOMMAND:
            if ((wp & 0xfff0) == SC_KEYMENU) return 0;
            break;
        case WM_ERASEBKGND:
            return 1;
        case WM_DESTROY:
            if (!recreating_) PostQuitMessage(0);
            return 0;
    }
    return DefWindowProcW(hwnd, msg, wp, lp);
}

bool Window::Create(const wchar_t* title, int w, int h, bool mica, bool startHidden) {
    ImGui_ImplWin32_EnableDpiAwareness();
    POINT pt{0, 0};
    dpiScale = ImGui_ImplWin32_GetDpiScaleForMonitor(MonitorFromPoint(pt, MONITOR_DEFAULTTOPRIMARY));

    WNDCLASSEXW wc{sizeof wc};
    wc.style = CS_HREDRAW | CS_VREDRAW;
    wc.lpfnWndProc = StaticWndProc;
    wc.hInstance = GetModuleHandleW(nullptr);
    wc.hCursor = LoadCursorW(nullptr, IDC_ARROW);
    wc.hIcon = LoadIconW(wc.hInstance, MAKEINTRESOURCEW(1));
    wc.hIconSm = wc.hIcon;
    wc.lpszClassName = L"LighthousePairingWnd";
    RegisterClassExW(&wc);

    composition = mica;
    DWORD exStyle = composition ? WS_EX_NOREDIRECTIONBITMAP : 0;
    int sw = (int)(w * dpiScale), sh = (int)(h * dpiScale);
    RECT wa{};
    SystemParametersInfoW(SPI_GETWORKAREA, 0, &wa, 0);
    sw = std::min<int>(sw, wa.right - wa.left);
    sh = std::min<int>(sh, wa.bottom - wa.top);
    int x = wa.left + ((wa.right - wa.left) - sw) / 2, y = wa.top + ((wa.bottom - wa.top) - sh) / 2;
    hwnd = CreateWindowExW(exStyle, wc.lpszClassName, title, WS_OVERLAPPEDWINDOW, x, y, sw, sh, nullptr, nullptr, wc.hInstance, this);
    if (!hwnd) return false;
    dpiScale = ImGui_ImplWin32_GetDpiScaleForHwnd(hwnd);

    SetDarkTitleBar(true);
    int corner = 2;  // DWMWCP_ROUND
    DwmSetWindowAttribute(hwnd, DWMWA_WINDOW_CORNER_PREFERENCE, &corner, sizeof corner);
    if (composition) {
        MARGINS m{-1, -1, -1, -1};
        DwmExtendFrameIntoClientArea(hwnd, &m);
        int backdrop = 2;  // DWMSBT_MAINWINDOW (Mica)
        micaActive = SUCCEEDED(DwmSetWindowAttribute(hwnd, DWMWA_SYSTEMBACKDROP_TYPE, &backdrop, sizeof backdrop));
    }

    if (!CreateDevice() || !CreateSwapChain()) {
        // Fall back to a classic swap chain if composition is unavailable.
        if (composition) {
            recreating_ = true;
            DestroyWindow(hwnd);
            hwnd = nullptr;
            Destroy();
            recreating_ = false;
            return Create(title, w, h, false, startHidden);
        }
        return false;
    }
    if (!startHidden) {
        ShowWindow(hwnd, SW_SHOWDEFAULT);
        UpdateWindow(hwnd);
    }
    return true;
}

void Window::SetDarkTitleBar(bool dark) {
    BOOL v = dark ? TRUE : FALSE;
    DwmSetWindowAttribute(hwnd, DWMWA_USE_IMMERSIVE_DARK_MODE, &v, sizeof v);
}

bool Window::CreateDevice() {
    UINT flags = D3D11_CREATE_DEVICE_BGRA_SUPPORT;
    const D3D_FEATURE_LEVEL levels[] = {D3D_FEATURE_LEVEL_11_1, D3D_FEATURE_LEVEL_11_0, D3D_FEATURE_LEVEL_10_0};
    D3D_FEATURE_LEVEL got{};
    HRESULT hr = D3D11CreateDevice(nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr, flags, levels, 3, D3D11_SDK_VERSION, &device, &got, &context);
    if (FAILED(hr)) hr = D3D11CreateDevice(nullptr, D3D_DRIVER_TYPE_WARP, nullptr, flags, levels, 3, D3D11_SDK_VERSION, &device, &got, &context);
    return SUCCEEDED(hr);
}

bool Window::CreateSwapChain() {
    RECT rc;
    GetClientRect(hwnd, &rc);
    width = std::max<UINT>(1, rc.right - rc.left);
    height = std::max<UINT>(1, rc.bottom - rc.top);

    IDXGIDevice* dxgiDevice = nullptr;
    IDXGIAdapter* adapter = nullptr;
    IDXGIFactory2* factory = nullptr;
    device->QueryInterface(IID_PPV_ARGS(&dxgiDevice));
    dxgiDevice->GetAdapter(&adapter);
    adapter->GetParent(IID_PPV_ARGS(&factory));

    DXGI_SWAP_CHAIN_DESC1 d{};
    d.Width = width;
    d.Height = height;
    d.Format = DXGI_FORMAT_B8G8R8A8_UNORM;
    d.SampleDesc.Count = 1;
    d.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
    d.BufferCount = 2;
    d.SwapEffect = DXGI_SWAP_EFFECT_FLIP_SEQUENTIAL;
    d.Scaling = DXGI_SCALING_STRETCH;
    HRESULT hr;
    if (composition) {
        d.AlphaMode = DXGI_ALPHA_MODE_PREMULTIPLIED;
        hr = factory->CreateSwapChainForComposition(device, &d, nullptr, &swapChain_);
        if (SUCCEEDED(hr)) hr = DCompositionCreateDevice(dxgiDevice, IID_PPV_ARGS(&dcompDevice_));
        if (SUCCEEDED(hr)) hr = dcompDevice_->CreateTargetForHwnd(hwnd, TRUE, &dcompTarget_);
        if (SUCCEEDED(hr)) hr = dcompDevice_->CreateVisual(&dcompVisual_);
        if (SUCCEEDED(hr)) hr = dcompVisual_->SetContent(swapChain_);
        if (SUCCEEDED(hr)) hr = dcompTarget_->SetRoot(dcompVisual_);
        if (SUCCEEDED(hr)) hr = dcompDevice_->Commit();
    } else {
        d.AlphaMode = DXGI_ALPHA_MODE_IGNORE;
        d.SwapEffect = DXGI_SWAP_EFFECT_FLIP_DISCARD;
        hr = factory->CreateSwapChainForHwnd(device, hwnd, &d, nullptr, nullptr, &swapChain_);
    }
    if (SUCCEEDED(hr)) factory->MakeWindowAssociation(hwnd, DXGI_MWA_NO_ALT_ENTER);
    SafeRelease(factory);
    SafeRelease(adapter);
    SafeRelease(dxgiDevice);
    if (FAILED(hr)) {
        SafeRelease(dcompVisual_);
        SafeRelease(dcompTarget_);
        SafeRelease(dcompDevice_);
        SafeRelease(swapChain_);
        return false;
    }
    CreateRenderTarget();
    return true;
}

void Window::CreateRenderTarget() {
    ID3D11Texture2D* back = nullptr;
    swapChain_->GetBuffer(0, IID_PPV_ARGS(&back));
    device->CreateRenderTargetView(back, nullptr, &rtv_);
    back->Release();
}

void Window::CleanupRenderTarget() { SafeRelease(rtv_); }

void Window::Destroy() {
    CleanupRenderTarget();
    SafeRelease(dcompVisual_);
    SafeRelease(dcompTarget_);
    SafeRelease(dcompDevice_);
    SafeRelease(swapChain_);
    SafeRelease(context);
    SafeRelease(device);
    if (hwnd) {
        DestroyWindow(hwnd);
        hwnd = nullptr;
    }
}

bool Window::PumpMessages() {
    MSG msg;
    while (PeekMessageW(&msg, nullptr, 0, 0, PM_REMOVE)) {
        TranslateMessage(&msg);
        DispatchMessageW(&msg);
        if (msg.message == WM_QUIT) return false;
    }
    return true;
}

void Window::WaitForEvents(DWORD timeoutMs) { MsgWaitForMultipleObjectsEx(0, nullptr, timeoutMs, QS_ALLINPUT, MWMO_INPUTAVAILABLE); }

bool Window::BeginFrame(const float clear[4]) {
    if (minimized_) return false;
    if (occluded_ && swapChain_->Present(0, DXGI_PRESENT_TEST) == DXGI_STATUS_OCCLUDED) return false;
    occluded_ = false;
    if (pendingW_ && pendingH_ && (pendingW_ != width || pendingH_ != height)) {
        CleanupRenderTarget();
        swapChain_->ResizeBuffers(0, pendingW_, pendingH_, DXGI_FORMAT_UNKNOWN, 0);
        width = pendingW_;
        height = pendingH_;
        CreateRenderTarget();
    }
    pendingW_ = pendingH_ = 0;
    context->OMSetRenderTargets(1, &rtv_, nullptr);
    context->ClearRenderTargetView(rtv_, clear);
    return true;
}

void Window::Present(bool vsync) {
    HRESULT hr = swapChain_->Present(vsync ? 1 : 0, 0);
    occluded_ = hr == DXGI_STATUS_OCCLUDED;
}

bool Window::SaveScreenshot(const std::wstring& path, const float bg[3]) {
    ID3D11Texture2D* back = nullptr;
    if (FAILED(swapChain_->GetBuffer(0, IID_PPV_ARGS(&back)))) return false;
    D3D11_TEXTURE2D_DESC td;
    back->GetDesc(&td);
    td.Usage = D3D11_USAGE_STAGING;
    td.BindFlags = 0;
    td.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
    td.MiscFlags = 0;
    ID3D11Texture2D* staging = nullptr;
    if (FAILED(device->CreateTexture2D(&td, nullptr, &staging))) {
        back->Release();
        return false;
    }
    context->CopyResource(staging, back);
    back->Release();
    D3D11_MAPPED_SUBRESOURCE map{};
    if (FAILED(context->Map(staging, 0, D3D11_MAP_READ, 0, &map))) {
        staging->Release();
        return false;
    }
    const UINT w = td.Width, h = td.Height;
    std::vector<uint8_t> px((size_t)w * h * 3);
    for (UINT y = 0; y < h; ++y) {
        const uint8_t* row = static_cast<const uint8_t*>(map.pData) + (size_t)y * map.RowPitch;
        uint8_t* dst = px.data() + (size_t)(h - 1 - y) * w * 3;   // BMP bottom-up
        for (UINT x = 0; x < w; ++x) {
            float a = row[x * 4 + 3] / 255.f;
            // B G R premultiplied
            for (int c = 0; c < 3; ++c) {
                float bgc = bg[2 - c] * 255.f;
                float v = composition ? row[x * 4 + c] + bgc * (1.f - a) : row[x * 4 + c];
                dst[x * 3 + c] = (uint8_t)std::min(255.f, v);
            }
        }
    }
    context->Unmap(staging, 0);
    staging->Release();

    UINT rowBytes = (w * 3 + 3) & ~3u;
    BITMAPFILEHEADER fh{};
    BITMAPINFOHEADER ih{};
    ih.biSize = sizeof ih;
    ih.biWidth = (LONG)w;
    ih.biHeight = (LONG)h;
    ih.biPlanes = 1;
    ih.biBitCount = 24;
    ih.biSizeImage = rowBytes * h;
    fh.bfType = 0x4D42;
    fh.bfOffBits = sizeof fh + sizeof ih;
    fh.bfSize = fh.bfOffBits + ih.biSizeImage;
    std::ofstream f(path, std::ios::binary);
    if (!f) return false;
    f.write(reinterpret_cast<char*>(&fh), sizeof fh);
    f.write(reinterpret_cast<char*>(&ih), sizeof ih);
    std::vector<uint8_t> pad(rowBytes - w * 3, 0);
    for (UINT y = 0; y < h; ++y) {
        f.write(reinterpret_cast<char*>(px.data() + (size_t)y * w * 3), w * 3);
        if (!pad.empty()) f.write(reinterpret_cast<char*>(pad.data()), pad.size());
    }
    return (bool)f;
}

}  // namespace lhp

#include "platform/Renderer3D.h"

#include <d3dcompiler.h>

#include <cmath>
#include <cstring>

namespace lhp::gfx {

namespace {

template <typename T>
void SafeRelease(T*& p) {
    if (p) {
        p->Release();
        p = nullptr;
    }
}

const char* kShader = R"(
cbuffer Frame : register(b0) {
    row_major float4x4 viewProj;
    float4 lightDir;
    float4 litFlag;   // x = 1 for lit triangles
};
struct VSIn { float3 pos : POSITION; float3 nrm : NORMAL; float4 col : COLOR; };
struct PSIn { float4 pos : SV_POSITION; float3 nrm : NORMAL; float4 col : COLOR; };
PSIn VS(VSIn v) {
    PSIn o;
    o.pos = mul(float4(v.pos, 1), viewProj);
    o.nrm = v.nrm;
    o.col = v.col;
    return o;
}
float4 PS(PSIn i) : SV_Target {
    float3 c = i.col.rgb;
    if (litFlag.x > 0.5) {
        float d = abs(dot(normalize(i.nrm), lightDir.xyz));
        c *= 0.45 + 0.55 * d;
    }
    return float4(c * i.col.a, i.col.a);   // premultiplied
}
)";

struct FrameCB {
    float viewProj[16];
    float lightDir[4];
    float lit[4];
};

}  // namespace

V3 Cross(const V3& a, const V3& b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
float Dot(const V3& a, const V3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
V3 Normalize(const V3& a) {
    float n = std::sqrt(Dot(a, a));
    return n > 0 ? a * (1.0f / n) : V3{};
}

Mat4 Mat4::Identity() {
    Mat4 r;
    for (int i = 0; i < 4; ++i) r.m[i][i] = 1;
    return r;
}

Mat4 Mat4::LookAt(const V3& eye, const V3& target, const V3& up) {
    V3 z = Normalize(eye - target);   // camera looks down -z
    V3 x = Normalize(Cross(up, z));
    V3 y = Cross(z, x);
    Mat4 r = Identity();
    r.m[0][0] = x.x, r.m[1][0] = x.y, r.m[2][0] = x.z;
    r.m[0][1] = y.x, r.m[1][1] = y.y, r.m[2][1] = y.z;
    r.m[0][2] = z.x, r.m[1][2] = z.y, r.m[2][2] = z.z;
    r.m[3][0] = -Dot(x, eye), r.m[3][1] = -Dot(y, eye), r.m[3][2] = -Dot(z, eye);
    return r;
}

Mat4 Mat4::Perspective(float fovY, float aspect, float zn, float zf) {
    float f = 1.0f / std::tan(fovY * 0.5f);
    Mat4 r;
    r.m[0][0] = f / aspect;
    r.m[1][1] = f;
    r.m[2][2] = zf / (zn - zf);
    r.m[2][3] = -1;
    r.m[3][2] = zn * zf / (zn - zf);
    return r;
}

Mat4 Mat4::operator*(const Mat4& b) const {
    Mat4 r;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            for (int k = 0; k < 4; ++k) r.m[i][j] += m[i][k] * b.m[k][j];
    return r;
}

V3 OrbitCamera::Eye() const {
    return {target.x + distance * std::cos(pitch) * std::sin(yaw), target.y + distance * std::sin(pitch), target.z + distance * std::cos(pitch) * std::cos(yaw)};
}

Renderer3D::~Renderer3D() { Shutdown(); }

bool Renderer3D::Init(ID3D11Device* device, ID3D11DeviceContext* ctx) {
    dev_ = device;
    ctx_ = ctx;
    ID3DBlob *vsb = nullptr, *psb = nullptr, *err = nullptr;
    if (FAILED(D3DCompile(kShader, std::strlen(kShader), "r3d", nullptr, nullptr, "VS", "vs_4_0", 0, 0, &vsb, &err))) {
        SafeRelease(err);
        return false;
    }
    if (FAILED(D3DCompile(kShader, std::strlen(kShader), "r3d", nullptr, nullptr, "PS", "ps_4_0", 0, 0, &psb, &err))) {
        SafeRelease(err);
        SafeRelease(vsb);
        return false;
    }
    dev_->CreateVertexShader(vsb->GetBufferPointer(), vsb->GetBufferSize(), nullptr, &vs_);
    dev_->CreatePixelShader(psb->GetBufferPointer(), psb->GetBufferSize(), nullptr, &ps_);
    D3D11_INPUT_ELEMENT_DESC il[] = {
        {"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0},
        {"NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 12, D3D11_INPUT_PER_VERTEX_DATA, 0},
        {"COLOR", 0, DXGI_FORMAT_R8G8B8A8_UNORM, 0, 24, D3D11_INPUT_PER_VERTEX_DATA, 0},
    };
    dev_->CreateInputLayout(il, 3, vsb->GetBufferPointer(), vsb->GetBufferSize(), &layout_);
    SafeRelease(vsb);
    SafeRelease(psb);

    D3D11_BUFFER_DESC cbd{sizeof(FrameCB), D3D11_USAGE_DYNAMIC, D3D11_BIND_CONSTANT_BUFFER, D3D11_CPU_ACCESS_WRITE, 0, 0};
    dev_->CreateBuffer(&cbd, nullptr, &cb_);

    D3D11_RASTERIZER_DESC rd{};
    rd.FillMode = D3D11_FILL_SOLID;
    rd.CullMode = D3D11_CULL_NONE;
    rd.DepthClipEnable = TRUE;
    rd.MultisampleEnable = TRUE;
    rd.AntialiasedLineEnable = TRUE;
    dev_->CreateRasterizerState(&rd, &rs_);

    D3D11_BLEND_DESC bd{};
    bd.RenderTarget[0].BlendEnable = TRUE;
    bd.RenderTarget[0].SrcBlend = D3D11_BLEND_ONE;
    bd.RenderTarget[0].DestBlend = D3D11_BLEND_INV_SRC_ALPHA;
    bd.RenderTarget[0].BlendOp = D3D11_BLEND_OP_ADD;
    bd.RenderTarget[0].SrcBlendAlpha = D3D11_BLEND_ONE;
    bd.RenderTarget[0].DestBlendAlpha = D3D11_BLEND_INV_SRC_ALPHA;
    bd.RenderTarget[0].BlendOpAlpha = D3D11_BLEND_OP_ADD;
    bd.RenderTarget[0].RenderTargetWriteMask = D3D11_COLOR_WRITE_ENABLE_ALL;
    dev_->CreateBlendState(&bd, &blend_);

    D3D11_DEPTH_STENCIL_DESC dd{};
    dd.DepthEnable = TRUE;
    dd.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ALL;
    dd.DepthFunc = D3D11_COMPARISON_LESS_EQUAL;
    dev_->CreateDepthStencilState(&dd, &depth_);
    dd.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ZERO;
    dev_->CreateDepthStencilState(&dd, &depthNoWrite_);

    UINT q = 0;
    if (FAILED(dev_->CheckMultisampleQualityLevels(DXGI_FORMAT_R8G8B8A8_UNORM, 4, &q)) || q == 0) samples_ = 1;
    return vs_ && ps_ && layout_ && cb_;
}

void Renderer3D::Shutdown() {
    ReleaseTargets();
    SafeRelease(vs_);
    SafeRelease(ps_);
    SafeRelease(layout_);
    SafeRelease(cb_);
    SafeRelease(vb_);
    SafeRelease(rs_);
    SafeRelease(blend_);
    SafeRelease(depth_);
    SafeRelease(depthNoWrite_);
    vbCapacity_ = 0;
}

void Renderer3D::ReleaseTargets() {
    SafeRelease(msaaRtv_);
    SafeRelease(msaaTex_);
    SafeRelease(dsv_);
    SafeRelease(depthTex_);
    SafeRelease(srv_);
    SafeRelease(resolveTex_);
    w_ = h_ = 0;
}

bool Renderer3D::CreateTargets(int w, int h) {
    ReleaseTargets();
    D3D11_TEXTURE2D_DESC td{};
    td.Width = (UINT)w;
    td.Height = (UINT)h;
    td.MipLevels = 1;
    td.ArraySize = 1;
    td.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    td.SampleDesc.Count = samples_;
    td.Usage = D3D11_USAGE_DEFAULT;
    td.BindFlags = D3D11_BIND_RENDER_TARGET;
    if (FAILED(dev_->CreateTexture2D(&td, nullptr, &msaaTex_))) return false;
    dev_->CreateRenderTargetView(msaaTex_, nullptr, &msaaRtv_);
    td.Format = DXGI_FORMAT_D24_UNORM_S8_UINT;
    td.BindFlags = D3D11_BIND_DEPTH_STENCIL;
    if (FAILED(dev_->CreateTexture2D(&td, nullptr, &depthTex_))) return false;
    dev_->CreateDepthStencilView(depthTex_, nullptr, &dsv_);
    td.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    td.SampleDesc.Count = 1;
    td.BindFlags = D3D11_BIND_SHADER_RESOURCE;
    if (FAILED(dev_->CreateTexture2D(&td, nullptr, &resolveTex_))) return false;
    dev_->CreateShaderResourceView(resolveTex_, nullptr, &srv_);
    w_ = w;
    h_ = h;
    return true;
}

void Renderer3D::Begin(int width, int height, const OrbitCamera& cam, const float clear[4]) {
    width = width < 8 ? 8 : width;
    height = height < 8 ? 8 : height;
    if (width != w_ || height != h_) CreateTargets(width, height);
    lines_.clear();
    tris_.clear();
    alphaTris_.clear();
    eye_ = cam.Eye();
    Mat4 view = Mat4::LookAt(eye_, cam.target, {0, 1, 0});
    Mat4 proj = Mat4::Perspective(cam.fovY, (float)w_ / (float)h_, 0.02f, 200.0f);
    viewProj_ = view * proj;
    if (msaaRtv_) ctx_->ClearRenderTargetView(msaaRtv_, clear);
    if (dsv_) ctx_->ClearDepthStencilView(dsv_, D3D11_CLEAR_DEPTH, 1.0f, 0);
}

void Renderer3D::Line(const V3& a, const V3& b, Color c) {
    lines_.push_back({a.x, a.y, a.z, 0, 1, 0, c});
    lines_.push_back({b.x, b.y, b.z, 0, 1, 0, c});
}

void Renderer3D::Triangle(const V3& a, const V3& b, const V3& c, Color col) {
    V3 n = Normalize(Cross(b - a, c - a));
    auto& dst = (col >> 24) < 250 ? alphaTris_ : tris_;
    dst.push_back({a.x, a.y, a.z, n.x, n.y, n.z, col});
    dst.push_back({b.x, b.y, b.z, n.x, n.y, n.z, col});
    dst.push_back({c.x, c.y, c.z, n.x, n.y, n.z, col});
}

void Renderer3D::Box(const V3& c, const V3& ax, const V3& ay, const V3& az, Color col) {
    V3 p[8];
    for (int i = 0; i < 8; ++i) p[i] = c + ax * ((i & 1) ? 1.f : -1.f) + ay * ((i & 2) ? 1.f : -1.f) + az * ((i & 4) ? 1.f : -1.f);
    const int f[6][4] = {{0, 2, 3, 1}, {4, 5, 7, 6}, {0, 1, 5, 4}, {2, 6, 7, 3}, {0, 4, 6, 2}, {1, 3, 7, 5}};
    for (auto& q : f) {
        Triangle(p[q[0]], p[q[1]], p[q[2]], col);
        Triangle(p[q[0]], p[q[2]], p[q[3]], col);
    }
}

void Renderer3D::WireBox(const V3& c, const V3& ax, const V3& ay, const V3& az, Color col) {
    V3 p[8];
    for (int i = 0; i < 8; ++i) p[i] = c + ax * ((i & 1) ? 1.f : -1.f) + ay * ((i & 2) ? 1.f : -1.f) + az * ((i & 4) ? 1.f : -1.f);
    const int e[12][2] = {{0, 1}, {2, 3}, {4, 5}, {6, 7}, {0, 2}, {1, 3}, {4, 6}, {5, 7}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
    for (auto& k : e) Line(p[k[0]], p[k[1]], col);
}

void Renderer3D::Grid(float half, float step, Color minor, Color major, int majorEvery) {
    int n = (int)std::round(half / step);
    for (int i = -n; i <= n; ++i) {
        float v = i * step;
        Color c = (i % majorEvery == 0) ? major : minor;
        Line({v, 0, -half}, {v, 0, half}, c);
        Line({-half, 0, v}, {half, 0, v}, c);
    }
}

void Renderer3D::Axes(const V3& o, const V3& ax, const V3& ay, const V3& az, float len) {
    Line(o, o + ax * len, Rgba(235, 80, 80));
    Line(o, o + ay * len, Rgba(90, 210, 110));
    Line(o, o + az * len, Rgba(90, 140, 245));
}

bool Renderer3D::Project(const V3& p, float& x, float& y) const {
    const auto& m = viewProj_.m;
    float cx = p.x * m[0][0] + p.y * m[1][0] + p.z * m[2][0] + m[3][0];
    float cy = p.x * m[0][1] + p.y * m[1][1] + p.z * m[2][1] + m[3][1];
    float cw = p.x * m[0][3] + p.y * m[1][3] + p.z * m[2][3] + m[3][3];
    if (cw <= 1e-4f) return false;
    x = (cx / cw * 0.5f + 0.5f) * w_;
    y = (1.0f - (cy / cw * 0.5f + 0.5f)) * h_;
    return true;
}

void Renderer3D::Flush(std::vector<Vertex>& v, D3D11_PRIMITIVE_TOPOLOGY topo, bool lit) {
    if (v.empty()) return;
    if (v.size() > vbCapacity_) {
        SafeRelease(vb_);
        vbCapacity_ = v.size() + v.size() / 2 + 1024;
        D3D11_BUFFER_DESC bd{(UINT)(vbCapacity_ * sizeof(Vertex)), D3D11_USAGE_DYNAMIC, D3D11_BIND_VERTEX_BUFFER, D3D11_CPU_ACCESS_WRITE, 0, 0};
        if (FAILED(dev_->CreateBuffer(&bd, nullptr, &vb_))) {
            vbCapacity_ = 0;
            return;
        }
    }
    D3D11_MAPPED_SUBRESOURCE ms;
    if (FAILED(ctx_->Map(vb_, 0, D3D11_MAP_WRITE_DISCARD, 0, &ms))) return;
    std::memcpy(ms.pData, v.data(), v.size() * sizeof(Vertex));
    ctx_->Unmap(vb_, 0);
    if (SUCCEEDED(ctx_->Map(cb_, 0, D3D11_MAP_WRITE_DISCARD, 0, &ms))) {
        FrameCB* f = (FrameCB*)ms.pData;
        std::memcpy(f->viewProj, viewProj_.m, sizeof f->viewProj);
        V3 l = Normalize({0.35f, 0.85f, 0.4f});
        f->lightDir[0] = l.x, f->lightDir[1] = l.y, f->lightDir[2] = l.z, f->lightDir[3] = 0;
        f->lit[0] = lit ? 1.f : 0.f;
        ctx_->Unmap(cb_, 0);
    }
    UINT stride = sizeof(Vertex), offset = 0;
    ctx_->IASetVertexBuffers(0, 1, &vb_, &stride, &offset);
    ctx_->IASetPrimitiveTopology(topo);
    ctx_->Draw((UINT)v.size(), 0);
}

void Renderer3D::End() {
    if (!msaaRtv_) return;
    prevVpCount_ = 1;
    ctx_->RSGetViewports(&prevVpCount_, &prevVp_);
    ctx_->OMGetRenderTargets(1, &prevRtv_, &prevDsv_);

    D3D11_VIEWPORT vp{0, 0, (float)w_, (float)h_, 0, 1};
    ctx_->RSSetViewports(1, &vp);
    ctx_->OMSetRenderTargets(1, &msaaRtv_, dsv_);
    ctx_->IASetInputLayout(layout_);
    ctx_->VSSetShader(vs_, nullptr, 0);
    ctx_->PSSetShader(ps_, nullptr, 0);
    ctx_->VSSetConstantBuffers(0, 1, &cb_);
    ctx_->PSSetConstantBuffers(0, 1, &cb_);
    ctx_->GSSetShader(nullptr, nullptr, 0);
    ctx_->RSSetState(rs_);
    const float bf[4] = {0, 0, 0, 0};
    ctx_->OMSetBlendState(blend_, bf, 0xFFFFFFFF);
    ctx_->OMSetDepthStencilState(depth_, 0);
    Flush(tris_, D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST, true);
    Flush(lines_, D3D11_PRIMITIVE_TOPOLOGY_LINELIST, false);
    ctx_->OMSetDepthStencilState(depthNoWrite_, 0);
    Flush(alphaTris_, D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST, false);

    if (samples_ > 1) ctx_->ResolveSubresource(resolveTex_, 0, msaaTex_, 0, DXGI_FORMAT_R8G8B8A8_UNORM);
    else ctx_->CopyResource(resolveTex_, msaaTex_);

    ctx_->OMSetRenderTargets(1, &prevRtv_, prevDsv_);
    if (prevVpCount_) ctx_->RSSetViewports(prevVpCount_, &prevVp_);
    SafeRelease(prevRtv_);
    SafeRelease(prevDsv_);
}

}  // namespace lhp::gfx

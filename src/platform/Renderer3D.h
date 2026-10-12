#pragma once
// Minimal immediate-mode 3D renderer on D3D11, drawing into an offscreen MSAA texture that ImGui shows
// as an image. Knows nothing about tracking: it draws lines, triangles and simple shapes.
#include <d3d11.h>

#include <cstdint>
#include <vector>

namespace lhp::gfx {

struct V3 {
    float x = 0, y = 0, z = 0;
    V3() = default;
    constexpr V3(float X, float Y, float Z) : x(X), y(Y), z(Z) {}
    V3 operator+(const V3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    V3 operator-(const V3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    V3 operator*(float s) const { return {x * s, y * s, z * s}; }
};
V3 Cross(const V3& a, const V3& b);
float Dot(const V3& a, const V3& b);
V3 Normalize(const V3& a);

/// Row-major 4x4, row vectors (v' = v * M), matching HLSL mul(v, M) with row_major.
struct Mat4 {
    float m[4][4]{};
    static Mat4 Identity();
    static Mat4 LookAt(const V3& eye, const V3& target, const V3& up);   // right-handed
    static Mat4 Perspective(float fovY, float aspect, float zn, float zf);
    Mat4 operator*(const Mat4& b) const;
};

/// RGBA colour packed as 0xAABBGGRR (same as ImGui's IM_COL32).
using Color = uint32_t;
constexpr Color Rgba(uint8_t r, uint8_t g, uint8_t b, uint8_t a = 255) { return (Color)r | ((Color)g << 8) | ((Color)b << 16) | ((Color)a << 24); }

struct OrbitCamera {
    V3 target{0, 1, 0};
    float yaw = 0.6f, pitch = 0.45f, distance = 5.0f;
    float fovY = 0.9f;
    V3 Eye() const;
};

class Renderer3D {
public:
    ~Renderer3D();
    bool Init(ID3D11Device* device, ID3D11DeviceContext* ctx);
    void Shutdown();

    /// Starts a frame of the given pixel size (recreates targets on resize).
    void Begin(int width, int height, const OrbitCamera& cam, const float clear[4]);
    /// Renders everything queued since Begin into the texture and restores the caller's render target.
    void End();

    // ---- primitives (world space)
    void Line(const V3& a, const V3& b, Color c);
    void Triangle(const V3& a, const V3& b, const V3& c, Color col);
    /// Oriented box: centre, local axes (unit, scaled by half extents), colour.
    void Box(const V3& centre, const V3& ax, const V3& ay, const V3& az, Color col);
    void WireBox(const V3& centre, const V3& ax, const V3& ay, const V3& az, Color col);
    void Cube(const V3& centre, float half, Color col) { Box(centre, {half, 0, 0}, {0, half, 0}, {0, 0, half}, col); }
    void Grid(float halfSize, float step, Color minor, Color major, int majorEvery);
    void Axes(const V3& origin, const V3& ax, const V3& ay, const V3& az, float len);

    /// World -> pixel position in the texture (false when behind the camera).
    bool Project(const V3& p, float& x, float& y) const;
    ID3D11ShaderResourceView* Texture() const { return srv_; }
    int Width() const { return w_; }
    int Height() const { return h_; }
    const Mat4& ViewProj() const { return viewProj_; }

private:
    struct Vertex {
        float px, py, pz;
        float nx, ny, nz;
        uint32_t color;
    };
    bool CreateTargets(int w, int h);
    void ReleaseTargets();
    void Flush(std::vector<Vertex>& verts, D3D11_PRIMITIVE_TOPOLOGY topo, bool lit);

    ID3D11Device* dev_ = nullptr;
    ID3D11DeviceContext* ctx_ = nullptr;
    ID3D11Texture2D* msaaTex_ = nullptr;
    ID3D11RenderTargetView* msaaRtv_ = nullptr;
    ID3D11Texture2D* depthTex_ = nullptr;
    ID3D11DepthStencilView* dsv_ = nullptr;
    ID3D11Texture2D* resolveTex_ = nullptr;
    ID3D11ShaderResourceView* srv_ = nullptr;
    ID3D11VertexShader* vs_ = nullptr;
    ID3D11PixelShader* ps_ = nullptr;
    ID3D11InputLayout* layout_ = nullptr;
    ID3D11Buffer* cb_ = nullptr;
    ID3D11Buffer* vb_ = nullptr;
    size_t vbCapacity_ = 0;
    ID3D11RasterizerState* rs_ = nullptr;
    ID3D11BlendState* blend_ = nullptr;
    ID3D11DepthStencilState* depth_ = nullptr;
    ID3D11DepthStencilState* depthNoWrite_ = nullptr;
    int w_ = 0, h_ = 0;
    UINT samples_ = 4;
    Mat4 viewProj_;
    V3 eye_;
    std::vector<Vertex> lines_, tris_, alphaTris_;
    // saved caller state
    ID3D11RenderTargetView* prevRtv_ = nullptr;
    ID3D11DepthStencilView* prevDsv_ = nullptr;
    D3D11_VIEWPORT prevVp_{};
    UINT prevVpCount_ = 0;
};

}  // namespace lhp::gfx

#pragma once

#include <windows.h>
#include <d3d11.h>
#include <DirectXMath.h>
#include <wrl/client.h>
#include <vector>

#include "../physics/RigidBody.h"
#include "../collision/Contact.h"

class D3D11Renderer
{
public:
    // The client area must already match width x height.
    bool Initialize(HWND window, unsigned width, unsigned height);

    // Orbit camera + mouse picking share the same view/projection matrices.
    void SetOrbitCamera(float yawRadians, float pitchRadians, float distance, float targetHeight = 1.2f);
    bool ScreenPointToRay(int pixelX, int pixelY,
                          PE::Vector3& origin,
                          PE::Vector3& direction) const;

    // Call in this order: BeginFrame -> DrawBox / debug draw -> EndFrame.
    void BeginFrame();
    void DrawBox(const PE::RigidBody& body, const PE::Vector3& halfSize);
    void DrawSphere(const PE::RigidBody& body, PE::Real radius);

    // Lines are batched and submitted at EndFrame.
    // overlay=true draws through scene geometry (useful for contacts/axes).
    void DrawLine(const PE::Vector3& start, const PE::Vector3& end,
                  float red, float green, float blue, bool overlay = false);
    void DrawGrid(int halfCells = 5, float spacing = 1.0f);
    void DrawBodyAxes(const PE::RigidBody& body, float axisLength = 0.9f);
    void DrawContact(const PE::Contact& contact, float size = 0.12f);

    bool EndFrame();

private:
    struct DebugVertex
    {
        float x, y, z;
        float r, g, b;
    };

    bool FlushLines(const std::vector<DebugVertex>& vertices, bool overlay);
    std::vector<DebugVertex> depthLines_;
    std::vector<DebugVertex> overlayLines_;
    Microsoft::WRL::ComPtr<ID3D11Buffer> debugVertexBuffer_;
    Microsoft::WRL::ComPtr<ID3D11DepthStencilState> noDepthState_;

    Microsoft::WRL::ComPtr<ID3D11Device> device_;
    Microsoft::WRL::ComPtr<ID3D11DeviceContext> context_;
    Microsoft::WRL::ComPtr<IDXGISwapChain> swapChain_;
    Microsoft::WRL::ComPtr<ID3D11RenderTargetView> renderTarget_;
    Microsoft::WRL::ComPtr<ID3D11Texture2D> depthTexture_;
    Microsoft::WRL::ComPtr<ID3D11DepthStencilView> depthView_;
    Microsoft::WRL::ComPtr<ID3D11VertexShader> vertexShader_;
    Microsoft::WRL::ComPtr<ID3D11PixelShader> pixelShader_;
    Microsoft::WRL::ComPtr<ID3D11InputLayout> inputLayout_;
    Microsoft::WRL::ComPtr<ID3D11Buffer> vertexBuffer_;
    Microsoft::WRL::ComPtr<ID3D11Buffer> indexBuffer_;
    Microsoft::WRL::ComPtr<ID3D11Buffer> sphereVertexBuffer_;
    Microsoft::WRL::ComPtr<ID3D11Buffer> sphereIndexBuffer_;
    UINT sphereIndexCount_ = 0;
    Microsoft::WRL::ComPtr<ID3D11Buffer> constantBuffer_;
    Microsoft::WRL::ComPtr<ID3D11RasterizerState> rasterState_;
    Microsoft::WRL::ComPtr<ID3D11DepthStencilState> depthState_;
    DirectX::XMFLOAT4X4 viewProjection_{};
    DirectX::XMFLOAT4X4 viewMatrix_{};
    DirectX::XMFLOAT4X4 projectionMatrix_{};
    DirectX::XMFLOAT3 cameraPosition_{};
    D3D11_VIEWPORT viewport_{};
};

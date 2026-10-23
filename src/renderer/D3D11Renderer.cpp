#include "D3D11Renderer.h"

// Adjust this include only if your RigidBody.h lives elsewhere.
#include "../physics/RigidBody.h"

#include <d3dcompiler.h>
#include <cstring>
#include <cmath>
#include <vector>

#pragma comment(lib, "d3d11.lib")
#pragma comment(lib, "dxgi.lib")
#pragma comment(lib, "d3dcompiler.lib")

using Microsoft::WRL::ComPtr;
using namespace DirectX;

namespace
{
    constexpr UINT kMaxDebugVertices = 8192;

    struct Vertex
    {
        XMFLOAT3 position;
        XMFLOAT3 color;
    };

    struct TransformBuffer
    {
        XMFLOAT4X4 worldViewProjection;
    };

    constexpr char kShaderSource[] = R"(
        cbuffer Transform : register(b0)
        {
            float4x4 worldViewProjection;
        };

        struct VSInput
        {
            float3 position : POSITION;
            float3 color : COLOR;
        };

        struct VSOutput
        {
            float4 position : SV_POSITION;
            float3 color : COLOR;
        };

        VSOutput VSMain(VSInput input)
        {
            VSOutput output;
            output.position = mul(float4(input.position, 1.0f),
                                  worldViewProjection);
            output.color = input.color;
            return output;
        }

        float4 PSMain(VSOutput input) : SV_TARGET
        {
            return float4(input.color, 1.0f);
        }
    )";

    bool CompileShader(
        const char* entryPoint,
        const char* target,
        ComPtr<ID3DBlob>& byteCode)
    {
        ComPtr<ID3DBlob> errors;
        const HRESULT hr = D3DCompile(
            kShaderSource,
            std::strlen(kShaderSource),
            nullptr,
            nullptr,
            nullptr,
            entryPoint,
            target,
            0,
            0,
            byteCode.GetAddressOf(),
            errors.GetAddressOf());

        if (FAILED(hr))
        {
            if (errors)
            {
                MessageBoxA(
                    nullptr,
                    static_cast<const char*>(errors->GetBufferPointer()),
                    "DX11 Shader Error",
                    MB_OK | MB_ICONERROR);
            }
            return false;
        }
        return true;
    }
}

bool D3D11Renderer::Initialize(HWND window, unsigned width, unsigned height)
{
    DXGI_SWAP_CHAIN_DESC desc = {};
    desc.BufferCount = 1;
    desc.BufferDesc.Width = width;
    desc.BufferDesc.Height = height;
    desc.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    desc.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
    desc.OutputWindow = window;
    desc.SampleDesc.Count = 1;
    desc.Windowed = TRUE;
    desc.SwapEffect = DXGI_SWAP_EFFECT_DISCARD;

    HRESULT hr = D3D11CreateDeviceAndSwapChain(
        nullptr,
        D3D_DRIVER_TYPE_HARDWARE,
        nullptr,
        0,
        nullptr,
        0,
        D3D11_SDK_VERSION,
        &desc,
        swapChain_.GetAddressOf(),
        device_.GetAddressOf(),
        nullptr,
        context_.GetAddressOf());
    if (FAILED(hr)) return false;

    ComPtr<ID3D11Texture2D> backBuffer;
    hr = swapChain_->GetBuffer(0, IID_PPV_ARGS(backBuffer.GetAddressOf()));
    if (FAILED(hr)) return false;

    hr = device_->CreateRenderTargetView(
        backBuffer.Get(), nullptr, renderTarget_.GetAddressOf());
    if (FAILED(hr)) return false;

    D3D11_TEXTURE2D_DESC depthDesc = {};
    depthDesc.Width = width;
    depthDesc.Height = height;
    depthDesc.MipLevels = 1;
    depthDesc.ArraySize = 1;
    depthDesc.Format = DXGI_FORMAT_D24_UNORM_S8_UINT;
    depthDesc.SampleDesc.Count = 1;
    depthDesc.Usage = D3D11_USAGE_DEFAULT;
    depthDesc.BindFlags = D3D11_BIND_DEPTH_STENCIL;

    hr = device_->CreateTexture2D(
        &depthDesc, nullptr, depthTexture_.GetAddressOf());
    if (FAILED(hr)) return false;

    hr = device_->CreateDepthStencilView(
        depthTexture_.Get(), nullptr, depthView_.GetAddressOf());
    if (FAILED(hr)) return false;

    viewport_ = {};
    viewport_.Width = static_cast<float>(width);
    viewport_.Height = static_cast<float>(height);
    viewport_.MinDepth = 0.0f;
    viewport_.MaxDepth = 1.0f;

    ComPtr<ID3DBlob> vsByteCode;
    ComPtr<ID3DBlob> psByteCode;
    if (!CompileShader("VSMain", "vs_5_0", vsByteCode)) return false;
    if (!CompileShader("PSMain", "ps_5_0", psByteCode)) return false;

    hr = device_->CreateVertexShader(
        vsByteCode->GetBufferPointer(),
        vsByteCode->GetBufferSize(),
        nullptr,
        vertexShader_.GetAddressOf());
    if (FAILED(hr)) return false;

    hr = device_->CreatePixelShader(
        psByteCode->GetBufferPointer(),
        psByteCode->GetBufferSize(),
        nullptr,
        pixelShader_.GetAddressOf());
    if (FAILED(hr)) return false;

    const D3D11_INPUT_ELEMENT_DESC elements[] = {
        {"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0,
         D3D11_INPUT_PER_VERTEX_DATA, 0},
        {"COLOR", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 12,
         D3D11_INPUT_PER_VERTEX_DATA, 0}
    };

    hr = device_->CreateInputLayout(
        elements,
        2,
        vsByteCode->GetBufferPointer(),
        vsByteCode->GetBufferSize(),
        inputLayout_.GetAddressOf());
    if (FAILED(hr)) return false;

    // A unit cube: [-1, 1] in each coordinate.
    const Vertex vertices[] = {
        {{-1, -1, -1}, {0.2f, 0.4f, 1.0f}},
        {{-1,  1, -1}, {0.3f, 0.8f, 1.0f}},
        {{ 1,  1, -1}, {0.3f, 0.5f, 1.0f}},
        {{ 1, -1, -1}, {0.1f, 0.2f, 0.7f}},
        {{-1, -1,  1}, {0.5f, 0.3f, 1.0f}},
        {{-1,  1,  1}, {0.5f, 0.9f, 1.0f}},
        {{ 1,  1,  1}, {0.8f, 0.5f, 1.0f}},
        {{ 1, -1,  1}, {0.5f, 0.2f, 0.9f}}
    };

    const unsigned int indices[] = {
        0,1,2, 0,2,3,
        4,6,5, 4,7,6,
        4,5,1, 4,1,0,
        3,2,6, 3,6,7,
        1,5,6, 1,6,2,
        0,3,7, 0,7,4
    };

    D3D11_BUFFER_DESC vbDesc = {};
    vbDesc.ByteWidth = static_cast<UINT>(sizeof(vertices));
    vbDesc.Usage = D3D11_USAGE_IMMUTABLE;
    vbDesc.BindFlags = D3D11_BIND_VERTEX_BUFFER;

    D3D11_SUBRESOURCE_DATA vbData = {};
    vbData.pSysMem = vertices;
    hr = device_->CreateBuffer(&vbDesc, &vbData, vertexBuffer_.GetAddressOf());
    if (FAILED(hr)) return false;

    D3D11_BUFFER_DESC ibDesc = {};
    ibDesc.ByteWidth = static_cast<UINT>(sizeof(indices));
    ibDesc.Usage = D3D11_USAGE_IMMUTABLE;
    ibDesc.BindFlags = D3D11_BIND_INDEX_BUFFER;

    D3D11_SUBRESOURCE_DATA ibData = {};
    ibData.pSysMem = indices;
    hr = device_->CreateBuffer(&ibDesc, &ibData, indexBuffer_.GetAddressOf());
    if (FAILED(hr)) return false;

    D3D11_BUFFER_DESC cbDesc = {};
    cbDesc.ByteWidth = static_cast<UINT>(sizeof(TransformBuffer));
    cbDesc.Usage = D3D11_USAGE_DEFAULT;
    cbDesc.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
    hr = device_->CreateBuffer(&cbDesc, nullptr, constantBuffer_.GetAddressOf());
    if (FAILED(hr)) return false;

    D3D11_RASTERIZER_DESC rasterDesc = {};
    rasterDesc.FillMode = D3D11_FILL_SOLID;
    rasterDesc.CullMode = D3D11_CULL_NONE;
    rasterDesc.DepthClipEnable = TRUE;
    hr = device_->CreateRasterizerState(
        &rasterDesc, rasterState_.GetAddressOf());
    if (FAILED(hr)) return false;

    D3D11_DEPTH_STENCIL_DESC depthStateDesc = {};
    depthStateDesc.DepthEnable = TRUE;
    depthStateDesc.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ALL;
    depthStateDesc.DepthFunc = D3D11_COMPARISON_LESS;
    hr = device_->CreateDepthStencilState(
        &depthStateDesc, depthState_.GetAddressOf());
    if (FAILED(hr)) return false;

    // Unit sphere mesh. Shared by all sphere colliders.
    // No third-party rendering library is required.
    constexpr int kSlices = 24;
    constexpr int kStacks = 16;
    std::vector<Vertex> sphereVertices;
    std::vector<unsigned int> sphereIndices;
    sphereVertices.reserve((kStacks + 1) * (kSlices + 1));
    sphereIndices.reserve(kStacks * kSlices * 6);
    for (int stack = 0; stack <= kStacks; ++stack)
    {
        const float phi = XM_PI * stack / kStacks;
        const float yy = std::cos(phi);
        const float ring = std::sin(phi);
        for (int slice = 0; slice <= kSlices; ++slice)
        {
            const float theta = XM_2PI * slice / kSlices;
            const float xx = ring * std::cos(theta);
            const float zz = ring * std::sin(theta);
            sphereVertices.push_back({
                {xx, yy, zz},
                {0.98f, 0.52f + 0.28f * yy, 0.14f + 0.22f * xx}
            });
        }
    }
    for (int stack = 0; stack < kStacks; ++stack)
    {
        for (int slice = 0; slice < kSlices; ++slice)
        {
            const unsigned int a = stack * (kSlices + 1) + slice;
            const unsigned int b = a + (kSlices + 1);
            sphereIndices.push_back(a);
            sphereIndices.push_back(b);
            sphereIndices.push_back(a + 1);
            sphereIndices.push_back(a + 1);
            sphereIndices.push_back(b);
            sphereIndices.push_back(b + 1);
        }
    }
    D3D11_BUFFER_DESC sphereVBDesc = {};
    sphereVBDesc.ByteWidth = static_cast<UINT>(sphereVertices.size() * sizeof(Vertex));
    sphereVBDesc.Usage = D3D11_USAGE_IMMUTABLE;
    sphereVBDesc.BindFlags = D3D11_BIND_VERTEX_BUFFER;
    D3D11_SUBRESOURCE_DATA sphereVBData = {};
    sphereVBData.pSysMem = sphereVertices.data();
    hr = device_->CreateBuffer(&sphereVBDesc, &sphereVBData,
                               sphereVertexBuffer_.GetAddressOf());
    if (FAILED(hr)) return false;

    D3D11_BUFFER_DESC sphereIBDesc = {};
    sphereIBDesc.ByteWidth = static_cast<UINT>(sphereIndices.size() * sizeof(unsigned int));
    sphereIBDesc.Usage = D3D11_USAGE_IMMUTABLE;
    sphereIBDesc.BindFlags = D3D11_BIND_INDEX_BUFFER;
    D3D11_SUBRESOURCE_DATA sphereIBData = {};
    sphereIBData.pSysMem = sphereIndices.data();
    hr = device_->CreateBuffer(&sphereIBDesc, &sphereIBData,
                               sphereIndexBuffer_.GetAddressOf());
    if (FAILED(hr)) return false;
    sphereIndexCount_ = static_cast<UINT>(sphereIndices.size());

    // Dynamic vertex buffer for batched debug lines.
    D3D11_BUFFER_DESC debugVBDesc = {};
    debugVBDesc.ByteWidth = static_cast<UINT>(
        sizeof(DebugVertex) * kMaxDebugVertices);
    debugVBDesc.Usage = D3D11_USAGE_DYNAMIC;
    debugVBDesc.BindFlags = D3D11_BIND_VERTEX_BUFFER;
    debugVBDesc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
    hr = device_->CreateBuffer(
        &debugVBDesc, nullptr, debugVertexBuffer_.GetAddressOf());
    if (FAILED(hr)) return false;

    // Debug overlays (body axes, contact markers) ignore depth.
    D3D11_DEPTH_STENCIL_DESC noDepthDesc = {};
    noDepthDesc.DepthEnable = FALSE;
    noDepthDesc.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ZERO;
    noDepthDesc.DepthFunc = D3D11_COMPARISON_ALWAYS;
    hr = device_->CreateDepthStencilState(
        &noDepthDesc, noDepthState_.GetAddressOf());
    if (FAILED(hr)) return false;

    depthLines_.reserve(128);
    overlayLines_.reserve(128);

    SetOrbitCamera(2.655f, 0.342f, 20.35f);
    return true;
}

void D3D11Renderer::SetOrbitCamera(float yawRadians, float pitchRadians,
                                  float distance, float targetHeight)
{
    const float cp = std::cos(pitchRadians);
    const XMVECTOR eye = XMVectorSet(
        std::sin(yawRadians) * cp * distance,
        targetHeight + std::sin(pitchRadians) * distance,
        std::cos(yawRadians) * cp * distance,
        1.0f);
    const XMVECTOR target = XMVectorSet(0.0f, targetHeight, 0.0f, 1.0f);
    const XMVECTOR up = XMVectorSet(0.0f, 1.0f, 0.0f, 0.0f);
    const XMMATRIX view = XMMatrixLookAtLH(eye, target, up);
    const XMMATRIX projection = XMMatrixPerspectiveFovLH(
        XM_PIDIV4,
        viewport_.Width / viewport_.Height,
        0.1f,
        350.0f);
    XMStoreFloat3(&cameraPosition_, eye);
    XMStoreFloat4x4(&viewMatrix_, view);
    XMStoreFloat4x4(&projectionMatrix_, projection);
    XMStoreFloat4x4(&viewProjection_, view * projection);
}

bool D3D11Renderer::ScreenPointToRay(int pixelX, int pixelY,
                                    PE::Vector3& origin,
                                    PE::Vector3& direction) const
{
    if (viewport_.Width <= 0.0f || viewport_.Height <= 0.0f)
        return false;

    const XMMATRIX view = XMLoadFloat4x4(&viewMatrix_);
    const XMMATRIX projection = XMLoadFloat4x4(&projectionMatrix_);
    const XMVECTOR nearPoint = XMVector3Unproject(
        XMVectorSet(static_cast<float>(pixelX), static_cast<float>(pixelY),
                    0.0f, 1.0f),
        0.0f, 0.0f, viewport_.Width, viewport_.Height, 0.0f, 1.0f,
        projection, view, XMMatrixIdentity());
    const XMVECTOR eye = XMVectorSet(cameraPosition_.x, cameraPosition_.y,
                                    cameraPosition_.z, 1.0f);
    const XMVECTOR ray = XMVector3Normalize(XMVectorSubtract(nearPoint, eye));
    origin = PE::Vector3(cameraPosition_.x, cameraPosition_.y,
                         cameraPosition_.z);
    direction = PE::Vector3(XMVectorGetX(ray), XMVectorGetY(ray),
                            XMVectorGetZ(ray));
    return true;
}

void D3D11Renderer::BeginFrame()
{
    depthLines_.clear();
    overlayLines_.clear();
    ID3D11RenderTargetView* targets[] = {renderTarget_.Get()};
    context_->OMSetRenderTargets(1, targets, depthView_.Get());
    context_->RSSetViewports(1, &viewport_);
    context_->RSSetState(rasterState_.Get());
    context_->OMSetDepthStencilState(depthState_.Get(), 0);

    const float clearColor[] = {0.06f, 0.08f, 0.12f, 1.0f};
    context_->ClearRenderTargetView(renderTarget_.Get(), clearColor);
    context_->ClearDepthStencilView(
        depthView_.Get(), D3D11_CLEAR_DEPTH, 1.0f, 0);

    UINT stride = sizeof(Vertex);
    UINT offset = 0;
    ID3D11Buffer* vertexBuffers[] = {vertexBuffer_.Get()};
    context_->IASetVertexBuffers(0, 1, vertexBuffers, &stride, &offset);
    context_->IASetIndexBuffer(indexBuffer_.Get(), DXGI_FORMAT_R32_UINT, 0);
    context_->IASetInputLayout(inputLayout_.Get());
    context_->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
    context_->VSSetShader(vertexShader_.Get(), nullptr, 0);
    context_->PSSetShader(pixelShader_.Get(), nullptr, 0);
    ID3D11Buffer* constantBuffers[] = {constantBuffer_.Get()};
    context_->VSSetConstantBuffers(0, 1, constantBuffers);
}

void D3D11Renderer::DrawBox(const PE::RigidBody& body, const PE::Vector3& halfSize)
{
    // Drawing a sphere changes the IA buffers; explicitly bind cube buffers.
    const UINT stride = sizeof(Vertex);
    const UINT offset = 0;
    ID3D11Buffer* buffers[] = {vertexBuffer_.Get()};
    context_->IASetVertexBuffers(0, 1, buffers, &stride, &offset);
    context_->IASetIndexBuffer(indexBuffer_.Get(), DXGI_FORMAT_R32_UINT, 0);
    context_->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

    const PE::Matrix3 rotation = body.orientation.ToMatrix3();

    // Engine: column-vector rotation R * p.
    // DirectXMath: row-vector p * transpose(R).
    // Therefore the 3x3 part below is transpose(engineRotation).
    const XMMATRIX pose = XMMatrixSet(
        static_cast<float>(rotation.m00),
        static_cast<float>(rotation.m10),
        static_cast<float>(rotation.m20), 0.0f,
        static_cast<float>(rotation.m01),
        static_cast<float>(rotation.m11),
        static_cast<float>(rotation.m21), 0.0f,
        static_cast<float>(rotation.m02),
        static_cast<float>(rotation.m12),
        static_cast<float>(rotation.m22), 0.0f,
        static_cast<float>(body.position.x),
        static_cast<float>(body.position.y),
        static_cast<float>(body.position.z), 1.0f);

    const XMMATRIX world = XMMatrixScaling(
        static_cast<float>(halfSize.x),
        static_cast<float>(halfSize.y),
        static_cast<float>(halfSize.z)) * pose;

    const XMMATRIX vp = XMLoadFloat4x4(&viewProjection_);
    TransformBuffer transform = {};
    XMStoreFloat4x4(
        &transform.worldViewProjection,
        XMMatrixTranspose(world * vp));

    context_->UpdateSubresource(
        constantBuffer_.Get(), 0, nullptr, &transform, 0, 0);
    context_->DrawIndexed(36, 0, 0);
}

void D3D11Renderer::DrawSphere(const PE::RigidBody& body, PE::Real radius)
{
    if (radius <= PE::Real{0}) return;
    const PE::Matrix3 rotation = body.orientation.ToMatrix3();
    const XMMATRIX pose = XMMatrixSet(
        static_cast<float>(rotation.m00),
        static_cast<float>(rotation.m10),
        static_cast<float>(rotation.m20), 0.0f,
        static_cast<float>(rotation.m01),
        static_cast<float>(rotation.m11),
        static_cast<float>(rotation.m21), 0.0f,
        static_cast<float>(rotation.m02),
        static_cast<float>(rotation.m12),
        static_cast<float>(rotation.m22), 0.0f,
        static_cast<float>(body.position.x),
        static_cast<float>(body.position.y),
        static_cast<float>(body.position.z), 1.0f);
    const float r = static_cast<float>(radius);
    const XMMATRIX world = XMMatrixScaling(r, r, r) * pose;
    const XMMATRIX vp = XMLoadFloat4x4(&viewProjection_);
    TransformBuffer transform = {};
    XMStoreFloat4x4(&transform.worldViewProjection,
                    XMMatrixTranspose(world * vp));
    context_->UpdateSubresource(constantBuffer_.Get(), 0, nullptr,
                                &transform, 0, 0);

    const UINT stride = sizeof(Vertex);
    const UINT offset = 0;
    ID3D11Buffer* buffers[] = {sphereVertexBuffer_.Get()};
    context_->IASetVertexBuffers(0, 1, buffers, &stride, &offset);
    context_->IASetIndexBuffer(sphereIndexBuffer_.Get(),
                               DXGI_FORMAT_R32_UINT, 0);
    context_->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
    context_->DrawIndexed(sphereIndexCount_, 0, 0);
}

void D3D11Renderer::DrawLine(
    const PE::Vector3& start, const PE::Vector3& end,
    float red, float green, float blue, bool overlay)
{
    auto& list = overlay ? overlayLines_ : depthLines_;
    if (list.size() + 2 > kMaxDebugVertices) return;

    list.push_back({
        static_cast<float>(start.x),
        static_cast<float>(start.y),
        static_cast<float>(start.z), red, green, blue
    });
    list.push_back({
        static_cast<float>(end.x),
        static_cast<float>(end.y),
        static_cast<float>(end.z), red, green, blue
    });
}

void D3D11Renderer::DrawGrid(int halfCells, float spacing)
{
    if (halfCells <= 0 || spacing <= 0.0f) return;
    const float extent = static_cast<float>(halfCells) * spacing;

    // Floor visual's top is y=0; lift grid slightly to avoid z-fighting.
    constexpr float y = 0.012f;
    for (int i = -halfCells; i <= halfCells; ++i)
    {
        const float p = static_cast<float>(i) * spacing;
        DrawLine(PE::Vector3(-extent, y, p),
                 PE::Vector3( extent, y, p),
                 0.18f, 0.26f, 0.35f);
        DrawLine(PE::Vector3(p, y, -extent),
                 PE::Vector3(p, y,  extent),
                 0.18f, 0.26f, 0.35f);
    }
}

void D3D11Renderer::DrawBodyAxes(
    const PE::RigidBody& body, float axisLength)
{
    if (axisLength <= 0.0f) return;

    const PE::Matrix3 rotation = body.orientation.ToMatrix3();
    const PE::Vector3& p = body.position;
    const PE::Real length = static_cast<PE::Real>(axisLength);

    // Rotation matrix columns = local axes in world space.
    const PE::Vector3 xAxis(rotation.m00, rotation.m10, rotation.m20);
    const PE::Vector3 yAxis(rotation.m01, rotation.m11, rotation.m21);
    const PE::Vector3 zAxis(rotation.m02, rotation.m12, rotation.m22);

    DrawLine(p, p + xAxis * length, 1.0f, 0.1f, 0.1f, true);  // X: red
    DrawLine(p, p + yAxis * length, 0.1f, 1.0f, 0.1f, true);  // Y: green
    DrawLine(p, p + zAxis * length, 0.1f, 0.5f, 1.0f, true);  // Z: blue
}

void D3D11Renderer::DrawContact(const PE::Contact& contact, float size)
{
    if (size <= 0.0f) return;

    const PE::Vector3& p = contact.point;
    const PE::Real s = static_cast<PE::Real>(size);

    // A magenta 3D cross centered at the detected contact point.
    const PE::Vector3 dx(s, 0, 0);
    const PE::Vector3 dy(0, s, 0);
    const PE::Vector3 dz(0, 0, s);
    DrawLine(p - dx, p + dx, 1.0f, 0.15f, 0.7f, true);
    DrawLine(p - dy, p + dy, 1.0f, 0.15f, 0.7f, true);
    DrawLine(p - dz, p + dz, 1.0f, 0.15f, 0.7f, true);

    // Contact normal points from body B toward body A.
    DrawLine(p, p + contact.normal * (s * PE::Real{3}),
             1.0f, 1.0f, 0.1f, true);
}

bool D3D11Renderer::FlushLines(
    const std::vector<DebugVertex>& vertices, bool overlay)
{
    if (vertices.empty()) return true;
    if (vertices.size() > kMaxDebugVertices) return false;

    D3D11_MAPPED_SUBRESOURCE mapped = {};
    const HRESULT hr = context_->Map(
        debugVertexBuffer_.Get(), 0, D3D11_MAP_WRITE_DISCARD, 0, &mapped);
    if (FAILED(hr)) return false;

    std::memcpy(mapped.pData, vertices.data(),
                vertices.size() * sizeof(DebugVertex));
    context_->Unmap(debugVertexBuffer_.Get(), 0);

    // Debug vertices are already in world space: only view*projection.
    const XMMATRIX vp = XMLoadFloat4x4(&viewProjection_);
    TransformBuffer transform = {};
    XMStoreFloat4x4(
        &transform.worldViewProjection, XMMatrixTranspose(vp));
    context_->UpdateSubresource(
        constantBuffer_.Get(), 0, nullptr, &transform, 0, 0);

    context_->OMSetDepthStencilState(
        overlay ? noDepthState_.Get() : depthState_.Get(), 0);

    const UINT stride = sizeof(DebugVertex);
    const UINT offset = 0;
    ID3D11Buffer* buffers[] = {debugVertexBuffer_.Get()};
    context_->IASetVertexBuffers(0, 1, buffers, &stride, &offset);
    context_->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_LINELIST);
    context_->Draw(static_cast<UINT>(vertices.size()), 0);
    return true;
}

bool D3D11Renderer::EndFrame()
{
    if (!FlushLines(depthLines_, false)) return false;
    if (!FlushLines(overlayLines_, true)) return false;
    return SUCCEEDED(swapChain_->Present(1, 0));
}

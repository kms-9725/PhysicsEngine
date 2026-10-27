// Physics sandbox with native Win32 controls and DX11 visualization.
// Uses the project's existing namespace PE physics engine. No external GUI package.
#define NOMINMAX
#include <windows.h>
#include <windowsx.h>
#include <commctrl.h>
#pragma comment(lib, "comctl32.lib")

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cwchar>
#include <limits>
#include <deque>
#include <string>
#include <vector>

#include "renderer/D3D11Renderer.h"
#include "physics/RigidBody.h"
#include "collision/BoxCollider.h"
#include "collision/SphereCollider.h"
#include "collision/PlaneCollider.h"
#include "collision/CollisionDetector.h"
#include "collision/ContactResolver.h"
#include "sandbox/SandboxScene.h"

namespace
{
    constexpr int kClientW = 1660;
    constexpr int kClientH = 960;
    constexpr int kViewX = 10;
    constexpr int kViewY = 10;
    constexpr int kViewW = 1210;
    constexpr int kViewH = 930;
    constexpr int kPanelX = 1230;
    constexpr int kPanelW = 400;


    enum ControlId : int
    {
        IDC_SCENE = 100,
        IDC_GRAVITY = 101, IDC_RESTITUTION = 102, IDC_FRICTION = 103,
        IDC_SPEED = 104, IDC_HEIGHT = 105, IDC_TIMESCALE = 106,
        IDC_COUNT = 107, IDC_PROJECTILE_SPEED = 108,
        IDC_INTEGRATOR = 112, IDC_TIMESTEP = 113, IDC_ANGULAR = 114,
        IDC_PAUSE = 200, IDC_STEP = 201, IDC_RESET = 202,
        IDC_ADD_BOX = 203, IDC_ADD_SPHERE = 204, IDC_SHOOT = 205,
        IDC_GRID = 210, IDC_AXES = 211, IDC_CONTACTS = 212,
        IDC_FAST_SOLVER = 213
    };

    struct MouseGrab
    {
        PE::RigidBody* body = nullptr;
        PE::Vector3 localAnchor;
        PE::Vector3 target;
        PE::Real depth = PE::Real{0};
    };

    bool HitSphere(const PE::Vector3& rayOrigin,
                   const PE::Vector3& rayDirection,
                   const PE::SphereCollider& sphere,
                   PE::Real& distance)
    {
        const PE::Vector3 offset = rayOrigin - sphere.body->position;
        const PE::Real b = offset.Dot(rayDirection);
        const PE::Real c = offset.LengthSquared() - sphere.radius * sphere.radius;
        const PE::Real discriminant = b * b - c;
        if (discriminant < PE::Real{0}) return false;
        const PE::Real root = static_cast<PE::Real>(std::sqrt(discriminant));
        const PE::Real tNear = -b - root;
        const PE::Real tFar = -b + root;
        distance = tNear > PE::Real{0.001f} ? tNear : tFar;
        return distance > PE::Real{0.001f};
    }

    bool HitBox(const PE::Vector3& rayOrigin,
                const PE::Vector3& rayDirection,
                const PE::BoxCollider& box,
                PE::Real& distance)
    {
        const PE::Matrix3 worldToLocal =
            box.body->orientation.ToMatrix3().Transposed();
        const PE::Vector3 localOrigin =
            worldToLocal * (rayOrigin - box.body->position);
        const PE::Vector3 localDirection = worldToLocal * rayDirection;
        const PE::Real origins[3] = {localOrigin.x, localOrigin.y, localOrigin.z};
        const PE::Real directions[3] = {localDirection.x, localDirection.y, localDirection.z};
        const PE::Real extents[3] = {box.halfSize.x, box.halfSize.y, box.halfSize.z};
        PE::Real nearT = -std::numeric_limits<PE::Real>::infinity();
        PE::Real farT = std::numeric_limits<PE::Real>::infinity();
        for (int axis = 0; axis < 3; ++axis)
        {
            if (std::abs(directions[axis]) < PE::Real{1e-6f})
            {
                if (origins[axis] < -extents[axis] ||
                    origins[axis] > extents[axis]) return false;
                continue;
            }
            PE::Real first = (-extents[axis] - origins[axis]) / directions[axis];
            PE::Real second = (extents[axis] - origins[axis]) / directions[axis];
            if (first > second) std::swap(first, second);
            nearT = std::max(nearT, first);
            farT = std::min(farT, second);
            if (nearT > farT) return false;
        }
        distance = nearT > PE::Real{0.001f} ? nearT : farT;
        return distance > PE::Real{0.001f};
    }

    PE::Vector3 GrabWorldAnchor(const MouseGrab& grab)
    {
        if (!grab.body) return PE::Vector3();
        return grab.body->position +
            grab.body->orientation.ToMatrix3() * grab.localAnchor;
    }

    using SandboxScene = Demo::SandboxScene;

    struct AppState
    {
        HWND window = nullptr;
        HWND view = nullptr;
        HWND energyView = nullptr;
        HWND sceneCombo = nullptr;
        HWND integratorCombo = nullptr;
        HWND angularCombo = nullptr;
        HWND timestepCombo = nullptr;
        HWND stepButton = nullptr;
        HWND slider[8]{};
        HWND sliderText[8]{};
        HWND pausedButton = nullptr;
        HWND infoText = nullptr;
        HFONT font = nullptr;
        HFONT titleFont = nullptr;
        D3D11Renderer renderer;
        SandboxScene scene;
        int preset = Demo::Gallery;
        int objectCount = 12;
        bool paused = false;
        bool stepOnce = false;
        bool showGrid = true;
        bool showAxes = false;
        bool showContacts = false;
        PE::Real gravity = PE::Real{9.8f};
        PE::Real restitution = PE::Real{0.15f};
        PE::Real friction = PE::Real{0.65f};
        PE::Real startSpeed = PE::Real{2};
        PE::Real startHeight = PE::Real{3};
        PE::Real projectileSpeed = PE::Real{20};
        PE::Real springStiffness = PE::Real{10};
        float timeScale = 1.0f;
        double accumulator = 0;
        double fixedStep = 1.0 / 120.0;

        float cameraYaw = 2.655f;
        float cameraPitch = 0.342f;
        float cameraDistance = 20.35f;
        float cameraTargetY = 1.2f;
        bool orbiting = false;
        POINT mouseLast{};
        POINT mouseNow{};
        MouseGrab grab;
    };

    AppState* gApp = nullptr;

    void FitCamera(AppState& app)
    {
        app.cameraTargetY = 1.2f;
        app.cameraDistance = 20.35f;
        app.cameraYaw = 2.655f;
        app.cameraPitch = 0.342f;
        if (app.preset == Demo::TallStack)
        {
            app.cameraTargetY = 0.5f * app.objectCount;
            app.cameraDistance = std::max(16.0f, 1.65f * app.objectCount);
            app.cameraYaw = 2.95f;
            app.cameraPitch = 0.08f;
        }
        else if (app.preset == Demo::Pyramid)
        {
            app.cameraTargetY = 0.24f * app.objectCount;
            app.cameraDistance = std::max(14.0f, 0.98f * app.objectCount);
            app.cameraYaw = 0.09f; // Look nearly along Z: pyramid silhouette is visible.
            app.cameraPitch = 0.06f;
        }
        else if (app.preset == Demo::FreeRotation)
        {
            app.cameraTargetY = 1.5f;
            app.cameraDistance = 10.0f;
        }
        else if (app.preset == Demo::SpringTest)
        {
            app.cameraTargetY = 2.2f;
            app.cameraYaw = 0.23f;
            app.cameraPitch = 0.14f;
            app.cameraDistance = 13.0f;
        }
        app.renderer.SetOrbitCamera(app.cameraYaw, app.cameraPitch,
                                    app.cameraDistance, app.cameraTargetY);
    }

    void ReleaseGrab(AppState& app)
    {
        app.scene.EndGrab();
        app.grab = MouseGrab{};
    }

    void ResetScene(AppState& app)
    {
        ReleaseGrab(app);
        app.scene.SetRestitution(app.restitution);
        app.scene.friction = app.friction;
        const int linearChoice = static_cast<int>(SendMessageW(
            app.integratorCombo, CB_GETCURSEL, 0, 0));
        app.scene.linearMethod = (linearChoice == 1)
            ? PE::LinearIntegrationMethod::ExplicitEuler
            : PE::LinearIntegrationMethod::SemiImplicitEuler;

        // GUI order intentionally defaults to Strang splitting; enum order is different.
        constexpr PE::AngularIntegrationMethod kAngularMethods[] = {
            PE::AngularIntegrationMethod::StrangSplitting,
            PE::AngularIntegrationMethod::PreviousOmega,
            PE::AngularIntegrationMethod::NextOmega,
            PE::AngularIntegrationMethod::MidpointOmega,
            PE::AngularIntegrationMethod::EulerEquation,
            PE::AngularIntegrationMethod::JoltStyle,
            PE::AngularIntegrationMethod::BulletStyle,
            PE::AngularIntegrationMethod::GyroscopicOff
        };
        const int angularChoice = static_cast<int>(SendMessageW(
            app.angularCombo, CB_GETCURSEL, 0, 0));
        if (angularChoice >= 0 && angularChoice < 8)
            app.scene.angularMethod = kAngularMethods[angularChoice];
        app.scene.springStiffness = app.springStiffness;
        app.scene.SetGravity(app.gravity);
        app.scene.Reset(app.preset, app.startSpeed, app.startHeight, app.objectCount);
        app.accumulator = 0;
        app.stepOnce = false;
        FitCamera(app);
        if (app.energyView) InvalidateRect(app.energyView, nullptr, FALSE);
    }

    void SyncSliders(AppState& app)
    {
        wchar_t b[120];
        std::swprintf(b,120,L"Gravity: %.1f m/s^2 (diagnostics = 0)",app.gravity);
        SetWindowTextW(app.sliderText[0], b);
        std::swprintf(b,120,L"Restitution: %.2f",app.restitution);
        SetWindowTextW(app.sliderText[1], b);
        std::swprintf(b,120,L"Friction: %.2f",app.friction);
        SetWindowTextW(app.sliderText[2], b);
        if (app.preset == Demo::SpringTest)
            std::swprintf(b,120,L"Spring stiffness: %.1f N/m (Reset)",app.springStiffness);
        else
            std::swprintf(b,120,L"Collision speed: %.1f (Reset)",app.startSpeed);
        SetWindowTextW(app.sliderText[3], b);
        std::swprintf(b,120,L"Drop height: %.1f (Reset)",app.startHeight);
        SetWindowTextW(app.sliderText[4], b);
        std::swprintf(b,120,L"Time scale: %.2fx",app.timeScale);
        SetWindowTextW(app.sliderText[5], b);
        std::swprintf(b,120,L"Stack boxes / pyramid rows: %d (Reset)",app.objectCount);
        SetWindowTextW(app.sliderText[6], b);
        std::swprintf(b,120,L"Projectile speed: %.1f m/s",app.projectileSpeed);
        SetWindowTextW(app.sliderText[7], b);
    }

    void ReadSliders(AppState& app)
    {
        const auto pos=[&app](int i)->int{
            return static_cast<int>(SendMessageW(app.slider[i],TBM_GETPOS,0,0));
        };
        const PE::Real oldGravity=app.gravity;
        const PE::Real oldSpringStiffness=app.springStiffness;
        app.gravity = static_cast<PE::Real>(pos(0)*0.1f);
        app.restitution = static_cast<PE::Real>(pos(1)*0.01f);
        app.friction = static_cast<PE::Real>(pos(2)*0.01f);
        app.startSpeed = static_cast<PE::Real>(pos(3)*0.1f);
        app.springStiffness = static_cast<PE::Real>(pos(3)*0.5f);
        app.startHeight = static_cast<PE::Real>(pos(4)*0.1f);
        app.timeScale = pos(5)*0.01f;
        app.objectCount = pos(6);
        app.projectileSpeed = static_cast<PE::Real>(pos(7)*0.1f);
        app.scene.SetGravity(app.gravity);
        // The spring scenario ignores gravity. Changing its stiffness is a
        // change of the physical system, so restart from identical x(0),v(0).
        if (app.preset == Demo::SpringTest && oldSpringStiffness != app.springStiffness)
            ResetScene(app);
        app.scene.SetRestitution(app.restitution);
        app.scene.friction=app.friction;
        SyncSliders(app);
    }

    void UpdateGrabTarget(AppState& app)
    {
        if (!app.grab.body) return;
        PE::Vector3 rayOrigin, rayDirection;
        if (app.renderer.ScreenPointToRay(app.mouseNow.x, app.mouseNow.y,
                                          rayOrigin, rayDirection))
        {
            app.grab.target = rayOrigin + rayDirection * app.grab.depth;
            app.scene.MoveGrab(app.grab.target);
        }
    }

    void OrbitCamera(AppState& app, int deltaX, int deltaY)
    {
        app.cameraYaw -= static_cast<float>(deltaX) * 0.008f;
        app.cameraPitch += static_cast<float>(deltaY) * 0.008f;
        app.cameraPitch = std::clamp(app.cameraPitch, -1.45f, 1.45f);
        app.renderer.SetOrbitCamera(
            app.cameraYaw, app.cameraPitch, app.cameraDistance, app.cameraTargetY);
        UpdateGrabTarget(app);
    }

    void ZoomCamera(AppState& app, short wheelDelta)
    {
        // One wheel detent multiplies camera distance by 0.87 (in) or 1/0.87 (out).
        const float detents = static_cast<float>(wheelDelta) / WHEEL_DELTA;
        app.cameraDistance = std::clamp(
            app.cameraDistance * std::pow(0.87f, detents), 2.7f, 250.0f);
        app.renderer.SetOrbitCamera(
            app.cameraYaw, app.cameraPitch, app.cameraDistance, app.cameraTargetY);
        UpdateGrabTarget(app);
    }

    void BeginGrab(AppState& app, int mouseX, int mouseY)
    {
        app.grab = MouseGrab{};
        PE::Vector3 rayOrigin, rayDirection;
        if (!app.renderer.ScreenPointToRay(mouseX, mouseY,
                                          rayOrigin, rayDirection)) return;
        PE::Real closest = std::numeric_limits<PE::Real>::infinity();
        PE::RigidBody* picked = nullptr;
        for (const auto& box : app.scene.boxes)
        {
            PE::Real t = PE::Real{0};
            if (HitBox(rayOrigin, rayDirection, box, t) && t < closest)
            {
                closest = t;
                picked = box.body;
            }
        }
        for (const auto& sphere : app.scene.spheres)
        {
            PE::Real t = PE::Real{0};
            if (HitSphere(rayOrigin, rayDirection, sphere, t) && t < closest)
            {
                closest = t;
                picked = sphere.body;
            }
        }
        if (!picked || app.scene.IsDiagnostic()) return;
        const PE::Vector3 worldHit = rayOrigin + rayDirection * closest;
        app.grab.body = picked;
        app.grab.localAnchor =
            picked->orientation.ToMatrix3().Transposed() *
            (worldHit - picked->position);
        app.grab.depth = closest;
        app.grab.target = worldHit;
        app.scene.StartGrab(*picked,app.grab.localAnchor,worldHit);
    }

    void PhysicsStep(AppState& app, PE::Real dt)
    {
        app.scene.Step(dt, app.gravity);
    }

    void DrawScene(AppState& app)
    {
        const auto& scene=app.scene;
        auto& r=app.renderer;
        r.BeginFrame();
        if(scene.HasGround()) r.DrawBox(scene.groundVisual,scene.groundHalfSize);
        for(const auto& box:scene.boxes) r.DrawBox(*box.body,box.halfSize);
        for(const auto& sphere:scene.spheres) r.DrawSphere(*sphere.body,sphere.radius);
        if(app.showGrid && scene.HasGround()) r.DrawGrid(20,.75f);
        if(app.showAxes)
        {
            // Large stress scenes should not flood the debug line buffer.
            std::size_t drawn=0;
            for(const auto& body:scene.bodies)
            {
                if(drawn++>=72) break;
                r.DrawBodyAxes(body,.38f);
            }
        }
        if(app.showContacts)
        {
            for(std::size_t i=0;i<scene.debugContacts.size() && i<8000;++i)
                r.DrawContact(scene.debugContacts[i],.085f);
        }
        if(scene.IsSpringTest() && scene.springBody)
        {
            // Draw a stylized spring coil in the same horizontal line of action.
            const PE::Vector3 anchor=scene.springGenerator.equilibrium;
            const PE::Vector3 end=scene.springBody->position;
            const PE::Vector3 diff=end-anchor;
            PE::Vector3 prev=anchor;
            constexpr int segments=24;
            for(int i=1;i<=segments;++i)
            {
                const float t=static_cast<float>(i)/segments;
                PE::Vector3 p=anchor+diff*PE::Real{t};
                if(i!=segments)
                {
                    p.y += (i%2 ? .15f : -.15f);
                }
                r.DrawLine(prev,p,.97f,.73f,.17f,true);
                prev=p;
            }
            r.DrawLine(anchor+PE::Vector3(0,-.25f,0),
                       anchor+PE::Vector3(0,.25f,0),1.f,.85f,.1f,true);
            r.DrawLine(anchor+PE::Vector3(0,0,-.25f),
                       anchor+PE::Vector3(0,0,.25f),1.f,.85f,.1f,true);
        }
        if(app.grab.body)
        {
            const PE::Vector3 anchor=GrabWorldAnchor(app.grab);
            r.DrawLine(anchor,app.grab.target,1.0f,.85f,.1f,true);
        }
    }

    void ShootProjectile(AppState& app)
    {
        if (app.scene.IsDiagnostic() || !app.scene.MaySpawn()) return;
        PE::Vector3 origin,direction;
        if(!app.renderer.ScreenPointToRay(kViewW/2,kViewH/2,origin,direction)) return;
        app.scene.AddSphere(origin+direction*PE::Real{2.8f}, PE::Real{.16f},
            direction*app.projectileSpeed,PE::Real{.16f});
    }

    LRESULT CALLBACK EnergyGraphProc(HWND hwnd,UINT msg,WPARAM wParam,LPARAM lParam)
    {
        if(msg!=WM_PAINT) return DefWindowProcW(hwnd,msg,wParam,lParam);
        PAINTSTRUCT ps{};
        HDC dc=BeginPaint(hwnd,&ps);
        RECT rc{};GetClientRect(hwnd,&rc);
        HBRUSH bg=CreateSolidBrush(RGB(23,29,39));
        FillRect(dc,&rc,bg);DeleteObject(bg);
        const AppState* app=gApp;
        if(app && app->scene.IsDiagnostic() && app->scene.energyHistory.size()>1)
        {
            const auto& values=app->scene.energyHistory;
            double limit=.1;
            for(const auto& v:values)
                if(std::isfinite(v.driftPercent))
                    limit=std::max(limit,std::abs(v.driftPercent));
            limit=std::min(limit*1.15, 1e9);
            const int w=rc.right-rc.left, h=rc.bottom-rc.top;
            const int center=h/2;
            HPEN zero=CreatePen(PS_DOT,1,RGB(95,103,119));
            HGDIOBJ old=SelectObject(dc,zero);
            MoveToEx(dc,1,center,nullptr);LineTo(dc,w-1,center);
            SelectObject(dc,old);DeleteObject(zero);
            HPEN line=CreatePen(PS_SOLID,2,RGB(63,216,188));
            old=SelectObject(dc,line);
            const double denominator=std::max(1.0,static_cast<double>(values.size()-1));
            bool started=false;
            for(std::size_t i=0;i<values.size();++i)
            {
                if(!std::isfinite(values[i].driftPercent)) {started=false;continue;}
                const int x=static_cast<int>((w-4)*i/denominator)+2;
                const double frac=std::clamp(values[i].driftPercent/limit,-1.0,1.0);
                const int y=static_cast<int>(center-frac*(h/2-5));
                if(!started) {MoveToEx(dc,x,y,nullptr);started=true;}
                else LineTo(dc,x,y);
            }
            SelectObject(dc,old);DeleteObject(line);
        }
        EndPaint(hwnd,&ps);
        return 0;
    }

    bool IsCursorOverView(const AppState& app, POINT screenPoint)
    {
        if (!app.view) return false;
        ScreenToClient(app.view, &screenPoint);
        RECT bounds{};
        GetClientRect(app.view, &bounds);
        return PtInRect(&bounds, screenPoint) != 0;
    }

    LRESULT CALLBACK MainWindowProc(HWND hwnd, UINT msg,
                                    WPARAM wParam, LPARAM lParam)
    {
        AppState* app = gApp;
        switch (msg)
        {
        case WM_COMMAND:
            if (!app) break;
            switch (LOWORD(wParam))
            {
            case IDC_SCENE:
                if (HIWORD(wParam) == CBN_SELCHANGE)
                {
                    app->preset = static_cast<int>(SendMessageW(
                        app->sceneCombo, CB_GETCURSEL, 0, 0));
                    ResetScene(*app);
                    SyncSliders(*app); // Switch the speed slider label to spring stiffness.
                }
                return 0;
            case IDC_INTEGRATOR:
            case IDC_ANGULAR:
                if(HIWORD(wParam)==CBN_SELCHANGE) ResetScene(*app);
                return 0;
            case IDC_TIMESTEP:
                if(HIWORD(wParam)==CBN_SELCHANGE)
                {
                    const int selected=static_cast<int>(SendMessageW(
                        app->timestepCombo,CB_GETCURSEL,0,0));
                    const int hz=selected==0?60:selected==1?120:240;
                    app->fixedStep=1.0/hz;
                    if(app->stepButton)
                    {
                        wchar_t caption[50];
                        std::swprintf(caption,50,L"Step 1/%ds",hz);
                        SetWindowTextW(app->stepButton,caption);
                    }
                    ResetScene(*app);
                }
                return 0;
            case IDC_SHOOT:
                if(HIWORD(wParam)==BN_CLICKED) ShootProjectile(*app);
                return 0;
            case IDC_PAUSE:
                if (HIWORD(wParam) == BN_CLICKED)
                {
                    app->paused = !app->paused;
                    app->accumulator = 0.0;
                    SetWindowTextW(app->pausedButton,
                                   app->paused ? L"Resume" : L"Pause");
                }
                return 0;
            case IDC_STEP:
                if (HIWORD(wParam) == BN_CLICKED)
                {
                    app->paused = true;
                    app->stepOnce = true;
                    app->accumulator = 0.0;
                    SetWindowTextW(app->pausedButton, L"Resume");
                }
                return 0;
            case IDC_RESET:
                if (HIWORD(wParam) == BN_CLICKED) ResetScene(*app);
                return 0;
            case IDC_ADD_BOX:
                if (HIWORD(wParam) == BN_CLICKED && !app->scene.IsDiagnostic())
                    app->scene.AddBox(PE::Vector3(0, app->startHeight + 1.5f, 0),
                        PE::Vector3(0.45f, 0.45f, 0.45f),
                        PE::Vector3(0, 0, 0), PE::Vector3(0.1f, 0, 0.08f));
                return 0;
            case IDC_ADD_SPHERE:
                if (HIWORD(wParam) == BN_CLICKED && !app->scene.IsDiagnostic())
                    app->scene.AddSphere(PE::Vector3(0, app->startHeight + 1.5f, 0),
                                         PE::Real{0.48f});
                return 0;
            case IDC_GRID:
                if (HIWORD(wParam) == BN_CLICKED)
                    app->showGrid = SendMessageW(reinterpret_cast<HWND>(lParam),
                                                  BM_GETCHECK, 0, 0) == BST_CHECKED;
                return 0;
            case IDC_AXES:
                if (HIWORD(wParam) == BN_CLICKED)
                    app->showAxes = SendMessageW(reinterpret_cast<HWND>(lParam),
                                                  BM_GETCHECK, 0, 0) == BST_CHECKED;
                return 0;
            case IDC_FAST_SOLVER:
                if (HIWORD(wParam) == BN_CLICKED)
                {
                    app->scene.fastSolver = SendMessageW(
                        reinterpret_cast<HWND>(lParam), BM_GETCHECK, 0, 0)
                        == BST_CHECKED;
                    ResetScene(*app); // Compare both modes with the same initial state.
                }
                return 0;
            case IDC_CONTACTS:
                if (HIWORD(wParam) == BN_CLICKED)
                    app->showContacts = SendMessageW(reinterpret_cast<HWND>(lParam),
                                                      BM_GETCHECK, 0, 0) == BST_CHECKED;
                return 0;
            }
            break;
        case WM_MOUSEWHEEL:
            if (app)
            {
                POINT screenPoint{GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam)};
                if (IsCursorOverView(*app, screenPoint))
                {
                    ZoomCamera(*app, GET_WHEEL_DELTA_WPARAM(wParam));
                    return 0;
                }
            }
            break;
        case WM_HSCROLL:
            if (app) ReadSliders(*app);
            return 0;
        case WM_DESTROY:
            PostQuitMessage(0);
            return 0;
        }
        return DefWindowProcW(hwnd, msg, wParam, lParam);
    }

    HWND CreateControl(HWND parent, const wchar_t* klass, const wchar_t* title,
                       DWORD style, int id, int x, int y, int w, int h,
                       HFONT font)
    {
        HWND control = CreateWindowExW(0, klass, title,
            WS_CHILD | WS_VISIBLE | style, x, y, w, h, parent,
            reinterpret_cast<HMENU>(static_cast<INT_PTR>(id)),
            GetModuleHandleW(nullptr), nullptr);
        if (control && font)
            SendMessageW(control, WM_SETFONT, reinterpret_cast<WPARAM>(font), TRUE);
        return control;
    }

    HWND AddText(AppState& app, const wchar_t* text, int y, int h = 25,
                 bool title = false)
    {
        return CreateControl(app.window, L"STATIC", text, SS_LEFT,
                             -1, kPanelX, y, kPanelW, h,
                             title ? app.titleFont : app.font);
    }

    void AddSlider(AppState& app, int index, int y,
                   int minimum, int maximum, int initial)
    {
        app.sliderText[index] = AddText(app, L"", y, 23);
        app.slider[index] = CreateControl(app.window, TRACKBAR_CLASSW, L"",
            TBS_HORZ | TBS_AUTOTICKS, IDC_GRAVITY + index,
            kPanelX - 4, y + 24, kPanelW + 8, 29, app.font);
        SendMessageW(app.slider[index], TBM_SETRANGE, TRUE,
            MAKELPARAM(minimum, maximum));
        SendMessageW(app.slider[index], TBM_SETPOS, TRUE, initial);
    }

    void BuildControls(AppState& app)
    {
        app.font=CreateFontW(-16,0,0,0,FW_NORMAL,FALSE,FALSE,FALSE,
            DEFAULT_CHARSET,OUT_DEFAULT_PRECIS,CLIP_DEFAULT_PRECIS,
            CLEARTYPE_QUALITY,DEFAULT_PITCH|FF_DONTCARE,L"Segoe UI");
        app.titleFont=CreateFontW(-22,0,0,0,FW_BOLD,FALSE,FALSE,FALSE,
            DEFAULT_CHARSET,OUT_DEFAULT_PRECIS,CLIP_DEFAULT_PRECIS,
            CLEARTYPE_QUALITY,DEFAULT_PITCH|FF_DONTCARE,L"Segoe UI");
        AddText(app,L"PHYSICS ENGINE TEST LAB",16,31,true);
        AddText(app,L"Scenario",57,23);
        app.sceneCombo=CreateControl(app.window,L"COMBOBOX",L"",
            CBS_DROPDOWNLIST|WS_VSCROLL,IDC_SCENE,kPanelX,81,kPanelW,300,app.font);
        const wchar_t* names[]={
            L"Gallery - five collision tests", L"Box / Plane", L"Sphere / Plane",
            L"Sphere / Sphere", L"Box / Sphere", L"Box / Box",
            L"Four-box stack (legacy)", L"Mixed falling bodies",
            L"FREE ROTATION - no gravity, no collision",
            L"TALL STACK - adjustable height", L"THIN PYRAMID - adjustable rows",
            L"SPRING - undamped oscillator + energy"
        };
        for(const auto* name:names)
            SendMessageW(app.sceneCombo,CB_ADDSTRING,0,reinterpret_cast<LPARAM>(name));
        SendMessageW(app.sceneCombo,CB_SETCURSEL,0,0);

        const int min[]={0,0,0,0,15,0,4,10};
        const int max[]={200,100,150,80,70,200,40,500};
        const int initial[]={98,15,65,20,30,100,12,200};
        for(int i=0;i<8;++i) AddSlider(app,i,128+i*45,min[i],max[i],initial[i]);
        ReadSliders(app);

        CreateControl(app.window,L"STATIC",L"Linear integrator",SS_LEFT,-1,
            kPanelX,499,222,22,app.font);
        CreateControl(app.window,L"STATIC",L"Fixed physics dt",SS_LEFT,-1,
            kPanelX+232,499,168,22,app.font);
        app.integratorCombo=CreateControl(app.window,L"COMBOBOX",L"",
            CBS_DROPDOWNLIST|WS_VSCROLL,IDC_INTEGRATOR,
            kPanelX,523,222,100,app.font);
        SendMessageW(app.integratorCombo,CB_ADDSTRING,0,
                     reinterpret_cast<LPARAM>(L"Semi-implicit Euler (linear)"));
        SendMessageW(app.integratorCombo,CB_ADDSTRING,0,
                     reinterpret_cast<LPARAM>(L"Explicit Euler (linear)"));
        SendMessageW(app.integratorCombo,CB_SETCURSEL,0,0);
        app.timestepCombo=CreateControl(app.window,L"COMBOBOX",L"",
            CBS_DROPDOWNLIST|WS_VSCROLL,IDC_TIMESTEP,
            kPanelX+232,523,168,100,app.font);
        SendMessageW(app.timestepCombo,CB_ADDSTRING,0,
            reinterpret_cast<LPARAM>(L"1/60 s"));
        SendMessageW(app.timestepCombo,CB_ADDSTRING,0,
            reinterpret_cast<LPARAM>(L"1/120 s"));
        SendMessageW(app.timestepCombo,CB_ADDSTRING,0,
            reinterpret_cast<LPARAM>(L"1/240 s"));
        SendMessageW(app.timestepCombo,CB_SETCURSEL,1,0);

        app.pausedButton=CreateControl(app.window,L"BUTTON",L"Pause",
            BS_PUSHBUTTON,IDC_PAUSE,kPanelX,562,193,30,app.font);
        app.stepButton=CreateControl(app.window,L"BUTTON",L"Step 1/120s",
            BS_PUSHBUTTON,IDC_STEP,kPanelX+202,562,198,30,app.font);
        CreateControl(app.window,L"BUTTON",L"Reset / Rebuild scene",
            BS_PUSHBUTTON,IDC_RESET,kPanelX,598,kPanelW,32,app.font);
        CreateControl(app.window,L"BUTTON",L"Spawn box",
            BS_PUSHBUTTON,IDC_ADD_BOX,kPanelX,636,193,30,app.font);
        CreateControl(app.window,L"BUTTON",L"Spawn sphere",
            BS_PUSHBUTTON,IDC_ADD_SPHERE,kPanelX+202,636,198,30,app.font);
        CreateControl(app.window,L"BUTTON",L"FIRE small projectile (camera aim)",
            BS_PUSHBUTTON,IDC_SHOOT,kPanelX,671,kPanelW,32,app.font);

        HWND grid=CreateControl(app.window,L"BUTTON",L"Grid",
            BS_AUTOCHECKBOX,IDC_GRID,kPanelX,710,100,25,app.font);
        HWND axes=CreateControl(app.window,L"BUTTON",L"Body axes",
            BS_AUTOCHECKBOX,IDC_AXES,kPanelX+112,710,120,25,app.font);
        HWND contacts=CreateControl(app.window,L"BUTTON",L"Contacts",
            BS_AUTOCHECKBOX,IDC_CONTACTS,kPanelX+253,710,143,25,app.font);
        SendMessageW(grid,BM_SETCHECK,BST_CHECKED,0);
        SendMessageW(axes,BM_SETCHECK,BST_UNCHECKED,0);
        SendMessageW(contacts,BM_SETCHECK,BST_UNCHECKED,0);
        CreateControl(app.window,L"STATIC",L"Angular",SS_LEFT,-1,
            kPanelX,742,74,23,app.font);
        app.angularCombo=CreateControl(app.window,L"COMBOBOX",L"",
            CBS_DROPDOWNLIST|WS_VSCROLL,IDC_ANGULAR,
            kPanelX+81,738,kPanelW-81,200,app.font);
        const wchar_t* angularMethods[]={
            L"Strang Splitting (default)",
            L"Previous Omega (explicit)",
            L"Next Omega (implicit)",
            L"Midpoint Omega (RK2)",
            L"Euler Equation",
            L"Jolt Style",
            L"Bullet Style",
            L"Gyroscopic Off"
        };
        for(const wchar_t* method:angularMethods)
            SendMessageW(app.angularCombo,CB_ADDSTRING,0,
                reinterpret_cast<LPARAM>(method));
        SendMessageW(app.angularCombo,CB_SETCURSEL,0,0);
        AddText(app,L"LIVE STATS / ENERGY",766,23,true);
        CreateControl(app.window,L"BUTTON",L"Fast PGS (4/8)",
            BS_AUTOCHECKBOX,IDC_FAST_SOLVER,kPanelX+235,766,165,23,app.font);
        app.infoText=AddText(app,L"",790,92);
        app.energyView=CreateControl(app.window,L"PE_ENERGY_GRAPH",L"",
            0,-1,kPanelX,884,kPanelW,68,app.font);
    }

    LRESULT CALLBACK ViewWindowProc(HWND hwnd, UINT msg,
                                    WPARAM wParam, LPARAM lParam)
    {
        AppState* app = gApp;
        switch (msg)
        {
        case WM_KEYDOWN:
            if(app && (wParam=='F') && !((lParam>>30)&1))
            {
                ShootProjectile(*app);
                return 0;
            }
            break;
        case WM_RBUTTONDOWN:
            if (app)
            {
                SetFocus(hwnd);
                app->orbiting = true;
                app->mouseLast = {GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam)};
                app->mouseNow = app->mouseLast;
                SetCapture(hwnd);
            }
            return 0;
        case WM_RBUTTONUP:
            if (app) app->orbiting = false;
            if (GetCapture() == hwnd) ReleaseCapture();
            return 0;
        case WM_LBUTTONDOWN:
            if (app)
            {
                SetFocus(hwnd);
                app->mouseNow = {GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam)};
                BeginGrab(*app, app->mouseNow.x, app->mouseNow.y);
                // Capture only if we successfully picked a body.
                if (app->grab.body) SetCapture(hwnd);
            }
            return 0;
        case WM_LBUTTONUP:
            if (app) ReleaseGrab(*app);
            if (GetCapture() == hwnd) ReleaseCapture();
            return 0;
        case WM_MOUSEMOVE:
            if (app)
            {
                const POINT current{GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam)};
                app->mouseNow = current;
                if (app->orbiting)
                {
                    OrbitCamera(*app, current.x - app->mouseLast.x,
                                current.y - app->mouseLast.y);
                }
                app->mouseLast = current;
                UpdateGrabTarget(*app);
            }
            return 0;
        case WM_MOUSEWHEEL:
            if (app)
            {
                ZoomCamera(*app, GET_WHEEL_DELTA_WPARAM(wParam));
                return 0;
            }
            break;
        case WM_CAPTURECHANGED:
            if (app)
            {
                app->orbiting = false;
                ReleaseGrab(*app);
            }
            break;
        }
        return DefWindowProcW(hwnd, msg, wParam, lParam);
    }
}

int main()
{
    INITCOMMONCONTROLSEX icc = {sizeof(icc), ICC_BAR_CLASSES};
    InitCommonControlsEx(&icc);

    AppState app;
    gApp = &app;
    const HINSTANCE instance = GetModuleHandleW(nullptr);

    WNDCLASSW wc = {};
    wc.lpfnWndProc = MainWindowProc;
    wc.hInstance = instance;
    wc.hCursor = LoadCursorW(nullptr, IDC_ARROW);
    wc.hbrBackground = reinterpret_cast<HBRUSH>(COLOR_BTNFACE + 1);
    wc.lpszClassName = L"PE_GUI_MAIN_WINDOW";
    if (!RegisterClassW(&wc)) return 1;

    WNDCLASSW vc = {};
    vc.lpfnWndProc = ViewWindowProc;
    vc.hInstance = instance;
    vc.hCursor = LoadCursorW(nullptr, IDC_ARROW);
    vc.lpszClassName = L"PE_GUI_RENDER_VIEW";
    if (!RegisterClassW(&vc)) return 1;

    WNDCLASSW ec = {};
    ec.lpfnWndProc=EnergyGraphProc;
    ec.hInstance=instance;
    ec.hCursor=LoadCursorW(nullptr,IDC_ARROW);
    ec.lpszClassName=L"PE_ENERGY_GRAPH";
    if(!RegisterClassW(&ec)) return 1;

    const DWORD style = WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX | WS_CLIPCHILDREN;
    RECT clientRect{0, 0, kClientW, kClientH};
    AdjustWindowRect(&clientRect, style, FALSE);
    app.window = CreateWindowExW(0, wc.lpszClassName,
        L"PhysicsEngine - Spring Oscillator + PGS Test Lab", style,
        CW_USEDEFAULT, CW_USEDEFAULT,
        clientRect.right - clientRect.left,
        clientRect.bottom - clientRect.top,
        nullptr, nullptr, instance, nullptr);
    if (!app.window) return 1;

    app.view = CreateWindowExW(0, vc.lpszClassName, nullptr,
        WS_CHILD | WS_VISIBLE, kViewX, kViewY, kViewW, kViewH,
        app.window, nullptr, instance, nullptr);
    if (!app.view) return 1;

    BuildControls(app);
    ShowWindow(app.window, SW_SHOW);
    UpdateWindow(app.window);

    if (!app.renderer.Initialize(app.view, kViewW, kViewH))
    {
        MessageBoxW(app.window, L"DX11 renderer initialization failed.",
                    L"Error", MB_OK | MB_ICONERROR);
        DestroyWindow(app.window);
        return 1;
    }
    // The renderer viewport must be initialized before orbit-camera fitting.
    ResetScene(app);

    MSG msg = {};
    bool running = true;
    auto previous = std::chrono::steady_clock::now();
    auto previousStats = previous;
    std::size_t reportFrames = 0, reportPhysicsSteps = 0;
    double reportPhysicsMs = 0, reportDrawAndPresentMs = 0;

    while (running)
    {
        while (PeekMessageW(&msg, nullptr, 0, 0, PM_REMOVE))
        {
            if (msg.message == WM_QUIT)
            {
                running = false;
                break;
            }
            TranslateMessage(&msg);
            DispatchMessageW(&msg);
        }
        if (!running) break;

        const double fixedStep=app.fixedStep;
        const PE::Real dt=static_cast<PE::Real>(fixedStep);
        const auto now = std::chrono::steady_clock::now();
        double frameTime = std::chrono::duration<double>(now - previous).count();
        previous = now;
        if (frameTime > 0.05) frameTime = 0.05;
        if (!app.paused)
            app.accumulator += frameTime * app.timeScale;

        const auto physicsBegin = std::chrono::steady_clock::now();
        int substeps = 0;
        while (!app.paused && app.accumulator >= fixedStep && substeps < 16)
        {
            PhysicsStep(app, dt);
            app.accumulator -= fixedStep;
            ++substeps;
        }
        if (substeps == 16) app.accumulator = 0.0;
        if (app.stepOnce)
        {
            PhysicsStep(app, dt);
            app.stepOnce = false;
            ++substeps;
        }
        const auto physicsEnd = std::chrono::steady_clock::now();
        reportPhysicsMs += std::chrono::duration<double, std::milli>(
            physicsEnd - physicsBegin).count();
        reportPhysicsSteps += static_cast<std::size_t>(substeps);
        const auto drawBegin = physicsEnd;

        DrawScene(app);
        if (!app.renderer.EndFrame())
        {
            MessageBoxW(app.window, L"DX11 Present failed.",
                        L"Error", MB_OK | MB_ICONERROR);
            break;
        }

        const auto afterPresent = std::chrono::steady_clock::now();
        reportDrawAndPresentMs += std::chrono::duration<double, std::milli>(
            afterPresent - drawBegin).count();
        ++reportFrames;
        const double statsInterval = std::chrono::duration<double>(
            afterPresent - previousStats).count();
        if (statsInterval >= 0.35)
        {
            wchar_t stats[750];
            const auto& ms=app.scene.manifolds.GetStatistics();
            if(app.scene.IsSpringTest())
            {
                std::swprintf(stats,750,
                    L"SPRING  t %.2fs  steps %d  %s  %.0f Hz\n"
                    L"K %.3f J  U %.3f J  E %.4f J\n"
                    L"Energy %+.3f%%  Peak %.3f%%\n"
                    L"x %.3f m  analytic dx %+.4f m",
                    app.scene.simulationTime,app.scene.physicsSteps,
                    app.paused?L"PAUSED":L"RUNNING",1.0/app.fixedStep,
                    app.scene.SpringKineticEnergy(),app.scene.SpringPotentialEnergy(),
                    app.scene.currentEnergy,app.scene.energyErrorPercent,
                    app.scene.maxAbsoluteEnergyErrorPercent,app.scene.SpringDisplacement(),
                    app.scene.SpringAnalyticPositionError());
            }
            else if (app.scene.IsFreeRotation())
            {
                std::swprintf(stats, 750,
                    L"Time %.2fs  Step %d  %s  %.0f Hz\n"
                    L"Energy %.7g J  Delta %+.5f%%\n"
                    L"Max |Delta| %.4f%%  |L| delta %+.5f%%",
                    app.scene.simulationTime, app.scene.physicsSteps,
                    app.paused ? L"PAUSED" : L"RUNNING", 1.0 / app.fixedStep,
                    app.scene.currentEnergy, app.scene.energyErrorPercent,
                    app.scene.maxAbsoluteEnergyErrorPercent,
                    app.scene.angularMomentumErrorPercent);
            }
            else
            {
                std::swprintf(stats, 750,
                    L"Time %.1fs  Step %d  %s\n"
                    L"Bodies %zu  Contacts %zu  FPS %.0f\n"
                    L"Physics %.2f ms/step  Draw+VSync %.2f ms\n"
                    L"Broad %zu  Narrow %zu  Warm %zu  PGS %s",
                    app.scene.simulationTime, app.scene.physicsSteps,
                    app.paused ? L"PAUSED" : L"RUNNING",
                    app.scene.bodies.size(), app.scene.debugContacts.size(),
                    static_cast<double>(reportFrames) / statsInterval,
                    reportPhysicsSteps ? reportPhysicsMs / reportPhysicsSteps : 0.0,
                    reportFrames ? reportDrawAndPresentMs / reportFrames : 0.0,
                    app.scene.broadPhasePairs, app.scene.narrowPhasePairs,
                    app.scene.solver.GetStatistics().warmStartedCount,
                    app.scene.fastSolver ? L"FAST" : L"FULL");
            }
            SetWindowTextW(app.infoText,stats);
            if(app.energyView) InvalidateRect(app.energyView,nullptr,FALSE);
            previousStats = afterPresent;
            reportFrames = reportPhysicsSteps = 0;
            reportPhysicsMs = reportDrawAndPresentMs = 0;
        }
    }

    // Release renderer COM resources before the child HWND is destroyed.
    // Renderer owns no external GUI dependencies.
    DestroyWindow(app.window);
    DeleteObject(app.font);
    DeleteObject(app.titleFont);
    gApp = nullptr;
    return 0;
}

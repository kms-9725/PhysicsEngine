# PhysicsEngine

C++20으로 처음부터 구현한 3D 강체 물리엔진과, 그 동작을 확인하는 DirectX 11 테스트 랩입니다.

## 주요 구현

- **수학 라이브러리**: vector3, Matrix3, Quaternian, Precision(float,double 선택)
- **강체 동역학**: 위치, 속도, 자세(쿼터니언으로 표현), 각속도, 각운동량, 힘/토크 누적기, 병진 적분기 2종, 회전 적분기 8종 선택 가능
- **충돌 검출**: 구·평면·박스 조합, 박스-박스는 분리축 검사(SAT)와 접촉면 클리핑, Contact manifold 구조
- **접촉 해결**: Projected Gauss-Seidel(PGS), 웜 스타트, 쿨롱 마찰
- **테스트 랩**: 12개 시나리오, 마찰/반발계수/중력 설정, 실시간 에너지 그래프


## 핵심 결과

### 1. 박스 적층 안정성: 순차 해결 → PGS + 접촉 매니폴드

초기 버전은 박스 쌍마다 접촉점을 1개만 만들고, 접촉을 하나씩 순서대로 해결했습니다. 프레임 사이에 접촉 정보도 저장하지 않아서 4단 스택도 흔들리다 무너졌습니다.

현재 버전은 다음 두 가지를 바꿨습니다.

- **접촉 매니폴드**: 접촉면 클리핑으로 박스 쌍마다 접촉점을 최대 4개까지 만듭니다.
- **PGS(Projected Gauss-Seidel)**: 모든 접촉을 함께 반복해서 풀고, 이전 프레임의 충격량에서 시작합니다(웜 스타트).

**개선 전** (개선 전 코드는 [Releases의 Baseline](https://github.com/kms-9725/PhysicsEngine/releases/tag/Baseline)에서 받을 수 있습니다.)


https://github.com/user-attachments/assets/319928d0-61a5-435b-928b-68d18baacdeb




**개선 후**



https://github.com/user-attachments/assets/9cc81907-005d-481b-8079-777370fb8637





### 2. 적분기별 에너지 안정성

**병진 (SPRING 시나리오)**: Explicit Euler는 에너지가 계속 늘어나고, Semi-implicit Euler는 일정한 범위 안에서 유지됩니다.

**Explicit Euler**

https://github.com/user-attachments/assets/0496d35f-4e5d-43d8-8489-c71340356dc1


**Semi-Implicit Euler**



https://github.com/user-attachments/assets/0e5da935-703b-461c-9993-85f99671f060






**회전 (FREE ROTATION 시나리오)**: 기본값인 Strang splitting은 에너지 오차가 0.02% 안에서 진동만 합니다. 
오일러 방정식을 그대로 적분하면 에너지가 발산합니다. 
자이로 항을 끄면 에너지는 보존되지만 중간축 뒤집힘이 사라집니다.

**Strang splitting**



https://github.com/user-attachments/assets/0ab63873-225d-48b3-80ee-53044198fa99



**Euler-Equation**



https://github.com/user-attachments/assets/8bdcee73-3325-4c42-8d8c-5f48b9fceea0



**Gyroscopic Off**



https://github.com/user-attachments/assets/f9af0b60-9eda-4675-94e3-90d3b1074b3b



## 빌드와 실행

**요구 사항**
- Windows 10/11
- Visual Studio 2026(플랫폼 도구 집합 v145)과 "C++를 사용한 데스크톱 개발" 워크로드

**실행 방법**
1. `PhysicsEngine.slnx`를 엽니다.
2. 구성을 `Release | x64`로 바꿉니다.
3. `Ctrl + F5`로 실행합니다.

외부 라이브러리는 필요 없습니다. DirectX 11과 Win32 라이브러리는 코드 안에서 링크됩니다.

> Visual Studio 2022에서는 `PhysicsEngine.vcxproj`를 직접 열고, 프로젝트 속성에서 플랫폼 도구 집합을 v143으로 바꾸면 빌드됩니다.

## 조작법

| 입력 | 동작 |
|---|---|
| 마우스 오른쪽 드래그 | 카메라 회전 |
| 마우스 휠 | 확대·축소 |
| 마우스 왼쪽 드래그 | 물체 잡아 끌기 |
| `F`  | 작은 구 발사 |
| 오른쪽 패널 | 시나리오, 중력·반발·마찰, 적분기, 타임스텝 변경 |


## 테스트 시나리오 설명

1~8번째 시나리오(Gallery, Box / Plane, Sphere / Plane, Sphere / Sphere, Box / Sphere, Box / Box, Four-box stack, Mixed falling bodies)는 기본 도형끼리의 충돌을 확인하는 시나리오라 설명을 생략합니다.

### 9. FREE ROTATION (no gravity, no collision)

회전 적분기에 따른 에너지 안정성 차이를 보기 위한 시나리오입니다.

- 아무 힘도 받지 않고, 0이 아닌 초기 각속도만 가진 채로 시작합니다.
- 오일러 회전 운동방정식 `τ = I·dω/dt + ω × (I·ω)`에서 토크 τ가 0이어도, 자이로스코픽 항 `ω × (I·ω)` 때문에 각속도가 시간에 따라 변합니다.
- Angular 설정에서 Gyroscopic Off를 제외한 나머지 적분기는 모두 자이로스코픽 항을 반영합니다.
- LIVE STATS / ENERGY에서 다음 값을 볼 수 있습니다.
  - 현재 에너지
  - Delta: 시작 에너지 대비 변화율
  - Max |Delta|: 가장 크게 변했을 때의 값
  - Angular momentum magnitude delta: 각운동량 크기의 변화율
  - 에너지 오차 그래프

이론적으로 토크가 없으면 에너지와 각운동량은 변하면 안 되지만, 적분기의 수치 오차 때문에 변합니다. 적분기별 결과는 아래 표에 정리했습니다. (렌더러 없이 엔진만 돌리는 측정 코드로 얻은 값이며, 측정 코드 작성과 실행에 AI의 도움을 받았습니다.)

### 10. TALL STACK, 11. THIN PYRAMID

박스를 쌓았을 때의 안정성을 확인하는 시나리오입니다.

- `Stack boxes / pyramid rows` 슬라이더로 개수를 조절할 수 있습니다.
- 개수를 많이 늘리면 느려집니다. 브로드페이즈는 단순한 sweep-and-prune만 구현되어 있고, 슬리핑·아일랜드 분할 같은 최적화가 적용되지 않았으며, 단일 스레드로 동작하기 때문입니다.
- 추후 멀티스레딩, GPGPU 등의 최적화를 적용할 예정입니다.
- 개선 (2026-10-26): 웜 스타트 계수를 0.9에서 1.0으로 고쳐, 8단 이상 스택이 무너지던 문제를 개선했습니다. 12단 이상은 여전히 천천히 옆으로 흐릅니다.

### 12. SPRING

병진 적분기에 따른 에너지 안정성 차이를 보기 위한 시나리오입니다. `Linear integrator`에서 두 적분기를 선택할 수 있습니다.

| 적분기 | 갱신 순서 | 특징 |
|---|---|---|
| Explicit Euler | `x(n+1) = x(n) + v(n)·dt` → `v(n+1) = v(n) + a(n)·dt` | 위치를 먼저 계산(v(n)으로 위치 적분) |
| Semi-implicit Euler | `v(n+1) = v(n) + a(n)·dt` → `x(n+1) = x(n) + v(n+1)·dt` | 속도를 먼저 계산(v(n+1)으로 위치 적분) |

- **Explicit Euler**: 오차가 한 방향으로 누적되어 에너지가 발산합니다.
- **Semi-implicit Euler**: 오차가 +와 −를 오가며 진동할 뿐 누적되지 않아서, 에너지가 일정한 범위 안에 유지됩니다. 위상공간의 넓이를 보존하는 symplectic 적분기이기 때문입니다.


### 회전 적분기 비교 (AI활용)

FREE ROTATION 시나리오(중력·충돌 없음, 중간축 회전, dt = 1/120 s)에서 측정한 결과입니다.

| 적분기 | 상태 변수 | 60초 후 에너지 오차 | 60초 중 최대 | \|L\| 오차 | 비고 |
|---|---|---:|---:|---:|---|
| **StrangSplitting** (기본값) | L | +0.0004% | 0.0131% | 0% | 오차가 쌓이지 않고 진동만 함. 600초 동안 최대 0.0138% |
| MidpointOmega | L | +0.0051% | 0.0068% | 0% | 2차 정확도지만 천천히 누적됨. 600초 후 +0.078% |
| PreviousOmega | L | +110.76% | 110.76% | 0% | 최소 관성축으로 이동한 뒤 포화 |
| NextOmega | L | −19.41% | 19.41% | 0% | 최대 관성축으로 이동한 뒤 포화 |
| EulerEquation | ω | +103.18% | 103.18% | +42.54% | 계속 증가해 약 121초에 +1,000%를 넘고 이후 발산 |
| JoltStyle | ω | +0.149% | 0.150% | +0.076% | \|L\| 크기를 매 스텝 재정규화 |
| BulletStyle | ω | −40.51% | 40.51% | −22.88% | 암시적 뉴턴 1회. 안정적이지만 에너지가 줄어듦 |
| GyroscopicOff | ω | 0% | 0% | 0% | 자이로 항(ω × Iω)을 무시. 뒤집힘이 사라짐 |

강체 1개를 한 스텝 적분하는 데 걸리는 시간입니다.

| 적분기 | 시간 |
|---|---:|
| PreviousOmega, GyroscopicOff | 약 85 ns |
| EulerEquation, JoltStyle, BulletStyle | 120~150 ns |
| MidpointOmega | 184 ns |
| StrangSplitting | 379 ns |
| NextOmega | 1,085 ns (고정점 반복 10회) |

StrangSplitting은 한 스텝에 주축 회전을 5번 하면서 매번 회전행렬과 월드 관성텐서를 다시 계산하기 때문에 MidpointOmega보다 약 2배 느립니다. 다만 강체 1,000개를 적분해도 약 0.4 ms라서, 박스 210개짜리 피라미드의 충돌 처리(스텝당 약 10 ms)에 비하면 작은 비중입니다.

(측정 환경: Linux, g++ 13.3 `-O2`, Intel Xeon 2.1 GHz, 단일 스레드. 환경에 따라 절대 시간은 달라질 수 있습니다.)


## 프로젝트 구조

```
src/
├── math/        벡터, 행렬, 쿼터니언
├── physics/     강체, 힘 생성기
├── collision/   충돌체(구,평면,박스), 충돌 검출, 접촉 해결
├── sandbox/     테스트 시나리오
├── renderer/    DirectX 11 렌더러
└── main.cpp     Win32 GUI, 메인 루프
```

## 다음 목표
- 관절(힌지, 볼소켓) 구현과 이를 이용한 랙돌·로봇팔
- GJK/EPA로 캡슐·볼록 다면체 등 일반 볼록 형상 충돌 지원
- XPBD 솔버를 구현해 현재 PGS 솔버와 비교하고, 천·로프 시뮬레이션으로 확장
- 연속 충돌 감지(CCD)로 빠른 물체의 터널링 방지
  
## 참고

D3D11 렌더러와 Win32 GUI는 물리 시뮬레이션 결과를 눈으로 확인하기 위한 도구로, AI 도구의 도움을 받아 작성했습니다.

# Raytracer Setup Guide

This guide will help you set up and build the raytracer project on Windows, macOS, and Linux.

## Prerequisites

All platforms require:
- *Git* (for cloning the repository)
- *CMake* (version 3.21 or higher)
- *A C++17 compatible compiler*

## Recommended Build (Ninja, no vcpkg) (currently the only configuration we’ve verified(The windows one specifically).

Use Ninja with system packages for OpenCV and nlohmann/json.

### Install dependencies

- **Windows (MSYS2 MinGW UCRT64)**
 ```
pacman -Syu
pacman -S --needed mingw-w64-ucrt-x86_64-gcc mingw-w64-ucrt-x86_64-cmake mingw-w64-ucrt-x86_64-ninja \
                     mingw-w64-x86_64-opencv mingw-w64-x86_64-nlohmann-json
```
Open the “MSYS2 MinGW UCRT64” shell (echo $MSYSTEM → UCRT64).

-**macOS (Homebrew)**
```
brew install cmake ninja opencv nlohmann-json
```

-**Ubuntu/Debian**
```
sudo apt update
sudo apt install -y build-essential cmake ninja-build libopencv-dev nlohmann-json3-dev
```
-**Configure & build**
```
cmake -S . -B build -G "Ninja" -DCMAKE_BUILD_TYPE=Release
cmake --build build
```
-**Test(Optional)**
```
ctest --test-dir build --output-on-failure
```
-**Run**
```
./build/ray examples/example.json output.png
```
-**Run with paper mode(**Highly reccomended**)**
```
./build/ray examples/example.json output.png --paper
```
Alternative Build (with vcpkg)

Use vcpkg to fetch OpenCV and nlohmann/json, then build with CMake + Ninja.

1. Install vcpkg

* Windows (PowerShell):
```
   git clone [https://github.com/microsoft/vcpkg.git](https://github.com/microsoft/vcpkg.git)
  cd vcpkg
   .\bootstrap-vcpkg.bat
  ```
  * set the env var for this session:
    ```
    $env:VCPKG_ROOT = (Get-Location).Path
    ```

* macOS / Linux:
```
   git clone [https://github.com/microsoft/vcpkg.git](https://github.com/microsoft/vcpkg.git)
   cd vcpkg
   ./bootstrap-vcpkg.sh
   export VCPKG_ROOT="$(pwd)"
```
2. Install dependencies with vcpkg (from the vcpkg directory or any shell where VCPKG_ROOT is set)
```
 "$VCPKG_ROOT/vcpkg" install opencv4 nlohmann-json catch2
```
3. Make sure Ninja and a compiler are installed and on PATH

* Windows (Visual Studio toolchain recommended): install Ninja (e.g., winget install Ninja-build.Ninja), use “x64 Native Tools Command Prompt” or VS Developer PowerShell.
* macOS: brew install ninja
* Ubuntu/Debian: sudo apt install -y ninja-build

4. Configure and build (vcpkg toolchain)

* Windows (PowerShell, MSVC):
```
   cmake -S . -B build-vcpkg -G "Ninja" -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE="$env:VCPKG_ROOT\scripts\buildsystems\vcpkg.cmake" -DVCPKG_TARGET_TRIPLET=x64-windows
   cmake --build build-vcpkg
```
* macOS / Linux:
  ```

  cmake -S . -B build-vcpkg -G "Ninja" -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE="$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake"
  cmake --build build-vcpkg
  ```

5. Run tests (optional)
 ```
 ctest --test-dir build-vcpkg --output-on-failure
```

6. Run the program
```
 ./build-vcpkg/ray examples/example.json output.png
```
Paper mode: 
```
 ./build-vcpkg/ray examples/example.json output.png --paper
```
Notes

* Ensure cmake, ninja, and your compiler are from the same environment (check with: which cmake / ninja / g++ or cl.exe).
* On Windows, vcpkg works most smoothly with the Visual Studio (MSVC) toolchain. If you use MSYS2/MinGW, prefer the non-vcpkg Ninja method you already documented.

  # Additional Features
Paper mode renders a monochrome, cross-hatched image; enable it with the CLI flag --paper (no JSON changes needed), and lighting still controls brightness/contrast.
The pokeball node is a parametric ball with built-in top/bottom halves, belt, ring, and button, so you add one object instead of composing CSG.
Provide position and radius, plus a colors block with per-part material fields (top, bottom, belt, ring, button), each defining diffuse, ambient, specular, and shininess.
Optional geometry controls are belt_half (belt half-thickness as a fraction of radius), ring_width (ring thickness), button_outer (button radius), and button_dir (direction the button faces, e.g., a normalized XYZ vector).
Use it by inserting a "pokeball": { ... } node inside objects, alongside your lights/ground, exactly as in your JSON.

Json format:
**Top level**
- `screen`: sets image plane and camera.
  - `dpi` × `dimensions` → output resolution (pixels).
  - `position` = image plane origin; `observer` = camera/eye. Ensure objects are **in front of the screen** along view direction.
- `medium`: global render controls.
  - `ambient` = base light floor (RGB 0–1); use sparingly.
  - `index` = IOR of surrounding medium.
  - `recursion` = reflection/refraction depth (int ≥ 0).
- `background` (optional): color when rays miss everything.
- `sources`: **array of point lights** with `position` and `intensity` (RGB ≥ 0; values > 1 are fine).

**CRITICAL LIGHTING NOTE — USE MANY LIGHTS**
- This renderer expects **multiple, strong lights**. Start with **3–5 lights** (key, fill, rim, extra fills) and **generous intensities** (tens, not decimals).
- If the frame is dark/black: **increase light intensities and add lights first**; tweak `medium.ambient` only after that.

**Objects & materials**
- `objects`: list of nodes.
  - `sphere`: `position`, `radius`, `color` { `diffuse`, `specular`, `shininess` (and optionally `kd`, `ks`) }.
  - `pokeball`: `position`, `radius`, `colors` for `top/bottom/belt/ring/button` (each with `diffuse/ambient/specular/shininess`), plus optional `belt_half`, `ring_width`, `button_outer`, `button_dir`.
  - `halfSpace`: infinite plane via `position` and `normal` with a `color` block.
  - (If available) transforms/CSG follow the same node pattern.
- All RGB arrays are length 3 in `[0,1]`. `shininess` > 0. **No comments or trailing commas.**

**Quick sanity checks**
- Many lights present (3–5+), with sufficient `intensity`.
- Objects are in front of the camera; screen/observer positioned sensibly.
- `recursion` is adequate (e.g., ≥ 1 if reflections are expected).
- Still dull? Add a fill light, raise key intensity, then slightly increase `medium.ambient`.

Also, Please refer to **Examples**, we have some cool stuff and you can get more insight about the general light levels used.







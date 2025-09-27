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





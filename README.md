# Fishies

[Boids](https://www.red3d.com/cwr/boids/) simulation using CUDA.

## Usage

Requirements:
- CUDA-capable GPU
- [nvcc](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html)

Linux:
```bash
cmake -S . -B build
cmake --build build --config Release
```

Windows (VS Developer Command Prompt):
```bash
cmake -S . -B build -G "Visual Studio 17 2022"
cmake --build build --config Release
```

Run:
```bash
./build/fishies
```

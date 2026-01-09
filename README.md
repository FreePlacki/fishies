# Fishies

[Boids](https://www.red3d.com/cwr/boids/) simulation using CUDA.

## Compilation

Requirements:
- CUDA-capable GPU
- [nvcc](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html)

Linux:
```bash
cmake -S . -B build
cmake --build build --config Release
```

Windows:
```bash
cmake -S . -B build -G "Visual Studio 17 2022"
cmake --build build --config Release
```

## Usage

Run:
```bash
./build/fishies [config.ini]
```

Fish counts can be adjusted using the config file.

Holding `LMB` attracts fish; holding `RMB` repels.

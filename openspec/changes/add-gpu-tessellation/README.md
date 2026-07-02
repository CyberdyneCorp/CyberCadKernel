# add-gpu-tessellation

GPU NURBS/Bezier surface-grid evaluation (points + normals) feeding a CPU
triangulator (topology stays on CPU), plus GPU per-vertex mesh normals, with
GPU results matching a CPU reference within an fp32 tolerance

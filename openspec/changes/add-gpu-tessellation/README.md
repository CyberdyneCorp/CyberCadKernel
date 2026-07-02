# add-gpu-tessellation

GPU NURBS/Bezier surface-grid evaluation (points + normals) wired into
`cc_tessellate` / `cc_face_meshes` behind an additive default-OFF toggle
(`cc_set_gpu_tessellation`), with per-face GPU-eligibility classification and
mandatory OCCT `BRepMesh` fallback (only untrimmed rectangular patches take the GPU
path), GPU/OCCT faces stitched into one mesh, plus GPU per-vertex mesh normals —
GPU-path results matching the OCCT/CPU reference within an fp32 tolerance and
GPU-OFF identical to today

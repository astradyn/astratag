# AstraTag
#### Multi-Range Fiducial Marker for Spacecraft Rendezvous, Proximity Operations and Docking

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.18524681.svg)](https://doi.org/10.5281/zenodo.18524681)

This repository contains the implementation of multi-range, recursive fiducial marker AstraTag. The marker template is derived from square shaped Spidron. The marker is designed for in-orbit operations around cooperative targets. The objective of introducing recurisve feature is to make it suitable for conducting rendezvous and proximity operation. Its three layers provide robustness against a partial occlusion. Below are some of the sample markers. 

<div style="display: flex; justify-content: space-between;">
    <img src="sample/marker_1.png" width="200">
    <img src="sample/marker_2.png" width="200">
    <img src="sample/marker_3.png" width="200">
</div>

To test the marker, a space-like environment was created using a pair of mock-up spacecraft. The mock-ups, inspired by the Aditya-L1 spacecraft and the Indian Space Station module BAS-01, were used as targets. The images below demonstrate a sample result. The setup was placed on a turntable to simulate out-of-plane rotation.

<div style="display:flex; flex-direction:row; justify-content:center; gap:12px; align-items:flex-start; flex-wrap:nowrap;">
   <img src="test/frame_0001.png" alt="frame 0001" style="width:48%; height:auto; display:block; object-fit:contain;">
   <img src="test/frame_0103.png" alt="frame 0103" style="width:48%; height:auto; display:block; object-fit:contain;">
</div>

AstraTag markers can also be used in robotics application in terrestial and underwater environments. 

#### Requirements

- C++17 or later
- OpenCV 4.x (required by CMake)
- CMake 3.16 or later
- A standard build toolchain (gcc/clang, make)

Quick build (recommended)
1. Clone the repository and create a build directory:

```bash
git clone https://github.com/ravikt/astratag.git
cd astratag
mkdir build && cd build
cmake ..
make -j$(nproc)
```

```bash
# from project root
./build/astratag_detect

# or from build/
./astratag_detect
```

#### Citation

If you use AstraTag in your work, please cite it:

```bibtex
@software{astratag,
  title = {AstraTag: Multi-Range Fiducial Marker for Spacecraft RPOD},
  author = {Astradyn Systems LLP},
  year = {2026},
  url = {https://github.com/astradyn/astratag},
  doi = {10.5281/zenodo.18524681},
  license = {Apache-2.0}
}
```

#### License
See `LICENSE` in the repository root for license terms.


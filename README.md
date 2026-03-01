# Bulbit

[![Build](https://github.com/Sopiro/Bulbit/actions/workflows/cmake-multi-platform.yml/badge.svg)](https://github.com/Sopiro/Bulbit/actions/workflows/cmake-multi-platform.yml)

Bulbit is a physically based raytracing renderer.

## Features

Here are the rendering algorithms implemented in Bulbit.

### Integrator
- Unidirectional integrators
  - Path tracing with NEE and continuation ray MIS
  - Volumetric path tracing
  - Light tracing and Volumetric light tracing
  - Naive versions of above algorithms

- Bidirectional integrators
  - Bidirectional path tracing with MIS
  - Volumetric BDPT
  - Vertex Connection and Merging

- Photon mapping integrators
  - Original photon mapping and Volumetric photon mapping
  - Stochastic Progressive Photon Mapping
  - Volumetric SPPM

- ReSTIR integrators
  - Unbiased ReSTIR with hybrid shift (delayed reconnection)
  - GRIS with pairwise MIS
  - ReSTIR Direct lighting
  - ReSTIR Path tracing

- Others
  - Random walk ray tracing
  - Whitted style ray tracing
  - Ambient occlusion

### Light
- Light sources
  - Point, Spot and Directional light
  - Diffuse, Spot and Directional area light
- Light samplers
  - Uniform light sampler, Power light sampler

### Material
- BSDF
  - Lambertian and Specular reflection BRDF
  - Dielectric and Conductor BSDF
  - Energy compansating multi-scattering dielectic and conductor BSDF
  - Thin dielectric BSDF
  - Metallic-Roughness BRDF
  - Energy-preserving Oren–Nayar(EON) BRDF
  - Charlie Sheen BRDF
  - Substrate BRDF
    - Rough dielectric coat on diffuse substrate, with volume layer between them
  - Principled BSDF
    - Support several GLTF 2.0 extensions
    - Clearcoat, transmission, dielectic, conductor, sheen and diffuse
  - Layered BSDF

- BSSRDF
  - Christensen-Burley BSSRDF (Exponential fits)
  - Volumetric random walk based BSSRDF

### Participating Media
- Null-scattering path integral formulation
- Sampling volume scattering with majorant transmittance
- Spectral/Wavelength MIS
- Homogeneous medium
- Nano VDB medium (density and temperature grid)

### Geometry
- Shape
  - Sphere and Triangle mesh
- Acceleration Structure
  - SAH based BVH and Dynamic BVH

### Camera
- Perspective, Orthographic and Spherical camera
- Depth of field effect
- Reconstruction filters
  - Box, Tent and Gaussian filter

## Building
- Install [CMake](https://cmake.org/install/)
- Ensure CMake is in your system `PATH`
- Clone the repository 
  ```bash
  git clone https://github.com/Sopiro/Bulbit
  ```
- Build the project
  ```bash
  mkdir build
  cd build
  cmake ..
  cmake --build .
  ```

## Showcase

Conductor materials with physical parameters
![metals](.github/img/metals.jpg)
Courtesy of [LTE orb](https://github.com/lighttransport/lighttransportequation-orb)  

Subsurface Scattering with Random walk BSSRDF
![sss](.github/img/sss.png)
Courtesy of [Dragon](https://graphics.stanford.edu/data/3Dscanrep/)

Volumetric caustics rendered with Volumetric BDPT  
![caustics2](.github/img/caustics2.jpg)  

SDS path rendered with Vertex Connection and Merging  
![sds](.github/img/sds.jpg)
Courtesy of [Bunny](https://graphics.stanford.edu/data/3Dscanrep/)

Nano VDB volume rendered with volumetric path tracer  
![cloud](.github/img/cloud.jpg)
Courtesy of [Disney cloud](https://github.com/mmp/pbrt-v4-scenes?tab=readme-ov-file#disney-cloud)

ReSTIR PT 1spp spatial_radius=20 spatial_samples=10, no temporal reuse
![1spp](.github/img/kitchen1spp.jpg)
Courtesy of [Jay](https://benedikt-bitterli.me/resources/) 

Experimental Spectral Rendering  
![spectral](.github/img/spectral.jpg)

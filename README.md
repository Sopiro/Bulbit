# Bulbit

Bulbit is a physically based raytracing renderer.

## Features
- Integrator
  - Whitted style, Ambient occlusion and Unidirectional path tracing
- Acceleration structure
  - SAH based BVH and dynamic BVH
- Material
  - Lambertian, Dielectic, Metal and Microfacet
- Light source
  - Point, Directional, Area and Environment lights
- Camera
  - Perspective, Orthographic and Spherical camera
  - Depth of field
- Multi-thread rendering

## Building
- Install [CMake](https://cmake.org/install/)
- Ensure CMake is in the system `PATH`
- Clone the repository `git clone https://github.com/Sopiro/Bulbit`
- Run CMake build script depend on your system
  - Visual Studio: Run `build.bat`
  - Otherwise: Run `build.sh`

## Samples
|![CornellBox](.github/image/render_1000x1000_s1024_d50_t266.3692223s.png)|![CornellBox](.github/image/render_1000x1000_s2048_d50_t554.1794322s.png)|
|--|--|

![RTIOW](.github/image/render_1920x1080_s256_d50_t172.178533s.png)
<!-- ![Materials](.github/image/render_1920x1080_s2048_d2147483647_t885.34s.png) -->
![MIS](.github/image/render_1920x1080_s128_d2147483647_t191.966s.png)
![StanfordModels](.github/image/render_1920x1080_s1024_d2147483647_t490.157s.png)
|![DamagedHelmet1](.github/image/render_1000x1000_s1024_d20_t211.0695558s.png)|![DamagedHelmet2](.github/image/render_1000x1000_s1024_d20_t206.2167148s.png)|
|--|--|  
|![Statue1](.github/image/render_1000x1000_s1024_d2147483647_t366.618s.png)|![Statue2](.github/image/render_1000x1000_s1024_d2147483647_t369.879s.png)|

![AntiqueCamera](.github/image/render_1920x1080_s1024_d2147483647_t233.835s.png)

![Sponza](.github/image/render_1920x1080_s1024_d2147483647_t4680.33s.png)
![Bistro](.github/image/render_1600x900_s1024_d50_t6627.5219105s.png)
![SunTemple](.github/image/render_1600x900_s1024_d50_t1166.8416745s.png)

<!-- ![Ship](.github/image/render_1600x1200_s2048_d2147483647_t2203.37s.png) -->
|![Volume1](.github/image/render_1000x1000_s1000_d2147483647_t887.5372418s.png)|![Volume2](.github/image/render_1000x1000_s1000_d2147483647_t996.1365369s.png)|
|--|--|
|![Volume3](.github/image/render_1000x1000_s1000_d2147483647_t1909.5677176s.png)|![Volume4](.github/image/render_1000x1000_s1000_d2147483647_t3018.2762476s.png)|
<!-- |![Volume5](.github/image/render_1000x1000_s1000_d2147483647_t400.2025311s.png)|![Volume6](.github/image/render_1000x1000_s1000_d2147483647_t989.787817s.png)| -->

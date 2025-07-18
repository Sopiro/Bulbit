#pragma once

#include "bulbit/materials.h"
#include "bulbit/scene.h"

namespace bulbit
{

DiffuseMaterial* CreateDiffuseMaterial(
    Scene& scene, Float reflectance, Float roughness = 0, const SpectrumTexture* normalmap = nullptr, Float alpha = 1
);
DiffuseMaterial* CreateDiffuseMaterial(
    Scene& scene, const Spectrum& reflectance, Float roughness = 0, const SpectrumTexture* normalmap = nullptr, Float alpha = 1
);
DiffuseMaterial* CreateDiffuseMaterial(
    Scene& scene, const std::string& filename, const SpectrumTexture* normalmap = nullptr, Float alpha = 1
);

DielectricMaterial* CreateDielectricMaterial(
    Scene& scene,
    Float eta,
    Float roughness = 0,
    Spectrum reflectance = Spectrum(1),
    bool energy_compensation = true,
    const SpectrumTexture* normalmap = nullptr
);

ThinDielectricMaterial* CreateThinDielectricMaterial(Scene& scene, Float eta, Spectrum reflectance = Spectrum(1));

ConductorMaterial* CreateConductorMaterial(
    Scene& scene,
    const Spectrum& eta,
    const Spectrum& k,
    Float roughness,
    bool energy_compensation = true,
    const SpectrumTexture* normalmap = nullptr,
    Float alpha = 1
);
ConductorMaterial* CreateConductorMaterial(
    Scene& scene,
    const Spectrum& eta,
    const Spectrum& k,
    Float roughness_u,
    Float roughness_v,
    bool energy_compensation = true,
    const SpectrumTexture* normalmap = nullptr,
    Float alpha = 1
);
ConductorMaterial* CreateConductorMaterial(
    Scene& scene,
    const Spectrum& reflectance,
    Float roughness,
    bool energy_compensation = true,
    const SpectrumTexture* normalmap = nullptr,
    Float alpha = 1
);
ConductorMaterial* CreateConductorMaterial(
    Scene& scene,
    const Spectrum& reflectance,
    Float roughness_u,
    Float roughness_v,
    bool energy_compensation = true,
    const SpectrumTexture* normalmap = nullptr,
    Float alpha = 1
);

ClothMaterial* CreateClothMaterial(
    Scene& scene,
    const Spectrum& basecolor,
    const Spectrum& sheen_color,
    Float roughness,
    const SpectrumTexture* normalmap = nullptr,
    Float alpha = 1
);

MetallicRoughnessMaterial* CreateMetallicRoughnessMaterial(
    Scene& scene,
    const Spectrum& basecolor,
    Float metallic,
    Float roughness,
    const Spectrum& emission = Spectrum::black,
    const SpectrumTexture* normalmap = nullptr,
    const FloatTexture* alpha = nullptr
);
MetallicRoughnessMaterial* CreateMetallicRoughnessMaterial(
    Scene& scene,
    const Spectrum& basecolor,
    Float metallic,
    Float u_roughness,
    Float v_roughness,
    const Spectrum& emission = Spectrum::black,
    const SpectrumTexture* normalmap = nullptr,
    const FloatTexture* alpha = nullptr
);

PrincipledMaterial* CreatePrincipledMaterial(
    Scene& scene,
    const Spectrum& basecolor,
    Float metallic = 0,
    Float roughness = 1,
    Float anisotropy = 0,
    Float ior = 1.5f,
    Float transmission = 0,
    Float clearcoat = 0,
    Float clearcoat_roughness = 0,
    const Spectrum& clearcoat_color = Spectrum(1),
    Float sheen = 0,
    Float sheen_roughness = 0,
    const Spectrum& sheen_color = Spectrum(1),
    const Spectrum& emission = Spectrum::black,
    const SpectrumTexture* normalmap = nullptr,
    const FloatTexture* alpha = nullptr
);

SubsurfaceDiffusionMaterial* CreateSubsurfaceDiffusionMaterial(
    Scene& scene,
    const Spectrum& reflectance,
    const Spectrum& mfp,
    Float eta,
    Float roughness,
    const SpectrumTexture* normalmap = nullptr
);

SubsurfaceDiffusionMaterial* CreateSubsurfaceDiffusionMaterial(
    Scene& scene,
    const SpectrumTexture* reflectance,
    const Spectrum& mfp,
    Float eta,
    Float roughness,
    const SpectrumTexture* normalmap = nullptr
);

SubsurfaceRandomWalkMaterial* CreateSubsurfaceRandomWalkMaterial(
    Scene& scene,
    const SpectrumTexture* reflectance,
    const Spectrum& mfp,
    Float eta,
    Float roughness,
    Float g = 0,
    const SpectrumTexture* normalmap = nullptr
);
SubsurfaceRandomWalkMaterial* CreateSubsurfaceRandomWalkMaterial(
    Scene& scene,
    const Spectrum& reflectance,
    const Spectrum& mfp,
    Float eta,
    Float roughness,
    Float g = 0,
    const SpectrumTexture* normalmap = nullptr
);

MixtureMaterial* CreateMixtureMaterial(Scene& scene, const Material* material1, const Material* material2, Float amount);

MirrorMaterial* CreateMirrorMaterial(
    Scene& scene, const Spectrum& reflectance, const SpectrumTexture* normalmap = nullptr, Float alpha = 1
);

DiffuseLightMaterial* CreateDiffuseLightMaterial(Scene& scene, Float emission, bool two_sided = false, Float alpha = 1);
DiffuseLightMaterial* CreateDiffuseLightMaterial(Scene& scene, const Spectrum& emission, bool two_sided = false, Float alpha = 1);

LayeredMaterial* CreateLayeredMaterial(
    Scene& scene,
    const Material* top,
    const Material* bottom,
    bool two_sided = true,
    const Spectrum& albedo = Spectrum(0),
    Float thickness = 1e-4f,
    Float g = 0,
    int32 max_bounces = 16,
    int32 samples = 1,
    const SpectrumTexture* normalmap = nullptr,
    Float alpha = 1
);

const Material* CreateRandomPrincipledMaterial(Scene& scene);

} // namespace bulbit

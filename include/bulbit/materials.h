#pragma once

#include "material.h"

namespace bulbit
{

class DiffuseMaterial : public Material
{
public:
    DiffuseMaterial(
        const SpectrumTexture* reflectance,
        const FloatTexture* roughness = nullptr,
        const SpectrumTexture* normal = nullptr,
        const FloatTexture* alpha = nullptr
    );

    bool GetBSDF(BSDF* bsdf, const Intersection& isect, Allocator& alloc) const;
    bool GetBSSRDF(BSSRDF** bssrdf, const Intersection& isect, Allocator& alloc) const;

    const FloatTexture* GetAlphaTexture() const;
    const SpectrumTexture* GetNormalTexture() const;

public:
    const SpectrumTexture* reflectance;
    const FloatTexture* roughness;

    const SpectrumTexture* normal;
    const FloatTexture* alpha;
};

class MirrorMaterial : public Material
{
public:
    MirrorMaterial(
        const SpectrumTexture* reflectance, const SpectrumTexture* normal = nullptr, const FloatTexture* alpha = nullptr
    );

    bool GetBSDF(BSDF* bsdf, const Intersection& isect, Allocator& alloc) const;
    bool GetBSSRDF(BSSRDF** bssrdf, const Intersection& isect, Allocator& alloc) const;

    const FloatTexture* GetAlphaTexture() const;
    const SpectrumTexture* GetNormalTexture() const;

public:
    const SpectrumTexture* reflectance;

    const SpectrumTexture* normal;
    const FloatTexture* alpha;
};

class DielectricMaterial : public Material
{
public:
    DielectricMaterial(
        Float eta,
        const FloatTexture* u_roughness,
        const FloatTexture* v_roughness,
        const SpectrumTexture* reflectance,
        bool energy_compensation = true,
        const SpectrumTexture* normal = nullptr
    );

    bool GetBSDF(BSDF* bsdf, const Intersection& isect, Allocator& alloc) const;
    bool GetBSSRDF(BSSRDF** bssrdf, const Intersection& isect, Allocator& alloc) const;

    const FloatTexture* GetAlphaTexture() const;
    const SpectrumTexture* GetNormalTexture() const;

private:
    Float eta;

    const FloatTexture* u_roughness;
    const FloatTexture* v_roughness;
    const SpectrumTexture* reflectance;

    bool energy_compensation;

    const SpectrumTexture* normal;
};

class ThinDielectricMaterial : public Material
{
public:
    ThinDielectricMaterial(Float eta, const SpectrumTexture* reflectance);

    bool GetBSDF(BSDF* bsdf, const Intersection& isect, Allocator& alloc) const;
    bool GetBSSRDF(BSSRDF** bssrdf, const Intersection& isect, Allocator& alloc) const;

    const FloatTexture* GetAlphaTexture() const;
    const SpectrumTexture* GetNormalTexture() const;

private:
    Float eta;
    const SpectrumTexture* reflectance;
};

class ClothMaterial : public Material
{
public:
    ClothMaterial(
        const SpectrumTexture* basecolor,
        const SpectrumTexture* sheen_color,
        const FloatTexture* roughness,
        const SpectrumTexture* normal = nullptr,
        const FloatTexture* alpha = nullptr
    );

    bool GetBSDF(BSDF* bsdf, const Intersection& isect, Allocator& alloc) const;
    bool GetBSSRDF(BSSRDF** bssrdf, const Intersection& isect, Allocator& alloc) const;

    const FloatTexture* GetAlphaTexture() const;
    const SpectrumTexture* GetNormalTexture() const;

private:
    const SpectrumTexture* basecolor;
    const SpectrumTexture* sheen_color;
    const FloatTexture* roughness;

    const SpectrumTexture* normal;
    const FloatTexture* alpha;
};

class ConductorMaterial : public Material
{
public:
    ConductorMaterial(
        const SpectrumTexture* eta,
        const SpectrumTexture* k,
        const FloatTexture* u_roughness,
        const FloatTexture* v_roughness,
        const SpectrumTexture* reflectance = nullptr,
        bool energy_compensation = true,
        const SpectrumTexture* normal = nullptr,
        const FloatTexture* alpha = nullptr
    );
    ConductorMaterial(
        const SpectrumTexture* R,                     // physical reflectance
        const FloatTexture* u_roughness,
        const FloatTexture* v_roughness,
        const SpectrumTexture* reflectance = nullptr, // lobe reflectance
        bool energy_compensation = true,
        const SpectrumTexture* normal = nullptr,
        const FloatTexture* alpha = nullptr
    );

    bool GetBSDF(BSDF* bsdf, const Intersection& isect, Allocator& alloc) const;
    bool GetBSSRDF(BSSRDF** bssrdf, const Intersection& isect, Allocator& alloc) const;

    const FloatTexture* GetAlphaTexture() const;
    const SpectrumTexture* GetNormalTexture() const;

private:
    const SpectrumTexture* eta;
    const SpectrumTexture* k;

    const FloatTexture* u_roughness;
    const FloatTexture* v_roughness;

    const SpectrumTexture* reflectance;

    bool energy_compensation;

    const SpectrumTexture* normal;
    const FloatTexture* alpha;
};

class MetallicRoughnessMaterial : public Material
{
public:
    MetallicRoughnessMaterial(
        const SpectrumTexture* basecolor,
        const FloatTexture* metallic,
        const FloatTexture* u_roughness,
        const FloatTexture* v_roughness,
        const SpectrumTexture* normal = nullptr,
        const FloatTexture* alpha = nullptr
    );

    bool GetBSDF(BSDF* bsdf, const Intersection& isect, Allocator& alloc) const;
    bool GetBSSRDF(BSSRDF** bssrdf, const Intersection& isect, Allocator& alloc) const;

    const FloatTexture* GetAlphaTexture() const;
    const SpectrumTexture* GetNormalTexture() const;

private:
    const SpectrumTexture* basecolor;
    const FloatTexture* metallic;
    const FloatTexture* u_roughness;
    const FloatTexture* v_roughness;

    const SpectrumTexture* normal;
    const FloatTexture* alpha;
};

class PrincipledMaterial : public Material
{
public:
    PrincipledMaterial(
        const SpectrumTexture* basecolor,
        const FloatTexture* metallic,
        const FloatTexture* roughness,
        const FloatTexture* anisotropy,
        Float ior,
        const FloatTexture* transmission,
        const FloatTexture* clearcoat,
        const FloatTexture* clearcoat_roughness,
        const SpectrumTexture* clearcoat_color,
        const FloatTexture* sheen,
        const FloatTexture* sheen_roughness,
        const SpectrumTexture* sheen_color,
        const SpectrumTexture* normal = nullptr,
        const FloatTexture* alpha = nullptr
    );

    bool GetBSDF(BSDF* bsdf, const Intersection& isect, Allocator& alloc) const;
    bool GetBSSRDF(BSSRDF** bssrdf, const Intersection& isect, Allocator& alloc) const;

    const FloatTexture* GetAlphaTexture() const;
    const SpectrumTexture* GetNormalTexture() const;

private:
    const SpectrumTexture* basecolor;
    const FloatTexture* metallic;
    const FloatTexture* roughness;
    const FloatTexture* anisotropy;
    Float ior;
    const FloatTexture* transmission;
    const FloatTexture* clearcoat;
    const FloatTexture* clearcoat_roughness;
    const SpectrumTexture* clearcoat_color;
    const FloatTexture* sheen;
    const FloatTexture* sheen_roughness;
    const SpectrumTexture* sheen_color;

    const SpectrumTexture* normal;
    const FloatTexture* alpha;
};

class MixtureMaterial : public Material
{
public:
    MixtureMaterial(
        const Material* material1, const Material* material2, const FloatTexture* mix, const FloatTexture* alpha = nullptr
    );

    bool GetBSDF(BSDF* bsdf, const Intersection& isect, Allocator& alloc) const;
    bool GetBSSRDF(BSSRDF** bssrdf, const Intersection& isect, Allocator& alloc) const;

    const FloatTexture* GetAlphaTexture() const;
    const SpectrumTexture* GetNormalTexture() const;

    const Material* ChooseMaterial(const Intersection& isect, const Vec3& wo) const;

private:
    const Material* materials[2];
    const FloatTexture* mixture_amount;

    const FloatTexture* alpha;
};

class SubsurfaceDiffusionMaterial : public Material
{
public:
    SubsurfaceDiffusionMaterial(
        const SpectrumTexture* reflectance,
        const Spectrum& mfp,
        Float eta,
        const FloatTexture* u_roughness,
        const FloatTexture* v_roughness,
        const SpectrumTexture* normal = nullptr
    );

    bool GetBSDF(BSDF* bsdf, const Intersection& isect, Allocator& alloc) const;
    bool GetBSSRDF(BSSRDF** bssrdf, const Intersection& isect, Allocator& alloc) const;

    const FloatTexture* GetAlphaTexture() const;
    const SpectrumTexture* GetNormalTexture() const;

private:
    const SpectrumTexture* reflectance;

    Spectrum l; // Mean free path
    Float eta;

    const FloatTexture* u_roughness;
    const FloatTexture* v_roughness;

    const SpectrumTexture* normal;
};

class SubsurfaceRandomWalkMaterial : public Material
{
public:
    SubsurfaceRandomWalkMaterial(
        const SpectrumTexture* reflectance,
        const Spectrum& mfp,
        Float eta,
        const FloatTexture* u_roughness,
        const FloatTexture* v_roughness,
        Float g = 0,
        const SpectrumTexture* normal = nullptr
    );

    bool GetBSDF(BSDF* bsdf, const Intersection& isect, Allocator& alloc) const;
    bool GetBSSRDF(BSSRDF** bssrdf, const Intersection& isect, Allocator& alloc) const;

    const FloatTexture* GetAlphaTexture() const;
    const SpectrumTexture* GetNormalTexture() const;

private:
    const SpectrumTexture* reflectance;
    Spectrum l; // Mean free path
    Float eta;
    const FloatTexture* u_roughness;
    const FloatTexture* v_roughness;
    Float g;

    const SpectrumTexture* normal;
};

class LayeredMaterial : public Material
{
public:
    LayeredMaterial(
        const Material* top,
        const Material* bottom,
        bool two_sided,
        const Spectrum& albedo,
        Float thickness,
        Float g = 0,
        int32 max_bounces = 16,
        int32 samples = 1,
        const SpectrumTexture* normal = nullptr,
        const FloatTexture* alpha = nullptr
    );

    bool GetBSDF(BSDF* bsdf, const Intersection& isect, Allocator& alloc) const;
    bool GetBSSRDF(BSSRDF** bssrdf, const Intersection& isect, Allocator& alloc) const;

    const FloatTexture* GetAlphaTexture() const;
    const SpectrumTexture* GetNormalTexture() const;

private:
    const Material* top;
    const Material* bottom;

    bool two_sided;

    Spectrum albedo;

    Float thickness, g;
    int32 max_bounces, samples;

    const SpectrumTexture* normal;
    const FloatTexture* alpha;
};

inline bool Material::GetBSDF(BSDF* bsdf, const Intersection& isect, Allocator& alloc) const
{
    return Dispatch([&](auto mat) { return mat->GetBSDF(bsdf, isect, alloc); });
}

inline bool Material::GetBSSRDF(BSSRDF** bssrdf, const Intersection& isect, Allocator& alloc) const
{
    return Dispatch([&](auto mat) { return mat->GetBSSRDF(bssrdf, isect, alloc); });
}

inline const FloatTexture* Material::GetAlphaTexture() const
{
    return Dispatch([&](auto mat) { return mat->GetAlphaTexture(); });
}

inline const SpectrumTexture* Material::GetNormalTexture() const
{
    return Dispatch([](auto mat) { return mat->GetNormalTexture(); });
}

} // namespace bulbit

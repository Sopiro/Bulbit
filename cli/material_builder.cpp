#include "material_builder.h"
#include "texture_builder.h"

namespace bulbit
{

DiffuseMaterial* CreateDiffuseMaterial(
    Scene& scene, Float reflectance, Float roughness, const Float3Texture* normalmap, Float alpha
)
{
    return scene.CreateMaterial<DiffuseMaterial>(
        CreateSpectrumConstantTexture(scene, reflectance), CreateFloatConstantTexture(scene, roughness), normalmap,
        CreateFloatConstantTexture(scene, alpha)
    );
}

DiffuseMaterial* CreateDiffuseMaterial(
    Scene& scene, const Spectrum& reflectance, Float roughness, const Float3Texture* normalmap, Float alpha
)
{
    return scene.CreateMaterial<DiffuseMaterial>(
        CreateSpectrumConstantTexture(scene, reflectance), CreateFloatConstantTexture(scene, roughness), normalmap,
        CreateFloatConstantTexture(scene, alpha)
    );
}

DiffuseMaterial* CreateDiffuseMaterial(Scene& scene, const std::string& filename, const Float3Texture* normalmap, Float alpha)
{
    return scene.CreateMaterial<DiffuseMaterial>(
        CreateSpectrumImageTexture(scene, filename), nullptr, normalmap, CreateFloatConstantTexture(scene, alpha)
    );
}

DielectricMaterial* CreateDielectricMaterial(
    Scene& scene, Float eta, Float roughness, Spectrum reflectance, bool energy_compensation, const Float3Texture* normalmap
)
{
    return CreateDielectricMaterial(scene, Spectrum::Constant(eta), roughness, reflectance, energy_compensation, normalmap);
}

DielectricMaterial* CreateDielectricMaterial(
    Scene& scene,
    const Spectrum& eta,
    Float roughness,
    Spectrum reflectance,
    bool energy_compensation,
    const Float3Texture* normalmap
)
{
    return scene.CreateMaterial<DielectricMaterial>(
        eta, CreateFloatConstantTexture(scene, roughness), CreateFloatConstantTexture(scene, roughness),
        CreateSpectrumConstantTexture(scene, reflectance), energy_compensation, normalmap
    );
}

DielectricMaterial* CreateDielectricMaterial(
    Scene& scene,
    Float cauchy_a,
    Float cauchy_b,
    Float roughness,
    Spectrum reflectance,
    bool energy_compensation,
    const Float3Texture* normalmap
)
{
    return CreateDielectricMaterial(
        scene, Spectrum::CauchyIOR(cauchy_a, cauchy_b), roughness, reflectance, energy_compensation, normalmap
    );
}

SubstrateMaterial* CreateSubstrateMaterial(
    Scene& scene,
    const Spectrum& reflectance,
    Float roughness,
    Float ior,
    const Spectrum& sigma_a,
    Float thickness,
    const Float3Texture* normalmap,
    Float alpha
)
{
    return scene.CreateMaterial<SubstrateMaterial>(
        CreateSpectrumConstantTexture(scene, reflectance), CreateFloatConstantTexture(scene, roughness), ior, sigma_a, thickness,
        normalmap, CreateFloatConstantTexture(scene, alpha)
    );
}

ThinDielectricMaterial* CreateThinDielectricMaterial(Scene& scene, Float eta, Spectrum reflectance)
{
    return CreateThinDielectricMaterial(scene, Spectrum::Constant(eta), reflectance);
}

ThinDielectricMaterial* CreateThinDielectricMaterial(Scene& scene, const Spectrum& eta, Spectrum reflectance)
{
    return scene.CreateMaterial<ThinDielectricMaterial>(eta, CreateSpectrumConstantTexture(scene, reflectance));
}

ThinDielectricMaterial* CreateThinDielectricMaterial(Scene& scene, Float cauchy_a, Float cauchy_b, Spectrum reflectance)
{
    return CreateThinDielectricMaterial(scene, Spectrum::CauchyIOR(cauchy_a, cauchy_b), reflectance);
}

ConductorMaterial* CreateConductorMaterial(
    Scene& scene,
    const Spectrum& eta,
    const Spectrum& k,
    Float roughness,
    const Spectrum& reflectance,
    bool energy_compensation,
    const Float3Texture* normalmap,
    Float alpha
)
{
    return scene.CreateMaterial<ConductorMaterial>(
        CreateSpectrumConstantTexture(scene, eta), CreateSpectrumConstantTexture(scene, k),
        CreateFloatConstantTexture(scene, roughness), CreateFloatConstantTexture(scene, roughness),
        CreateSpectrumConstantTexture(scene, reflectance), energy_compensation, normalmap,
        CreateFloatConstantTexture(scene, alpha)
    );
}

ConductorMaterial* CreateConductorMaterial(
    Scene& scene,
    const Spectrum& eta,
    const Spectrum& k,
    Float roughness_u,
    Float roughness_v,
    const Spectrum& reflectance,
    bool energy_compensation,
    const Float3Texture* normalmap,
    Float alpha
)
{
    return scene.CreateMaterial<ConductorMaterial>(
        CreateSpectrumConstantTexture(scene, eta), CreateSpectrumConstantTexture(scene, k),
        CreateFloatConstantTexture(scene, roughness_u), CreateFloatConstantTexture(scene, roughness_v),
        CreateSpectrumConstantTexture(scene, reflectance), energy_compensation, normalmap,
        CreateFloatConstantTexture(scene, alpha)
    );
}

ConductorMaterial* CreateConductorMaterial(
    Scene& scene,
    const Spectrum& R,
    Float roughness,
    const Spectrum& reflectance,
    bool energy_compensation,
    const Float3Texture* normalmap,
    Float alpha
)
{
    return scene.CreateMaterial<ConductorMaterial>(
        CreateSpectrumConstantTexture(scene, R), CreateFloatConstantTexture(scene, roughness),
        CreateFloatConstantTexture(scene, roughness), CreateSpectrumConstantTexture(scene, reflectance), energy_compensation,
        normalmap, CreateFloatConstantTexture(scene, alpha)
    );
}

ConductorMaterial* CreateConductorMaterial(
    Scene& scene,
    const Spectrum& R,
    Float roughness_u,
    Float roughness_v,
    const Spectrum& reflectance,
    bool energy_compensation,
    const Float3Texture* normalmap,
    Float alpha
)
{
    return scene.CreateMaterial<ConductorMaterial>(
        CreateSpectrumConstantTexture(scene, R), CreateFloatConstantTexture(scene, roughness_u),
        CreateFloatConstantTexture(scene, roughness_v), CreateSpectrumConstantTexture(scene, reflectance), energy_compensation,
        normalmap, CreateFloatConstantTexture(scene, alpha)
    );
}

ClothMaterial* CreateClothMaterial(
    Scene& scene,
    const Spectrum& basecolor,
    const Spectrum& sheen_color,
    Float roughness,
    const Float3Texture* normalmap,
    Float alpha
)
{
    return scene.CreateMaterial<ClothMaterial>(
        CreateSpectrumConstantTexture(scene, basecolor), CreateSpectrumConstantTexture(scene, sheen_color),
        CreateFloatConstantTexture(scene, roughness), normalmap, CreateFloatConstantTexture(scene, alpha)
    );
}

MetallicRoughnessMaterial* CreateMetallicRoughnessMaterial(
    Scene& scene,
    const Spectrum& basecolor,
    Float metallic,
    Float roughness,
    const Float3Texture* normalmap,
    const FloatTexture* alpha
)
{
    return scene.CreateMaterial<MetallicRoughnessMaterial>(
        CreateSpectrumConstantTexture(scene, basecolor), CreateFloatConstantTexture(scene, metallic),
        CreateFloatConstantTexture(scene, roughness), CreateFloatConstantTexture(scene, roughness), normalmap, alpha
    );
}

MetallicRoughnessMaterial* CreateMetallicRoughnessMaterial(
    Scene& scene,
    const Spectrum& basecolor,
    Float metallic,
    Float u_roughness,
    Float v_roughness,
    const Float3Texture* normalmap,
    const FloatTexture* alpha
)
{
    return scene.CreateMaterial<MetallicRoughnessMaterial>(
        CreateSpectrumConstantTexture(scene, basecolor), CreateFloatConstantTexture(scene, metallic),
        CreateFloatConstantTexture(scene, u_roughness), CreateFloatConstantTexture(scene, v_roughness), normalmap, alpha
    );
}

PrincipledMaterial* CreatePrincipledMaterial(
    Scene& scene,
    const Spectrum& basecolor,
    Float metallic,
    Float roughness,
    Float anisotropy,
    Float ior,
    Float transmission,
    Float clearcoat,
    Float clearcoat_roughness,
    const Spectrum& clearcoat_color,
    Float sheen,
    Float sheen_roughness,
    const Spectrum& sheen_color,
    const Float3Texture* normalmap,
    const FloatTexture* alpha
)
{
    return scene.CreateMaterial<PrincipledMaterial>(
        CreateSpectrumConstantTexture(scene, basecolor), CreateFloatConstantTexture(scene, metallic),
        CreateFloatConstantTexture(scene, roughness), CreateFloatConstantTexture(scene, anisotropy), ior,
        CreateFloatConstantTexture(scene, transmission), CreateFloatConstantTexture(scene, clearcoat),
        CreateFloatConstantTexture(scene, clearcoat_roughness), CreateSpectrumConstantTexture(scene, clearcoat_color),
        CreateFloatConstantTexture(scene, sheen), CreateFloatConstantTexture(scene, sheen_roughness),
        CreateSpectrumConstantTexture(scene, sheen_color), normalmap, alpha
    );
}

SubsurfaceDiffusionMaterial* CreateSubsurfaceDiffusionMaterial(
    Scene& scene, const Spectrum& reflectance, const Spectrum& mfp, Float eta, Float roughness, const Float3Texture* normalmap
)
{
    return scene.CreateMaterial<SubsurfaceDiffusionMaterial>(
        CreateSpectrumConstantTexture(scene, reflectance), mfp, eta, CreateFloatConstantTexture(scene, roughness),
        CreateFloatConstantTexture(scene, roughness), normalmap
    );
}

SubsurfaceDiffusionMaterial* CreateSubsurfaceDiffusionMaterial(
    Scene& scene,
    const SpectrumTexture* reflectance,
    const Spectrum& mfp,
    Float eta,
    Float roughness,
    const Float3Texture* normalmap
)
{
    return scene.CreateMaterial<SubsurfaceDiffusionMaterial>(
        reflectance, mfp, eta, CreateFloatConstantTexture(scene, roughness), CreateFloatConstantTexture(scene, roughness),
        normalmap
    );
}

SubsurfaceRandomWalkMaterial* CreateSubsurfaceRandomWalkMaterial(
    Scene& scene,
    const SpectrumTexture* reflectance,
    const Spectrum& mfp,
    Float eta,
    Float roughness,
    Float g,
    const Float3Texture* normalmap
)
{
    return scene.CreateMaterial<SubsurfaceRandomWalkMaterial>(
        reflectance, mfp, eta, CreateFloatConstantTexture(scene, roughness), CreateFloatConstantTexture(scene, roughness), g,
        normalmap
    );
}

SubsurfaceRandomWalkMaterial* CreateSubsurfaceRandomWalkMaterial(
    Scene& scene,
    const Spectrum& reflectance,
    const Spectrum& mfp,
    Float eta,
    Float roughness,
    Float g,
    const Float3Texture* normalmap
)
{
    return scene.CreateMaterial<SubsurfaceRandomWalkMaterial>(
        CreateSpectrumConstantTexture(scene, reflectance), mfp, eta, CreateFloatConstantTexture(scene, roughness),
        CreateFloatConstantTexture(scene, roughness), g, normalmap
    );
}

MixtureMaterial* CreateMixtureMaterial(Scene& scene, const Material* material1, const Material* material2, Float amount)
{
    return scene.CreateMaterial<MixtureMaterial>(material1, material2, CreateFloatConstantTexture(scene, amount));
}

MirrorMaterial* CreateMirrorMaterial(Scene& scene, const Spectrum& reflectance, const Float3Texture* normalmap, Float alpha)
{
    return scene.CreateMaterial<MirrorMaterial>(
        CreateSpectrumConstantTexture(scene, reflectance), normalmap, CreateFloatConstantTexture(scene, alpha)
    );
}

LayeredMaterial* CreateLayeredMaterial(
    Scene& scene,
    const Material* top,
    const Material* bottom,
    bool two_sided,
    const Spectrum& albedo,
    Float thickness,
    Float g,
    int32 max_bounces,
    int32 samples,
    const Float3Texture* normalmap,
    Float alpha
)
{
    return scene.CreateMaterial<LayeredMaterial>(
        top, bottom, two_sided, albedo, thickness, g, max_bounces, samples, normalmap, CreateFloatConstantTexture(scene, alpha)
    );
}

const Material* CreateRandomPrincipledMaterial(Scene& scene)
{
    // clang-format off
    Spectrum basecolor(Rand(0.0f, 1.0f) * 0.7f, Rand(0.0f, 1.0f) * 0.7f, Rand(0.0f, 1.0f) * 0.7f);
    return CreateMetallicRoughnessMaterial(scene, 
        basecolor,
        Rand() > 0.5f ? 1.0f : 0.0f,
        (Float)std::sqrt(Rand(0.1f, 1.0f)),
        (Float)std::sqrt(Rand(0.1f, 1.0f))
    );
    // clang-format on
}

} // namespace bulbit

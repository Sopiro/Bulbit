#include "bulbit/material.h"
#include "bulbit/scene.h"

using namespace bulbit;

inline const Material* CreateRandomMicrofacetMaterial(Scene& scene)
{
    // clang-format off
    Spectrum basecolor = Spectrum(Rand(0.0, 1.0), Rand(0.0, 1.0), Rand(0.0, 1.0)) * Float(0.7);
    return scene.CreateMaterial<Microfacet>(
        ConstantColor::Create(basecolor),
        ConstantColor::Create(Spectrum(Rand() > 0.5 ? Float(1.0) : Float(0.0))),
        ConstantColor::Create(Spectrum((Float)std::sqrt(Rand(0.0, 1.0)))),
        ConstantColor::Create(basecolor * (Rand() < 0.08 ? Rand(0.0, Float(0.3)) : Float(0.0))),
        ConstantColor::Create(0.5, 0.5, 1.0)
    );
    // clang-format on
}
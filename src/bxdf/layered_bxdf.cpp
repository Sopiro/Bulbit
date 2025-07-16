#include "bulbit/bxdfs.h"

namespace bulbit
{

Spectrum LayeredBxDF::f(Vec3 wo, Vec3 wi, TransportDirection direction) const
{
    Spectrum f(0);

    if (two_sided && wo.z < 0)
    {
        wo.Negate();
        wi.Negate();
    }

    BxDF* enter_interface;

    bool entered_top = two_sided || wo.z > 0;
    if (entered_top)
    {
        enter_interface = top;
    }
    else
    {
        enter_interface = bottom;
    }

    BxDF* exit_interface;
    BxDF* non_exit_interface;

    bool exit_bottom = SameHemisphere(wo, wi) ^ entered_top;
    if (exit_bottom)
    {
        exit_interface = bottom;
        non_exit_interface = top;
    }
    else
    {
        exit_interface = top;
        non_exit_interface = bottom;
    }

    Float z_exit = exit_bottom ? 0 : thickness;

    // Part of BSDF is given by reflection
    if (SameHemisphere(wo, wi))
    {
        f = samples * enter_interface->f(wo, wi, direction);
    }

    // Prepare rng for stochastic BSDF evaluation
    RNG rng(Hash(wo), Hash(wi));

    // Estimate BSDF by unidirectional random walk
    for (int32 s = 0; s < samples; ++s)
    {
        // Sample transmission direction through entrance interface conditioned on wo
        // This is the initial direction of random walk
        BSDFSample wo_sample;
        if (!enter_interface->Sample_f(
                &wo_sample, wo, rng.NextFloat(), { rng.NextFloat(), rng.NextFloat() }, direction, BxDF_SamplingFlags::Transmission
            ))
        {
            continue;
        }

        if (wo_sample.f == Spectrum::black || wo_sample.pdf == 0 || wo_sample.wi.z == 0)
        {
            continue;
        }

        // Sample transmission direction through entrance interface conditioned on wi
        // This is the virtual light direction used for NEE contribution
        BSDFSample wi_sample;
        if (!exit_interface->Sample_f(
                &wi_sample, wi, rng.NextFloat(), { rng.NextFloat(), rng.NextFloat() }, !direction,
                BxDF_SamplingFlags::Transmission
            ))
        {
            continue;
        }

        if (wi_sample.f == Spectrum::black || wi_sample.pdf == 0 || wi_sample.wi.z == 0)
        {
            continue;
        }

        // Path states for random walk BSDF evaluation
        Spectrum beta = wo_sample.f * AbsCosTheta(wo_sample.wi) / wo_sample.pdf;
        Float z = entered_top ? thickness : 0;
        Vec3 w = wo_sample.wi;
        HenyeyGreensteinPhaseFunction phase_function(g);

        constexpr Float rr_min = 0.25f;
        for (int32 bounce = 0; bounce < max_bounces; ++bounce)
        {
            // Possibly terminate random walk with russian roulette
            if (bounce > 3)
            {
                if (Float p = beta.MaxComponent(); p < rr_min)
                {
                    if (rng.NextFloat() > p)
                    {
                        break;
                    }
                    else
                    {
                        beta /= p;
                    }
                }
            }

            if (albedo == Spectrum::black)
            {
                // No medium scattering, advance to next layer boundary
                z = (z == thickness) ? 0 : thickness;

                // Update beta for transmittance
                beta *= Tr(thickness, w);
            }
            else
            {
                constexpr Float sigma_t = 1;
                Float dz = SampleExponential(rng.NextFloat(), sigma_t / std::abs(w.z));

                // Sampled position by homogenious medium scattering
                Float z_p = w.z > 0 ? (z + dz) : (z - dz);

                // It's still inside the medium
                if (0 < z_p && z_p < thickness)
                {
                    // Add MIS combined NEE contiribution from virtual light through exit interface
                    Float w_mis = 1;
                    if (!IsSpecular(exit_interface->Flags()))
                    {
                        w_mis = PowerHeuristic(wi_sample.pdf, phase_function.PDF(-w, -wi_sample.wi));
                    }

                    f += beta * w_mis * albedo * phase_function.p(-w, -wi_sample.wi) * Tr(z_p - z_exit, wi_sample.wi) *
                         wi_sample.f / wi_sample.pdf;

                    // Sample phase function for next path vertex
                    PhaseFunctionSample phase_sample;
                    if (!phase_function.Sample_p(&phase_sample, -w, { rng.NextFloat(), rng.NextFloat() }))
                    {
                        continue;
                    }

                    if (phase_sample.pdf == 0 || phase_sample.wi.z == 0)
                    {
                        continue;
                    }

                    beta *= albedo * phase_sample.p / phase_sample.pdf;
                    w = phase_sample.wi;
                    z = z_p;

                    // Add MIS combined phase function contribution
                    if (!IsSpecular(exit_interface->Flags()))
                    {
                        if ((w.z > 0 && z < z_exit) || (w.z < 0 && z > z_exit))
                        {
                            // Incorporate contribution from exit interface
                            Spectrum f_exit = exit_interface->f(-w, wi, direction);
                            if (f_exit != Spectrum::black)
                            {
                                Float pdf_exit = exit_interface->PDF(-w, wi, direction, BxDF_SamplingFlags::Transmission);
                                w_mis = PowerHeuristic(phase_sample.pdf, pdf_exit);
                                f += beta * w_mis * Tr(z_p - z_exit, phase_sample.wi) * f_exit;
                            }
                        }
                    }

                    // Proceed to sample next path vertex
                    continue;
                }

                // Sampled surface scattering, L_o
                z = Clamp(z_p, 0, thickness);
            }

            // Handle interface sampling
            if (z == z_exit)
            {
                // No light contribution is added since we are doing NEE
                BSDFSample exit_sample;
                if (!exit_interface->Sample_f(
                        &exit_sample, -w, rng.NextFloat(), { rng.NextFloat(), rng.NextFloat() }, direction,
                        BxDF_SamplingFlags::Reflection
                    ))
                {
                    break;
                }

                if (exit_sample.f == Spectrum::black || exit_sample.pdf == 0 || exit_sample.wi.z == 0)
                {
                    break;
                }

                beta *= exit_sample.f * AbsCosTheta(exit_sample.wi) / exit_sample.pdf;
                w = exit_sample.wi;
            }
            else
            {
                if (!IsSpecular(non_exit_interface->Flags()))
                {
                    // Add NEE contribution
                    Float w_mis = 1;
                    if (!IsSpecular((exit_interface->Flags())))
                    {
                        w_mis = PowerHeuristic(wi_sample.pdf, non_exit_interface->PDF(-w, -wi_sample.wi, direction));
                    }

                    f += beta * w_mis * non_exit_interface->f(-w, -wi_sample.wi, direction) * AbsCosTheta(wi_sample.wi) *
                         Tr(thickness, wi_sample.wi) * wi_sample.f / wi_sample.pdf;
                }

                // Sample next reflection direction
                BSDFSample ref_sample;
                if (!non_exit_interface->Sample_f(
                        &ref_sample, -w, rng.NextFloat(), { rng.NextFloat(), rng.NextFloat() }, direction,
                        BxDF_SamplingFlags::Reflection
                    ))
                {
                    break;
                }

                if (ref_sample.f == Spectrum::black || ref_sample.pdf == 0 || ref_sample.wi.z == 0)
                {
                    break;
                }

                // Update path state
                beta *= ref_sample.f * AbsCosTheta(ref_sample.wi) / ref_sample.pdf;
                w = ref_sample.wi;

                if (!IsSpecular(exit_interface->Flags()))
                {
                    // Add light contribution from BSDF sampling
                    Spectrum f_exit = exit_interface->f(-w, wi, direction);
                    if (f_exit != Spectrum::black)
                    {
                        Float w_mis = 1;
                        if (!IsSpecular(non_exit_interface->Flags()))
                        {
                            Float pdf_exit = exit_interface->PDF(-w, wi, direction, BxDF_SamplingFlags::Transmission);
                            w_mis = PowerHeuristic(ref_sample.pdf, pdf_exit);
                        }

                        f += beta * w_mis * Tr(thickness, ref_sample.wi) * f_exit;
                    }
                }
            }
        }
    }

    return f / samples;
}

Float LayeredBxDF::PDF(Vec3 wo, Vec3 wi, TransportDirection direction, BxDF_SamplingFlags flags) const
{
    Float pdf = 0;

    if (two_sided && wo.z < 0)
    {
        wo.Negate();
        wi.Negate();
    }

    bool entered_top = two_sided || wo.z > 0;
    bool reflection = SameHemisphere(wo, wi);

    if (reflection && (flags & BxDF_SamplingFlags::Reflection))
    {
        // Add the first R pdf
        pdf += samples * (entered_top ? top->PDF(wo, wi, direction, BxDF_SamplingFlags::Reflection)
                                      : bottom->PDF(wo, wi, direction, BxDF_SamplingFlags::Reflection));
    }

    RNG rng(Hash(wi), Hash(wo));

    for (int32 s = 0; s < samples; ++s)
    {
        if (reflection && (flags & BxDF_SamplingFlags::Reflection))
        {
            // Estimate the first TRT pdf
            BxDF* r_interface;
            BxDF* t_interface;

            if (entered_top)
            {
                r_interface = bottom;
                t_interface = top;
            }
            else
            {
                r_interface = top;
                t_interface = bottom;
            }

            BSDFSample wo_sample, wi_sample;
            if (t_interface->Sample_f(
                    &wo_sample, wo, rng.NextFloat(), { rng.NextFloat(), rng.NextFloat() }, direction,
                    BxDF_SamplingFlags::Transmission
                ) &&
                t_interface->Sample_f(
                    &wi_sample, wi, rng.NextFloat(), { rng.NextFloat(), rng.NextFloat() }, !direction,
                    BxDF_SamplingFlags::Transmission
                ))
            {
                if (wo_sample.f == Spectrum::black || wo_sample.pdf == 0)
                {
                    continue;
                }

                if (wi_sample.f == Spectrum::black || wi_sample.pdf == 0)
                {
                    continue;
                }

                if (IsSpecular(t_interface->Flags()))
                {
                    pdf += r_interface->PDF(-wo_sample.wi, -wi_sample.wi, direction);
                }
                else
                {
                    BSDFSample r_sample;
                    if (r_interface->Sample_f(
                            &r_sample, -wo_sample.wi, rng.NextFloat(), { rng.NextFloat(), rng.NextFloat() }, direction
                        ))
                    {
                        if (r_sample.f == Spectrum::black || r_sample.pdf == 0)
                        {
                            continue;
                        }

                        if (IsSpecular(r_interface->Flags()))
                        {
                            pdf += t_interface->PDF(-r_sample.wi, wi, direction);
                        }
                        else
                        {
                            Float pdf_r = r_interface->PDF(-wo_sample.wi, -wi_sample.wi, direction);
                            Float w_mis = PowerHeuristic(wi_sample.pdf, pdf_r);
                            pdf += w_mis * pdf_r;

                            Float pdf_t = t_interface->PDF(-r_sample.wi, wi, direction);
                            w_mis = PowerHeuristic(r_sample.pdf, pdf_t);
                            pdf += w_mis * pdf_t;
                        }
                    }
                }
            }
        }
        else if (!reflection && (flags & BxDF_SamplingFlags::Transmission))
        {
            // Estimate the first TT pdf
            BxDF* to_interface;
            BxDF* ti_interface;

            if (entered_top)
            {
                to_interface = top;
                ti_interface = bottom;
            }
            else
            {
                to_interface = bottom;
                ti_interface = top;
            }

            BSDFSample wo_sample;
            if (!to_interface->Sample_f(&wo_sample, wo, rng.NextFloat(), { rng.NextFloat(), rng.NextFloat() }, direction))
            {
                continue;
            }

            if (wo_sample.f == Spectrum::black || wo_sample.pdf == 0 || wo_sample.wi.z == 0 || wo_sample.IsReflection())
            {
                continue;
            }

            BSDFSample wi_sample;
            if (!ti_interface->Sample_f(&wi_sample, wi, rng.NextFloat(), { rng.NextFloat(), rng.NextFloat() }, !direction))
            {
                continue;
            }

            if (wi_sample.f == Spectrum::black || wi_sample.pdf == 0 || wi_sample.wi.z == 0 || wi_sample.IsReflection())
            {
                continue;
            }

            if (IsSpecular(to_interface->Flags()))
            {
                pdf += ti_interface->PDF(-wo_sample.wi, wi, direction);
            }
            else if (IsSpecular(ti_interface->Flags()))
            {
                pdf += to_interface->PDF(wo, -wi_sample.wi, direction);
            }
            else
            {
                // Combine two sampling strategy with constant weights
                Float pdf_ti = ti_interface->PDF(-wo_sample.wi, wi, direction);
                Float pdf_to = to_interface->PDF(wo, -wi_sample.wi, direction);

                pdf += 0.5f * (pdf_to + pdf_ti);
            }
        }
    }

    // Return mixture of two pdfs, uniform sphere pdf approximates diffused multiple scattering
    return Lerp(UniformSpherePDF(), pdf / samples, 0.9f);
}

bool LayeredBxDF::Sample_f(
    BSDFSample* sample, Vec3 wo, Float u0, Point2 u12, TransportDirection direction, BxDF_SamplingFlags flags
) const
{
    bool flipped = false;
    if (two_sided && wo.z < 0)
    {
        wo.Negate();
        flipped = true;
    }

    // Sample BSDF at entrance interface to get inital direction
    bool entered_top = two_sided || wo.z > 0;

    bool sampled = false;
    BSDFSample wo_sample;
    if (entered_top)
    {
        sampled = top->Sample_f(&wo_sample, wo, u0, u12, direction);
    }
    else
    {
        sampled = bottom->Sample_f(&wo_sample, wo, u0, u12, direction);
    }

    if (!sampled || wo_sample.f == Spectrum::black || wo_sample.pdf == 0 || wo_sample.wi.z == 0)
    {
        return false;
    }

    if (wo_sample.IsReflection())
    {
        wo_sample.is_stochastic = true;

        if (flipped)
        {
            wo_sample.wi.Negate();
        }

        if (!(flags & BxDF_SamplingFlags::Reflection))
        {
            return false;
        }

        *sample = wo_sample;
        return true;
    }

    // Initial unidirectional random walk direction
    Vec3 w = wo_sample.wi;
    bool was_specular = wo_sample.IsSpecular();

    RNG rng(Hash(wo), Hash(u0, u12));

    // Path states intialized with initial w sample
    Spectrum f = wo_sample.f * AbsCosTheta(wo_sample.wi);
    Float pdf = wo_sample.pdf;

    Float z = entered_top ? thickness : 0;
    HenyeyGreensteinPhaseFunction phase_function(g);

    constexpr Float rr_min = 0.25f;
    for (int32 bounce = 0; bounce < max_bounces; ++bounce)
    {
        // Start random walk

        // Possibly terminate random walk with russian roulette
        if (bounce > 3)
        {
            if (Float beta = f.MaxComponent() / pdf; beta < rr_min)
            {
                if (rng.NextFloat() > beta)
                {
                    return false;
                }
                else
                {
                    pdf /= beta;
                }
            }
        }

        if (w.z == 0)
        {
            return false;
        }

        if (albedo == Spectrum::black)
        {
            // No medium scattering, advance to next layer boundary
            z = (z == thickness) ? 0 : thickness;

            // Update beta for transmittance
            f *= Tr(thickness, w);
        }
        else
        {
            // Sample scattering event and update path states
            constexpr Float sigma_t = 1;
            Float dz = SampleExponential(rng.NextFloat(), sigma_t / std::abs(w.z));
            Float z_p = w.z > 0 ? (z + dz) : (z - dz);

            // Sampled medium scattering
            if (0 < z_p && z_p < thickness)
            {
                // Sample phase function for next path vertex
                PhaseFunctionSample phase_sample;
                if (!phase_function.Sample_p(&phase_sample, -w, { rng.NextFloat(), rng.NextFloat() }))
                {
                    return false;
                }

                // Update path states for volume scattering vertex
                f *= albedo * phase_sample.p;
                pdf *= phase_sample.pdf;
                was_specular = false;

                w = phase_sample.wi;
                z = z_p;

                // Proceed to sample next path vertex
                continue;
            }
            else // Sampled surface scattering
            {
                z = Clamp(z_p, 0, thickness);
            }
        }

        BxDF* interface;
        if (z == 0)
        {
            interface = bottom;
        }
        else
        {
            interface = top;
        }

        BSDFSample bsdf_sample;
        if (!interface->Sample_f(&bsdf_sample, -w, rng.NextFloat(), { rng.NextFloat(), rng.NextFloat() }, direction))
        {
            return false;
        }

        if (bsdf_sample.f == Spectrum::black || bsdf_sample.pdf == 0 || bsdf_sample.wi.z == 0)
        {
            return false;
        }

        // Update path states for surface scattering vertex
        f *= bsdf_sample.f;
        pdf *= bsdf_sample.pdf;
        was_specular &= bsdf_sample.IsSpecular();
        w = bsdf_sample.wi;

        // Stop random walk and return sample if path has left the layers
        if (bsdf_sample.IsTransmission())
        {
            BxDF_Flags sample_flags;
            sample_flags = SameHemisphere(wo, w) ? BxDF_Flags::Reflection : BxDF_Flags::Transmission;
            sample_flags |= was_specular ? BxDF_Flags::Specular : BxDF_Flags::Glossy;

            if (flipped)
            {
                w.Negate();
            }

            *sample = BSDFSample(f, w, pdf, sample_flags, 1, true);
            return true;
        }

        // Don't forget the cosine term on surface scattering
        f *= AbsCosTheta(bsdf_sample.wi);
    }

    return false;
}

} // namespace bulbit

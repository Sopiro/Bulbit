#if defined(_WIN32) && defined(_DEBUG)
#include <crtdbg.h>
#endif

#include "bulbit/bulbit.h"
#include "samples.h"

int main(int argc, char* argv[])
{
#if defined(_WIN32) && defined(_DEBUG)
    // Enable memory-leak reports
    _CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
#endif

    ThreadPool::global_thread_pool.reset(new ThreadPool(std::thread::hardware_concurrency()));

    std::cout << "Loading scene.." << std::endl;
    Timer timer;

    RendererInfo ri;
    // bool result = LoadScene(&ri, "C:/Users/sopir/Desktop/scenes/bedroom/scene_v3.xml");
    bool result = Sample::Get(&ri, "cornell-box-fog");
    if (!result)
    {
        std::cout << "Sample not found!" << std::endl;
        return 0;
    }

    timer.Mark();
    std::cout << "Scene loading: " << timer.Get() << "s" << std::endl;
    std::cout << "Primitives: " << ri.scene.GetPrimitives().size() << std::endl;
    std::cout << "Lights: " << ri.scene.GetLights().size() << std::endl;

    const ReconFilterInfo& fi = ri.camera_info.film_info.recon_filter_info;

    std::unique_ptr<Filter> pixel_filter;
    switch (fi.type)
    {
    case ReconFilterType::box:
        pixel_filter = std::make_unique<BoxFilter>(fi.extent);
        break;
    case ReconFilterType::tent:
        pixel_filter = std::make_unique<TentFilter>(fi.extent);
        break;
    case ReconFilterType::gaussian:
        pixel_filter = std::make_unique<GaussianFilter>(fi.extent, fi.gaussian_stddev);
        break;

    default:
        return 0;
    }

    const CameraInfo& ci = ri.camera_info;

    std::unique_ptr<Camera> camera;
    const Medium* camera_medium = nullptr; // TODO: handle it correctly
    switch (ci.type)
    {
    case CameraType::perspective:
        camera = std::make_unique<PerspectiveCamera>(
            ci.transform, ci.fov, ci.aperture_radius, ci.focus_distance, ci.film_info.resolution, camera_medium,
            pixel_filter.get()
        );
        break;
    case CameraType::orthographic:
        camera = std::make_unique<OrthographicCamera>(
            ci.transform, ci.viewport_size, ci.film_info.resolution.x, camera_medium, pixel_filter.get()
        );
        break;
    case CameraType::spherical:
        camera = std::make_unique<SphericalCamera>(ci.transform, ci.film_info.resolution, camera_medium, pixel_filter.get());
        break;

    default:
        return 0;
    }

    std::cout << "Building acceleration structure.." << std::endl;
    BVH accel(ri.scene.GetPrimitives());
    timer.Mark();
    std::cout << "Acceleration structure build: " << timer.Get() << "s" << std::endl;

    std::unique_ptr<Sampler> sampler;
    int32 spp = ci.sampler_info.spp;

    switch (ci.sampler_info.type)
    {
    case SamplerType::independent:
        sampler = std::make_unique<IndependentSampler>(spp);
        break;
    case SamplerType::stratified:
    {
        int32 h = std::sqrt(spp);
        sampler = std::make_unique<StratifiedSampler>(h, h, true);
    }
    break;

    default:
        return 0;
    }

    const IntegratorInfo& ii = ri.integrator_info;
    int32 max_bounces = ii.max_bounces;
    int32 rr_min_bounces = ii.rr_min_bounces;

    std::unique_ptr<Integrator> integrator;
    switch (ii.type)
    {
    case IntegratorType::path:
        integrator = std::make_unique<PathIntegrator>(
            &accel, ri.scene.GetLights(), sampler.get(), max_bounces, rr_min_bounces, ii.regularize_bsdf
        );
        break;
    case IntegratorType::vol_path:
        integrator = std::make_unique<VolPathIntegrator>(
            &accel, ri.scene.GetLights(), sampler.get(), max_bounces, rr_min_bounces, ii.regularize_bsdf
        );
        break;
    case IntegratorType::light_path:
        integrator =
            std::make_unique<LightPathIntegrator>(&accel, ri.scene.GetLights(), sampler.get(), max_bounces, rr_min_bounces);
        break;
    case IntegratorType::light_vol_path:
        integrator =
            std::make_unique<LightVolPathIntegrator>(&accel, ri.scene.GetLights(), sampler.get(), max_bounces, rr_min_bounces);
        break;
    case IntegratorType::bdpt:
        integrator = std::make_unique<BiDirectionalPathIntegrator>(
            &accel, ri.scene.GetLights(), sampler.get(), max_bounces, rr_min_bounces
        );
        break;
    case IntegratorType::vol_bdpt:
        integrator = std::make_unique<BiDirectionalVolPathIntegrator>(
            &accel, ri.scene.GetLights(), sampler.get(), max_bounces, rr_min_bounces
        );
        break;
    case IntegratorType::pm:
        integrator = std::make_unique<PhotonMappingIntegrator>(
            &accel, ri.scene.GetLights(), sampler.get(), max_bounces, ii.n_photons, ii.initial_radius
        );
        break;
    case IntegratorType::sppm:
        integrator = std::make_unique<SPPMIntegrator>(
            &accel, ri.scene.GetLights(), sampler.get(), max_bounces, ii.n_photons, ii.initial_radius
        );
        break;
    case IntegratorType::naive_path:
        integrator =
            std::make_unique<NaivePathIntegrator>(&accel, ri.scene.GetLights(), sampler.get(), max_bounces, rr_min_bounces);
        break;
    case IntegratorType::naive_vol_path:
        integrator =
            std::make_unique<NaiveVolPathIntegrator>(&accel, ri.scene.GetLights(), sampler.get(), max_bounces, rr_min_bounces);
        break;
    case IntegratorType::random_walk:
        integrator = std::make_unique<RandomWalkIntegrator>(&accel, ri.scene.GetLights(), sampler.get(), max_bounces);
        break;
    case IntegratorType::ao:
        integrator = std::make_unique<AmbientOcclusion>(&accel, ri.scene.GetLights(), sampler.get(), ii.ao_range);
        break;
    case IntegratorType::albedo:
        integrator = std::make_unique<AlbedoIntegrator>(&accel, ri.scene.GetLights(), sampler.get());
        break;
    case IntegratorType::debug:
        integrator = std::make_unique<DebugIntegrator>(&accel, ri.scene.GetLights(), sampler.get());
        break;

    default:
        return 0;
    }

    std::unique_ptr<Rendering> rendering = integrator->Render(camera.get());
    rendering->WaitAndLogProgress();
    timer.Mark();
    double render_time = timer.Get();
    std::cout << "\nComplete: " << render_time << 's' << std::endl;

    const Film& film = rendering->GetFilm();
    Image3 image = film.GetRenderedImage();

    std::string filename =
        std::format("bulbit_render_{}x{}_s{}_d{}_t{}s.hdr", image.width, image.height, spp, max_bounces, render_time);
    WriteImage(image, filename.c_str());

#if _DEBUG
    return 0;
#else
    return system(filename.c_str());
#endif
}
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

    // SceneInfo si = LoadScene("C:/Users/sopir/Desktop/spaceship/scene_v3.xml");

    std::cout << "Loading scene.." << std::endl;
    Timer timer;
    SceneInfo si = Sample::Get("cornell-box");
    if (!si)
    {
        std::cout << "Sample not found!" << std::endl;
        return 0;
    }

    timer.Mark();
    std::cout << "Scene loading: " << timer.Get() << "s" << std::endl;
    std::cout << "Primitives: " << si.scene->GetPrimitives().size() << std::endl;
    std::cout << "Lights: " << si.scene->GetLights().size() << std::endl;

    std::unique_ptr<Camera> camera;

    switch (si.camera_info.type)
    {
    case CameraType::perspective:
    {
        camera = std::make_unique<PerspectiveCamera>(
            si.camera_info.transform, si.camera_info.fov, si.camera_info.aperture_radius, si.camera_info.focus_distance,
            si.camera_info.film_info.resolution
        );
    }
    break;

    default:
        return 0;
    }

    std::cout << "Building acceleration structure.." << std::endl;

    BVH accel(si.scene->GetPrimitives());
    std::unique_ptr<Sampler> sampler;

    timer.Mark();
    std::cout << "Acceleration structure build: " << timer.Get() << "s" << std::endl;

    int32 spp = si.camera_info.sampler_info.spp;

    switch (si.camera_info.sampler_info.type)
    {
    case SamplerType::independent:
    {
        sampler = std::make_unique<IndependentSampler>(spp);
    }
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

    std::unique_ptr<Integrator> integrator;
    int32 max_bounces = si.renderer_info.max_bounces;

    switch (si.renderer_info.type)
    {
    case IntegratorType::path:
        integrator = std::make_unique<PathIntegrator>(
            &accel, si.scene->GetLights(), sampler.get(), max_bounces, si.renderer_info.regularize_bsdf
        );
        break;
    case IntegratorType::vol_path:
        integrator = std::make_unique<VolPathIntegrator>(
            &accel, si.scene->GetLights(), sampler.get(), max_bounces, si.renderer_info.regularize_bsdf
        );
        break;
    case IntegratorType::light_path:
        integrator = std::make_unique<LightPathIntegrator>(&accel, si.scene->GetLights(), sampler.get(), max_bounces);
        break;
    case IntegratorType::light_vol_path:
        integrator = std::make_unique<LightVolPathIntegrator>(&accel, si.scene->GetLights(), sampler.get(), max_bounces);
        break;
    case IntegratorType::bdpt:
        integrator = std::make_unique<BiDirectionalPathIntegrator>(&accel, si.scene->GetLights(), sampler.get(), max_bounces);
        break;
    case IntegratorType::pm:
        integrator = std::make_unique<PhotonMappingIntegrator>(
            &accel, si.scene->GetLights(), sampler.get(), max_bounces, si.renderer_info.n_photons, si.renderer_info.initial_radius
        );
        break;
    case IntegratorType::sppm:
        integrator = std::make_unique<SPPMIntegrator>(
            &accel, si.scene->GetLights(), sampler.get(), max_bounces, si.renderer_info.n_photons, si.renderer_info.initial_radius
        );
        break;
    case IntegratorType::naive_path:
        integrator = std::make_unique<NaivePathIntegrator>(&accel, si.scene->GetLights(), sampler.get(), max_bounces);
        break;
    case IntegratorType::naive_vol_path:
        integrator = std::make_unique<NaiveVolPathIntegrator>(&accel, si.scene->GetLights(), sampler.get(), max_bounces);
        break;
    case IntegratorType::random_walk:
        integrator = std::make_unique<RandomWalkIntegrator>(&accel, si.scene->GetLights(), sampler.get(), max_bounces);
        break;
    case IntegratorType::ao:
        integrator = std::make_unique<AmbientOcclusion>(&accel, si.scene->GetLights(), sampler.get(), si.renderer_info.ao_range);
        break;
    case IntegratorType::albedo:
        integrator = std::make_unique<AlbedoIntegrator>(&accel, si.scene->GetLights(), sampler.get());
        break;
    case IntegratorType::debug:
        integrator = std::make_unique<DebugIntegrator>(&accel, si.scene->GetLights(), sampler.get());
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
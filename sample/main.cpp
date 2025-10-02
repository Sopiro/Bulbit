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
    bool result = Sample::Get(&ri, "cornell-box");
    if (!result)
    {
        std::cout << "Sample not found!" << std::endl;
        return 0;
    }

    timer.Mark();
    std::cout << "Scene loading: " << timer.Get() << "s" << std::endl;
    std::cout << "Primitives: " << ri.scene.GetPrimitives().size() << ", Lights: " << ri.scene.GetLights().size() << std::endl;

    Allocator alloc;

    Filter* filter = Filter::Create(alloc, ri.camera_info.film_info.filter_info);
    if (!filter)
    {
        std::cout << "Faild to create pixel filter" << std::endl;
        return 0;
    }

    Camera* camera = Camera::Create(alloc, ri.camera_info, filter);
    if (!camera)
    {
        std::cout << "Faild to create camera" << std::endl;
        return 0;
    }

    std::cout << "Building acceleration structure.." << std::endl;
    BVH accel(ri.scene.GetPrimitives());
    timer.Mark();
    std::cout << "Acceleration structure build: " << timer.Get() << "s" << std::endl;

    Sampler* sampler = Sampler::Create(alloc, ri.camera_info.sampler_info);
    if (!sampler)
    {
        std::cout << "Faild to sampler" << std::endl;
        return 0;
    }

    Integrator* integrator = Integrator::Create(alloc, ri.integrator_info, &accel, ri.scene.GetLights(), sampler);
    if (!integrator)
    {
        std::cout << "Faild to create integrator" << std::endl;
        return 0;
    }

    Rendering* rendering = integrator->Render(alloc, camera);
    rendering->WaitAndLogProgress();
    timer.Mark();
    double render_time = timer.Get();
    std::cout << "\nComplete: " << render_time << 's' << std::endl;

    Image3 image = rendering->GetFilm().GetRenderedImage();

    std::string filename = ri.camera_info.film_info.filename;
    if (filename.size() == 0)
    {
        filename = std::format(
            "bulbit_render_{}x{}_s{}_d{}_t{}s.hdr", image.width, image.height, ri.camera_info.sampler_info.spp,
            ri.integrator_info.max_bounces, render_time
        );
    }

    WriteImage(image, filename.c_str());

    alloc.delete_object(rendering);
    alloc.delete_object(integrator);
    alloc.delete_object(sampler);
    alloc.delete_object(camera);
    alloc.delete_object(filter);

#if _DEBUG
    return 0;
#else
    return system(filename.c_str());
#endif
}
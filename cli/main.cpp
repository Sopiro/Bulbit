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

    if (argc <= 1 || std::string(argv[1]) == "--help")
    {
        std::cout << "\nUsage:\n";
        std::cout << "  bbcli [options] <scene.xml | sample_name> [<scene.xml | sample_name>...]\n\n";
        std::cout << "Options:\n";
        std::cout << "  -t <num_threads>          Number of threads to use (default: hardware concurrency)\n";
        std::cout << "  -o <output_file>          Override output file name (default: value from scene or auto-generated)\n";
        std::cout << "  -s <samples-per-pixel>    Override number of samples per pixel (default: value from scene)\n";
        std::cout << "  -b <max_bounces>          Override number of maximum bounces (default: value from scene)\n";
        std::cout << "  -r <image_scale>          Scale output image (default: 1)\n";
        std::cout << "  --list-samples            Show list of available built-in sample scenes\n";
        std::cout << "  --help                    Show this help message\n\n";
        std::cout << "Arguments:\n";
        std::cout << "  <scene.xml>               Path to scene file to render\n";
        std::cout << "  <sample_name>             Name of built-in sample scene (e.g., cornell-box)\n\n";
        std::cout << "Examples:\n";
        std::cout << "  bbcli scene.xml\n";
        std::cout << "  bbcli cornell-box\n";
        std::cout << "  bbcli -r 2 -o render.hdr cornell-box cornell-box-caustics\n";
        return 0;
    }

    int32 num_threads = std::thread::hardware_concurrency();
    std::string output_file = "";
    int32 spp = 0;
    int32 max_bounces = -1;
    float scale = 1;

    std::vector<std::string> inputs;

    for (int32 i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];

        if (arg == "-t" && i + 1 < argc)
        {
            num_threads = std::stoi(argv[++i]);
        }
        else if (arg == "-o" && i + 1 < argc)
        {
            output_file = argv[++i];
        }
        else if (arg == "-s" && i + 1 < argc)
        {
            spp = std::stoi(argv[++i]);
        }
        else if (arg == "-b" && i + 1 < argc)
        {
            max_bounces = std::stoi(argv[++i]);
        }
        else if (arg == "-r" && i + 1 < argc)
        {
            scale = std::stof(argv[++i]);
        }
        else if (arg == "--list-samples")
        {
            std::cout << "Available built-in samples:\n";
            for (const auto& sample : Sample::samples)
            {
                std::cout << "  - " << sample.first << '\n';
            }
            return 0;
        }
        else
        {
            inputs.push_back(arg);
        }
    }

    if (inputs.size() == 0)
    {
        std::cerr << "Error: No scene file or sample name provided.\n";
        return 1;
    }

    ThreadPool::global_thread_pool.reset(new ThreadPool(num_threads));

    for (const std::string& input : inputs)
    {
        std::cout << "\n--- " << input << " ---\n";
        std::cout << "\rLoading scene.. " << std::flush;

        Timer timer;
        RendererInfo ri;

        bool result = Sample::Get(&ri, input);
        if (!result)
        {
            result = LoadScene(&ri, input.c_str());
        }

        if (!result)
        {
            std::cerr << "Failed to load scene or sample: " << input << '\n';
            continue;
        }

        if (spp > 0) ri.camera_info.sampler_info.spp = spp;
        if (max_bounces >= 0) ri.integrator_info.max_bounces = max_bounces;
        ri.camera_info.film_info.resolution *= scale;

        std::cout << "\rLoading scene.. " << timer.Mark() << "s" << std::endl;
        std::cout << "Primitives: " << ri.scene.GetPrimitives().size() << ", Lights: " << ri.scene.GetLights().size()
                  << std::endl;

        std::cout << "\rBuilding acceleration structure.. " << std::flush;
        BVH accel(ri.scene.GetPrimitives());
        std::cout << "\rBuilding acceleration structure.. " << timer.Mark() << "s" << std::endl;

        Allocator alloc;

        Filter* filter = Filter::Create(alloc, ri.camera_info.film_info.filter_info);
        if (!filter)
        {
            std::cerr << "Failed to create pixel filter" << std::endl;
            return 0;
        }

        Camera* camera = Camera::Create(alloc, ri.camera_info, filter);
        if (!camera)
        {
            std::cerr << "Failed to create camera" << std::endl;
            return 0;
        }

        Sampler* sampler = Sampler::Create(alloc, ri.camera_info.sampler_info);
        if (!sampler)
        {
            std::cerr << "Failed to sampler" << std::endl;
            return 0;
        }

        Integrator* integrator = Integrator::Create(alloc, ri.integrator_info, &accel, ri.scene.GetLights(), sampler);
        if (!integrator)
        {
            std::cerr << "Failed to create integrator" << std::endl;
            return 0;
        }

        Rendering* rendering = integrator->Render(alloc, camera);
        rendering->WaitAndLogProgress();

        double render_time = timer.Mark();
        std::cout << "\nComplete " << render_time << 's' << std::endl;

        Image3 image = rendering->GetFilm().GetRenderedImage();

        std::string filename = output_file.size() == 0 ? ri.camera_info.film_info.filename : output_file;
        if (filename.size() == 0)
        {
            filename = std::format(
                "bulbit_render_{}x{}_s{}_d{}_t{}s.hdr", image.width, image.height, ri.camera_info.sampler_info.spp,
                ri.integrator_info.max_bounces, render_time
            );
        }

        if (!(filename.ends_with(".jpg") || filename.ends_with(".png") || filename.ends_with(".hdr")))
        {
            std::cout << "\nWarning: Unsupported file format for '" << input << "'\n";
            std::cout << "Supported formats are: .jpg, .png, .hdr\n";
            std::cout << "Saving as PNG instead.\n";

            std::filesystem::path original(filename);
            filename = original.replace_extension(".png").string();
        }

        filename = NextFileName(filename).string();
        WriteImage(image, filename.c_str());

        alloc.delete_object(rendering);
        alloc.delete_object(integrator);
        alloc.delete_object(sampler);
        alloc.delete_object(camera);
        alloc.delete_object(filter);
    }

    return 0;
}
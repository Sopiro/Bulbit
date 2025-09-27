#include "mistuba_loader.h"
#include "loader.h"
#include "material_builder.h"

#include "pugixml.hpp"
#include <regex>

namespace fs = std::filesystem;

namespace bulbit
{

static const int32 g_default_fov = 35;
static const int32 g_default_width = 640;
static const int32 g_default_height = 480;
static const std::string g_default_filename = "render.hdr";

using DefaultMap = std::unordered_map<std::string, std::string>;
using MaterialMap = std::unordered_map<std::string, const Material*>;

static float VFovToHFov(float vfov, float aspect)
{
    return 2.0f * std::atan(aspect * std::tan(DegToRad(vfov) * 0.5f));
}

static float HFovToVFov(float hfov, float aspect)
{
    return 2.0f * std::atan(std::tan(DegToRad(hfov) * 0.5f) / aspect);
}

static void ParseDefaultMap(pugi::xml_node node, DefaultMap& dm)
{
    if (node.attribute("name"))
    {
        std::string name = node.attribute("name").value();
        if (node.attribute("value"))
        {
            std::string value = node.attribute("value").value();
            dm[name] = value;
        }
    }
}

static std::string ParseString(pugi::xml_attribute attr, const DefaultMap& dm)
{
    std::string str = attr.value();
    size_t len = str.length();
    if (len == 0)
    {
        return "";
    }

    if (len == 1 || str[0] != '$')
    {
        return str;
    }

    str = str.substr(1);

    if (dm.contains(str))
    {
        return dm.at(str);
    }

    return "";
}

static Float ParseFloat(pugi::xml_attribute attr, const DefaultMap& dm)
{
    std::string str = ParseString(attr, dm);
    return std::stof(str);
}

static int32 ParseInteger(pugi::xml_attribute attr, const DefaultMap& dm)
{
    std::string str = ParseString(attr, dm);
    return std::stoi(str);
}

static bool ParseBoolean(pugi::xml_attribute attr, const DefaultMap& dm)
{
    std::string str = ParseString(attr, dm);
    if (str == "true")
    {
        return true;
    }
    else if (str == "false")
    {
        return false;
    }
    else
    {
        std::cerr << "Failed to parse boolean: " << str << std::endl;
        return false;
    }
}

static std::vector<std::string> split_string(const std::string& str, const std::regex& delim_regex)
{
    std::sregex_token_iterator first{ begin(str), end(str), delim_regex, -1 }, last;
    std::vector<std::string> list{ first, last };
    return list;
}

static std::string to_lowercase(const std::string& s)
{
    std::string out = s;
    std::transform(s.begin(), s.end(), out.begin(), ::tolower);
    return out;
}

static Vec3 ParseVec3(const std::string& value)
{
    std::vector<std::string> list = split_string(value, std::regex("(,| )+"));

    Vec3 v;
    if (list.size() == 1)
    {
        v[0] = std::stof(list[0]);
        v[1] = std::stof(list[0]);
        v[2] = std::stof(list[0]);
    }
    else if (list.size() == 3)
    {
        v[0] = std::stof(list[0]);
        v[1] = std::stof(list[1]);
        v[2] = std::stof(list[2]);
    }
    else
    {
        std::cerr << "ParseVec3 failed" << std::endl;
        return Vec3::zero;
    }

    return v;
}

static Mat4 ParseMat4(const std::string& value)
{
    std::vector<std::string> list = split_string(value, std::regex("(,| )+"));
    if (list.size() != 16)
    {
        std::cerr << "ParseMat4 failed" << std::endl;
        return identity;
    }

    Mat4 m;
    int32 k = 0;
    for (int32 i = 0; i < 4; ++i)
    {
        for (int32 j = 0; j < 4; ++j)
        {
            m[j][i] = std::stof(list[k++]);
        }
    }
    return m;
}

static Vec3 ParseVec3(pugi::xml_attribute attr, const DefaultMap& dm)
{
    std::string str = ParseString(attr, dm);
    return ParseVec3(str);
}

static Mat4 ParseMat4(pugi::xml_attribute attr, const DefaultMap& dm)
{
    std::string str = ParseString(attr, dm);
    return ParseMat4(str);
}

static Transform ParseTransform(pugi::xml_node node, const DefaultMap& dm)
{
    Transform tf = identity;

    for (auto child : node.children())
    {
        std::string name = to_lowercase(child.name());
        if (name == "matrix")
        {
            Mat4 mat = ParseMat4(child.attribute("value"), dm);
            tf = mat;
        }
        else if (name == "lookat")
        {
            Vec3 pos = ParseVec3(child.attribute("origin"), dm);
            Vec3 target = ParseVec3(child.attribute("target"), dm);
            Vec3 up = ParseVec3(child.attribute("up"), dm);

            Vec3 forward = Normalize(target - pos);
            tf = Transform(pos, Quat(forward, up));
        }
        else
        {
            std::cerr << "Transform not supported: " + name << std::endl;
        }
    }

    return tf;
}

struct FilmInfo
{
    int32 width;
    int32 height;
    std::string filename;
};

static FilmInfo ParseFilm(pugi::xml_node node, DefaultMap& dm)
{
    int32 width = g_default_width;
    int32 height = g_default_height;
    std::string filename = g_default_filename;

    for (auto child : node.children())
    {
        std::string type = child.name();
        std::string name = child.attribute("name").value();

        if (name == "width")
        {
            width = ParseInteger(child.attribute("value"), dm);
        }
        else if (name == "height")
        {
            height = ParseInteger(child.attribute("value"), dm);
        }
        else if (name == "filename")
        {
            filename = ParseString(child.attribute("value"), dm);
        }

        if (type == "rfilter")
        {
            // TODO: Fill here
        }
    }

    return { width, height, filename };
}

static std::unique_ptr<Camera> ParseCamera(pugi::xml_node node, DefaultMap& dm)
{
    std::string type = ParseString(node.attribute("type"), dm);

    Float fov = g_default_fov;
    Transform to_world = identity;

    if (type == "perspective")
    {
        for (auto child : node.children())
        {
            std::string name = ParseString(child.attribute("name"), dm);
            if (name == "fov")
            {
                fov = ParseFloat(child.attribute("value"), dm);
            }
            else if (name == "to_world" || name == "toWorld")
            {
                to_world = ParseTransform(child, dm);
            }
        }
    }
    else
    {
        std::cerr << "Camera not supported: " + type << std::endl;
        return nullptr;
    }

    FilmInfo film;

    for (auto child : node.children())
    {
        std::string name = child.name();

        if (name == "film")
        {
            film = ParseFilm(child, dm);
        }
    }

    if (type == "perspective")
    {
        std::cout << to_world.p.ToString() << std::endl;

        Point3 f = Mul(to_world, Point3(0));
        Point3 t = Mul(to_world, Point3(0, 0, 1));

        return std::make_unique<PerspectiveCamera>(f, t, y_axis, 36, 0, 1, Point2i{ film.width, film.height });
    }
    else
    {
        return nullptr;
    }
}

static bool ParseIntegrator(pugi::xml_node node, DefaultMap& dm)
{
    std::string type = ParseString(node.attribute("type"), dm);

    // TODO: add all types
    if (type == "path" || type == "volpath" || type == "bdpt" || type == "light_path" || type == "ao" || type == "debug" ||
        type == "sppm" || type == "random_walk")
    {
        dm["integrator"] = type;
    }
    else
    {
        std::cerr << "Integrator not supported: " + type << std::endl;
        return false;
    }

    return true;
}

static bool ParseBSDF(pugi::xml_node node, DefaultMap& dm, MaterialMap& mm, Scene* scene)
{
    // std::string type = node.attribute("type").value();
    return true;
}

static bool ParseShape(pugi::xml_node node, DefaultMap& dm, MaterialMap& mm, Scene* scene)
{
    const Material* mat = nullptr;

    for (auto child : node.children())
    {
        std::string name = child.name();

        if (name == "ref")
        {
            std::string name_value = child.attribute("name").value();
            pugi::xml_attribute id = child.attribute("id");
            if (id.empty())
            {
                std::cerr << "Material/medium reference id not specified." << std::endl;
            }

            if (!mm.contains(id.value()))
            {
                // std::cerr << "Material not found by id: " << std::string(id.value()) << std::endl;
                mat = mm["fallback"];
                // return false;
            }
            else
            {
                mat = mm.at(id.value());
            }
        }
    }

    std::string shape_type = node.attribute("type").value();
    if (shape_type == "obj")
    {
        std::string filename;
        Mat4 to_world = identity;
        bool face_normals = false;
        for (auto child : node.children())
        {
            std::string name = child.attribute("name").value();
            if (name == "filename")
            {
                filename = ParseString(child.attribute("value"), dm);
            }
            else if (name == "toWorld" || name == "to_world")
            {
                if (std::string(child.name()) == "transform")
                {
                    to_world = ParseTransform(child, dm);
                }
            }
            else if (name == "faceNormals" || name == "face_normals")
            {
                face_normals = ParseBoolean(child.attribute("value"), dm);
            }
        }

        SetLoaderGenSmoothNormal(true);
        SetLoaderFallbackMaterial(mat);
        LoadOBJ(*scene, filename, to_world);
    }
    else
    {
        std::cerr << "Shape type not supported: " << shape_type << std::endl;
        return true;
    }

    return true;
}

static std::optional<MitsubaScene> ParseScene(pugi::xml_node scene_node)
{
    std::unique_ptr<Scene> scene = std::make_unique<Scene>();
    std::unique_ptr<Intersectable> accel;
    std::unique_ptr<Sampler> sampler;
    std::unique_ptr<Camera> camera;
    std::unique_ptr<Integrator> integrator;

    DefaultMap dm;
    MaterialMap mm;

    auto fallback = CreateDiffuseMaterial(*scene, 1);
    mm["fallback"] = fallback;

    for (auto node : scene_node.children())
    {
        std::string name = node.name();

        if (name == "default")
        {
            ParseDefaultMap(node, dm);
        }
        else if (name == "integrator")
        {
            if (!ParseIntegrator(node, dm)) return {};
        }
        else if (name == "sensor")
        {
            camera = ParseCamera(node, dm);
            if (camera == nullptr) return {};
        }
        else if (name == "bsdf")
        {
            if (!ParseBSDF(node, dm, mm, scene.get())) return {};
        }
        else if (name == "shape")
        {
            if (!ParseShape(node, dm, mm, scene.get())) return {};
        }
    }

    for (auto p : dm)
    {
        std::cout << p.first << ": " << p.second << std::endl;
    }

    accel = std::make_unique<BVH>(scene->GetPrimitives());
    int32 spp = 64;
    sampler = std::make_unique<StratifiedSampler>(std::sqrt(spp), std::sqrt(spp), true);

    std::string integrator_type = dm.at("integrator");
    if (integrator_type == "path")
    {
        // integrator = std::make_unique<PathIntegrator>(accel.get(), scene->GetLights(), sampler.get(), 8, false);
        integrator = std::make_unique<DebugIntegrator>(accel.get(), scene->GetLights(), sampler.get());
        // integrator = std::make_unique<AmbientOcclusion>(accel.get(), scene->GetLights(), sampler.get(), 0.1f);
    }
    else
    {
        return {};
    }

    return MitsubaScene{ std::move(scene), std::move(accel), std::move(sampler), std::move(camera), std::move(integrator) };
}

std::optional<MitsubaScene> LoadMitsubaScene(std::filesystem::path filename)
{
    pugi::xml_document doc;
    pugi::xml_parse_result xml_result = doc.load_file(filename.c_str());
    if (!xml_result)
    {
        std::cerr << "Failed to load xml.." << std::endl;
        std::cerr << "Description: " << xml_result.description() << std::endl;
        return {};
    }

    fs::path old_path = fs::current_path();
    fs::current_path(filename.parent_path());
    auto scene = ParseScene(doc.child("scene"));
    fs::current_path(old_path);

    return scene;
}

} // namespace bulbit

#include "scene_loader.h"

#include "bulbit/color.h"
#include <pugixml.hpp>

#include "model_loader.h"
#include "util.h"

#include "light_builder.h"
#include "material_builder.h"
#include "scene_builder.h"
#include "texture_builder.h"

namespace fs = std::filesystem;

namespace bulbit
{

using DefaultMap = std::unordered_map<std::string, std::string>;
using MaterialMap = std::unordered_map<std::string, const Material*>;
using MediumMap = std::unordered_map<std::string, const Medium*>;

static Float VFovToHFov(Float vfov, Float aspect)
{
    return RadToDeg(2.0f * std::atan(aspect * std::tan(DegToRad(vfov) * 0.5f)));
}

static Float HFovToVFov(Float hfov, Float aspect)
{
    return RadToDeg(2.0f * std::atan(std::tan(DegToRad(hfov) * 0.5f) / aspect));
}

static void ParseDefault(pugi::xml_node node, DefaultMap& dm)
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

static Vec3 ParseVec3(const std::string& value)
{
    std::vector<std::string> list = SplitString(value, std::regex("(,| )+"));

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
    std::vector<std::string> list = SplitString(value, std::regex("(,| )+"));
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

static Vec3 ParseSRGB(const std::string& value)
{
    Vec3 srgb;
    if (value.size() == 7 && value[0] == '#')
    {
        char* end_ptr = NULL;
        // parse hex code (#abcdef)
        int encoded = strtol(value.c_str() + 1, &end_ptr, 16);
        if (*end_ptr != '\0')
        {
            std::cerr << "Invalid SRGB value: " << value << std::endl;
        }
        srgb[0] = ((encoded & 0xFF0000) >> 16) / 255.0f;
        srgb[1] = ((encoded & 0x00FF00) >> 8) / 255.0f;
        srgb[2] = (encoded & 0x0000FF) / 255.0f;
    }
    else
    {
        std::cerr << "Unknown SRGB format: " << value << std::endl;
    }
    return srgb;
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

static Vec3 ParseSRGB(pugi::xml_attribute attr, const DefaultMap& dm)
{
    std::string str = ParseString(attr, dm);
    return ParseSRGB(str);
}

static Transform ParseTransform(pugi::xml_node node, const DefaultMap& dm)
{
    Transform tf = identity;

    for (auto child : node.children())
    {
        std::string name = ToLowercase(child.name());

        if (name == "scale")
        {
            Float x = 1;
            Float y = 1;
            Float z = 1;
            if (!child.attribute("x").empty())
            {
                x = ParseFloat(child.attribute("x"), dm);
            }
            if (!child.attribute("y").empty())
            {
                y = ParseFloat(child.attribute("y"), dm);
            }
            if (!child.attribute("z").empty())
            {
                z = ParseFloat(child.attribute("z"), dm);
            }
            if (!child.attribute("value").empty())
            {
                Vec3 v = ParseVec3(child.attribute("value"), dm);
                x = v.x;
                y = v.y;
                z = v.z;
            }
            tf = Mul(Transform(Point3::zero, identity, Vec3(x, y, z)), tf);
        }
        else if (name == "translate")
        {
            Float x = 0;
            Float y = 0;
            Float z = 0;
            if (!child.attribute("x").empty())
            {
                x = ParseFloat(child.attribute("x"), dm);
            }
            if (!child.attribute("y").empty())
            {
                y = ParseFloat(child.attribute("y"), dm);
            }
            if (!child.attribute("z").empty())
            {
                z = ParseFloat(child.attribute("z"), dm);
            }
            if (!child.attribute("value").empty())
            {
                Vec3 v = ParseVec3(child.attribute("value"), dm);
                x = v.x;
                y = v.y;
                z = v.z;
            }
            tf = Mul(Transform(x, y, z), tf);
        }
        else if (name == "rotate")
        {
            Float x = 0;
            Float y = 0;
            Float z = 0;
            Float angle = 0;
            if (!child.attribute("x").empty())
            {
                x = ParseFloat(child.attribute("x"), dm);
            }
            if (!child.attribute("y").empty())
            {
                y = ParseFloat(child.attribute("y"), dm);
            }
            if (!child.attribute("z").empty())
            {
                z = ParseFloat(child.attribute("z"), dm);
            }
            if (!child.attribute("angle").empty())
            {
                angle = ParseFloat(child.attribute("angle"), dm);
            }
            tf = Mul(Quat(DegToRad(angle), Vec3(x, y, z)), tf);
        }
        else if (name == "matrix")
        {
            tf = ParseMat4(child.attribute("value"), dm);
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

static FilterInfo ParseReconFilter(pugi::xml_node node, const DefaultMap& dm)
{
    FilterInfo ri;

    std::string type = node.attribute("type").value();
    if (type == "box")
    {
        ri.type = FilterType::box;

        Float extent = 1;
        for (auto child : node.children())
        {
            if (std::string(child.attribute("name").value()) == "extent")
            {
                extent = ParseFloat(child.attribute("value"), dm);
            }
        }

        ri.extent = extent;
    }
    else if (type == "tent")
    {
        ri.type = FilterType::tent;

        Float extent = 2;
        for (auto child : node.children())
        {
            std::string name = child.attribute("name").value();
            if (name == "radius")
            {
                extent = 2 * ParseFloat(child.attribute("value"), dm);
            }
            else if (name == "extent")
            {
                extent = ParseFloat(child.attribute("value"), dm);
            }
        }

        ri.extent = extent;
    }
    else if (type == "gaussian")
    {
        ri.type = FilterType::gaussian;

        Float extent = 3;
        for (auto child : node.children())
        {
            std::string name = child.attribute("name").value();
            if (name == "extent")
            {
                extent = ParseFloat(child.attribute("value"), dm);
            }
            else if (name == "stddev")
            {
                ri.gaussian_stddev = ParseFloat(child.attribute("value"), dm);
            }
        }

        ri.extent = extent;
    }
    else
    {
        std::cerr << "Filter type not supported: " << type << std::endl;
    }

    return ri;
}

static FilmInfo ParseFilm(pugi::xml_node node, const DefaultMap& dm)
{
    FilmInfo fi;

    for (auto child : node.children())
    {
        std::string type = child.name();
        std::string name = child.attribute("name").value();

        if (name == "width")
        {
            fi.resolution.x = ParseInteger(child.attribute("value"), dm);
        }
        else if (name == "height")
        {
            fi.resolution.y = ParseInteger(child.attribute("value"), dm);
        }
        else if (name == "filename")
        {
            fi.filename = ParseString(child.attribute("value"), dm);
        }

        if (type == "rfilter")
        {
            fi.filter_info = ParseReconFilter(child, dm);
        }
    }

    return fi;
}

static SamplerInfo ParseSampler(pugi::xml_node node, const DefaultMap& dm)
{
    std::string name = node.attribute("type").value();

    SamplerInfo sampler;
    sampler.type = SamplerType::independent;
    sampler.spp = 64;

    if (name == "independent")
    {
        sampler.type = SamplerType::independent;
    }
    else if (name == "stratified")
    {
        sampler.type = SamplerType::stratified;
    }
    else
    {
        std::cerr << "Sampler not supported: " << name << std::endl;
    }

    for (auto child : node.children())
    {
        std::string name = child.attribute("name").value();
        if (name == "sample_count")
        {
            sampler.spp = ParseInteger(child.attribute("value"), dm);
        }
    }

    return sampler;
}

static void ParseCamera(pugi::xml_node node, const DefaultMap& dm, CameraInfo& ci)
{
    ci.type = CameraType::perspective;
    ci.transform = identity;
    ci.fov = 35;
    ci.aperture_radius = 0;
    ci.focus_distance = 1;

    std::string type = ParseString(node.attribute("type"), dm);
    std::string fov_axis = "x";

    if (type == "perspective" || type == "thinlens")
    {
        ci.type = CameraType::perspective;

        for (auto child : node.children())
        {
            std::string name = ParseString(child.attribute("name"), dm);
            if (name == "fov")
            {
                ci.fov = ParseFloat(child.attribute("value"), dm);
            }
            else if (name == "to_world")
            {
                ci.transform = ParseTransform(child, dm);
            }
            else if (name == "fov_axis")
            {
                fov_axis = child.attribute("value").value();
            }
            else if (name == "aperture_radius")
            {
                ci.aperture_radius = ParseFloat(child.attribute("value"), dm);
            }
        }

        if (fov_axis != "x" || fov_axis == "y")
        {
            std::cerr << "FOV axis not supported: " << fov_axis << std::endl;
            fov_axis = "x";
        }
    }
    else
    {
        std::cerr << "Camera not supported: " << type << std::endl;
    }

    for (auto child : node.children())
    {
        std::string name = child.name();

        if (name == "film")
        {
            ci.film_info = ParseFilm(child, dm);
        }
        else if (name == "sampler")
        {
            ci.sampler_info = ParseSampler(child, dm);
        }
    }

    if (fov_axis == "x")
    {
        Float aspect = Float(ci.film_info.resolution.x) / Float(ci.film_info.resolution.y);
        ci.fov = HFovToVFov(ci.fov, aspect);
    }
}

static void ParseIntegrator(pugi::xml_node node, DefaultMap& dm, IntegratorInfo& ri)
{
    std::string type = ParseString(node.attribute("type"), dm);

    // clang-format off
    if (type == "path") { ri.type = IntegratorType::path; }
    else if (type == "volpath") { ri.type = IntegratorType::vol_path; }
    else if (type == "light_path") { ri.type = IntegratorType::light_path; }
    else if (type == "light_vol_path") { ri.type = IntegratorType::light_vol_path; }
    else if (type == "bdpt") { ri.type = IntegratorType::bdpt; }
    else if (type == "vol_bdpt") { ri.type = IntegratorType::vol_bdpt; }
    else if (type == "naive_path") { ri.type = IntegratorType::naive_path; }
    else if (type == "naive_vol_path") { ri.type = IntegratorType::naive_vol_path; }
    else if (type == "random_walk") { ri.type = IntegratorType::random_walk; }
    else if (type == "ao") { ri.type = IntegratorType::ao; }
    else if (type == "albedo") { ri.type = IntegratorType::albedo; }
    else if (type == "debug") { ri.type = IntegratorType::debug; }
    else if (type == "pm") { ri.type = IntegratorType::pm; }
    else if (type == "sppm") { ri.type = IntegratorType::sppm; }
    // clang-format on
    else
    {
        std::cerr << "Integrator not supported: " + type << std::endl;
    }

    for (auto child : node.children())
    {
        std::string name = child.name();

        if (name == "max_depth")
        {
            ri.max_bounces = ParseInteger(node.attribute("value"), dm);
        }
        else if (name == "rr_depth")
        {
            ri.rr_min_bounces = ParseInteger(node.attribute("value"), dm);
        }
        else if (name == "ao_range")
        {
            ri.ao_range = ParseFloat(node.attribute("value"), dm);
        }
        else if (name == "n_photons")
        {
            ri.n_photons = ParseFloat(node.attribute("value"), dm);
        }
        else if (name == "initial_radius")
        {
            ri.initial_radius = ParseFloat(node.attribute("value"), dm);
        }
    }
}

static Spectrum ParseColor(pugi::xml_node node, const DefaultMap& dm)
{
    std::string type = node.name();
    if (type == "spectrum")
    {
        std::cerr << "Color type not supported: spectrum" << std::endl;
        return Spectrum::black;
    }
    else if (type == "rgb")
    {
        Vec3 rgb = ParseVec3(node.attribute("value"), dm);
        return Spectrum(rgb.x, rgb.y, rgb.z);
    }
    else if (type == "srgb")
    {
        Vec3 rgb = RGB_from_sRGB(ParseSRGB(node.attribute("value"), dm));
        return Spectrum(rgb.x, rgb.y, rgb.z);
    }
    else if (type == "float")
    {
        return Spectrum(ParseFloat(node.attribute("value"), dm));
    }
    else
    {
        std::cerr << "Unknown color type: " << type << std::endl;
        return Spectrum::black;
    }
}

static Spectrum ParseIntensity(pugi::xml_node node, const DefaultMap& dm)
{
    std::string rad_type = node.name();
    if (rad_type == "spectrum")
    {
        std::cerr << "Not a supported radiance type: spectrum" << std::endl;
        return Spectrum::black;
    }
    else if (rad_type == "rgb")
    {
        Vec3 rgb = ParseVec3(node.attribute("value"), dm);
        return Spectrum(rgb.x, rgb.y, rgb.z);
    }
    else if (rad_type == "srgb")
    {
        Vec3 rgb = RGB_from_sRGB(ParseSRGB(node.attribute("value"), dm));
        return Spectrum(rgb.x, rgb.y, rgb.z);
    }

    return Spectrum(1);
}

enum class TextureType
{
    bitmap,
    checkboard,
};

struct TextureInfo
{
    TextureType type;

    fs::path filename;
    Spectrum color0, color1; // for checkerboard
    Point2 scale;
    TexCoordFilter wrap_mode;
    int32 channel;
};

static TextureInfo ParseTexture(pugi::xml_node node, const DefaultMap& dm)
{
    TextureInfo ti;
    ti.type = TextureType::checkboard;
    ti.filename = "";
    ti.color0 = { 0.4f, 0.4f, 0.4f };
    ti.color1 = { 0.2f, 0.2f, 0.2f };
    ti.scale = { 1, 1 };
    ti.wrap_mode = repeat;
    ti.channel = 0;

    std::string type = node.attribute("type").value();
    if (type == "bitmap")
    {
        ti.type = TextureType::bitmap;

        std::string wm = "repeat";
        for (auto child : node.children())
        {
            std::string name = child.attribute("name").value();
            if (name == "filename")
            {
                ti.filename = ParseString(child.attribute("value"), dm);
            }
            else if (name == "wrap_mode")
            {
                wm = ParseString(child.attribute("value"), dm);
            }
            else if (name == "channel")
            {
                ti.channel = ParseInteger(child.attribute("value"), dm);
            }
        }

        if (wm == "repeat")
        {
            ti.wrap_mode = repeat;
        }
        else if (wm == "clamp")
        {
            ti.wrap_mode = clamp;
        }
        else
        {
            std::cerr << "Wrapmode not supported: " << wm << std::endl;
        }
    }
    else if (type == "checkerboard")
    {
        ti.type = TextureType::checkboard;

        for (auto child : node.children())
        {
            std::string name = child.attribute("name").value();

            if (name == "color0")
            {
                ti.color0 = ParseColor(child, dm);
            }
            else if (name == "color1")
            {
                ti.color1 = ParseColor(child, dm);
            }
            else if (name == "to_uv")
            {
                Transform tf = ParseTransform(child, dm);
                ti.scale.x = tf.s.x;
                ti.scale.y = tf.s.y;
            }
        }
    }
    else
    {
        std::cerr << "Texture type not supported: " << type << std::endl;
    }

    return ti;
}

static FloatTexture* ParseFloatTexture(
    pugi::xml_node node, const DefaultMap& dm, Scene* scene, std::function<Float(Float)> transform = [](Float v) { return v; }
)
{
    std::string type = node.name();
    if (type == "float")
    {
        Float value = ParseFloat(node.attribute("value"), dm);
        return CreateFloatConstantTexture(*scene, transform(value));
    }
    else if (type == "texture")
    {
        TextureInfo t = ParseTexture(node, dm);

        if (t.type == TextureType::bitmap)
        {
            return CreateFloatImageTexture(*scene, t.filename.string(), t.channel, true, std::move(transform));
        }
        else if (t.type == TextureType::checkboard)
        {
            return CreateFloatCheckerTexture(*scene, transform(t.color0.Average()), transform(t.color1.Average()), t.scale);
        }
        else
        {
            std::cerr << "Texture type not supported" << std::endl;
            return CreateFloatConstantTexture(*scene, 0);
        }
    }
    else if (type == "ref")
    {
        // TODO: implement
        std::cerr << "Not a supported float texture type: ref" << std::endl;
        return CreateFloatConstantTexture(*scene, 0);
    }
    else
    {
        std::cerr << "Unknown float texture type: " << type << std::endl;
        return CreateFloatConstantTexture(*scene, 0);
    }
}

static SpectrumTexture* ParseSpectrumTexture(
    pugi::xml_node node,
    const DefaultMap& dm,
    Scene* scene,
    bool non_color = false,
    std::function<Spectrum(Spectrum)> transform = [](Spectrum v) { return v; }
)
{
    std::string type = node.name();
    if (type == "spectrum")
    {
        std::cerr << "Not a supported texture type: spectrum" << std::endl;
        return nullptr;
    }
    else if (type == "rgb")
    {
        Vec3 rgb = ParseVec3(node.attribute("value"), dm);
        return CreateSpectrumConstantTexture(*scene, transform({ rgb.x, rgb.y, rgb.z }));
    }
    else if (type == "srgb")
    {
        Vec3 rgb = RGB_from_sRGB(ParseVec3(node.attribute("value"), dm));
        return CreateSpectrumConstantTexture(*scene, transform({ rgb.x, rgb.y, rgb.z }));
    }
    else if (type == "texture")
    {
        TextureInfo t = ParseTexture(node, dm);

        if (t.type == TextureType::bitmap)
        {
            return CreateSpectrumImageTexture(*scene, t.filename.string(), non_color, std::move(transform));
        }
        else if (t.type == TextureType::checkboard)
        {
            return CreateSpectrumCheckerTexture(*scene, transform(t.color0), transform(t.color1), t.scale);
        }
        else
        {
            std::cerr << "Texture type not supported: " << int32(t.type) << std::endl;
            return CreateSpectrumConstantTexture(*scene, 1, 0, 1);
        }
    }
    else if (type == "ref")
    {
        // TODO: implement
        std::cerr << "Not a supported spectrum texture type: ref" << std::endl;
        return CreateSpectrumConstantTexture(*scene, 1, 0, 1);
    }
    else
    {
        std::cerr << "Unknown spectrum texture type: " << type << std::endl;
        return CreateSpectrumConstantTexture(*scene, 1, 0, 1);
    }
}

struct MaterialInfo
{
    bool two_sided = false;
    const SpectrumTexture* normal = nullptr;
    const FloatTexture* alpha = nullptr;
};

static const Material* ParseMaterial(
    pugi::xml_node node,
    const DefaultMap& dm,
    MaterialMap& mm,
    Scene* scene,
    const MaterialInfo& mi,
    const std::string& parent_id = ""
)
{
    std::string type = node.attribute("type").value();
    std::string id = parent_id;
    if (!node.attribute("id").empty())
    {
        id = node.attribute("id").value();
    }

    const Material* mat = nullptr;

    if (type == "bumpmap")
    {
        std::cerr << "Bumpmap not supported" << std::endl;
        for (auto child : node.children())
        {
            if (std::string(child.name()) == "bsdf")
            {
                return ParseMaterial(child, dm, mm, scene, mi, id);
            }
        }
    }
    else if (type == "normalmap" || type == "normal")
    {
        const SpectrumTexture* normalmap = nullptr;

        for (auto child : node.children())
        {
            std::string name = child.attribute("name").value();
            if (name == "normalmap")
            {
                normalmap = ParseSpectrumTexture(child, dm, scene, true);
            }
        }

        if (!normalmap)
        {
            std::cerr << "normal texture not specified for normalmap bsdf" << std::endl;
        }

        for (auto child : node.children())
        {
            if (std::string(child.name()) == "bsdf")
            {
                MaterialInfo new_mi = mi;
                new_mi.normal = normalmap;
                return ParseMaterial(child, dm, mm, scene, new_mi, id);
            }
        }
    }
    else if (type == "blendbsdf" || type == "blend")
    {
        const FloatTexture* weight = nullptr;

        for (auto child : node.children())
        {
            std::string name = child.attribute("name").value();
            if (name == "weight")
            {
                weight = ParseFloatTexture(child, dm, scene);
            }
            else
            {
                std::cerr << "Unsupported parameter for blend material: " << name << std::endl;
            }
        }

        if (!weight)
        {
            std::cerr << "Weight texture not specified for blendbsdf" << std::endl;
        }

        int32 index = 0;
        const Material* m[2];
        for (auto child : node.children())
        {
            std::string type = child.name();
            if (type == "bsdf")
            {
                m[index++] = ParseMaterial(child, dm, mm, scene, mi);
            }

            if (index == 2)
            {
                break;
            }
        }

        mat = scene->CreateMaterial<MixtureMaterial>(m[0], m[1], weight, mi.alpha);
    }
    else if (type == "mask")
    {
        const FloatTexture* alpha = nullptr;

        for (auto child : node.children())
        {
            std::string name = child.attribute("name").value();
            if (name == "opacity")
            {
                alpha = ParseFloatTexture(child, dm, scene);
            }
        }

        if (!alpha)
        {
            std::cerr << "mask texture not specified for mask bsdf" << std::endl;
        }

        for (auto child : node.children())
        {
            if (std::string(child.name()) == "bsdf")
            {
                MaterialInfo new_mi = mi;
                new_mi.alpha = alpha;
                return ParseMaterial(child, dm, mm, scene, new_mi, id);
            }
        }
    }
    else if (type == "twosided" || type == "two_sided")
    {
        for (auto child : node.children())
        {
            if (std::string(child.name()) == "bsdf")
            {
                MaterialInfo new_mi = mi;
                new_mi.two_sided = true;
                return ParseMaterial(child, dm, mm, scene, new_mi, id);
            }
        }
    }
    else if (type == "layered")
    {
        Spectrum albedo(0);
        Float thickness = 1e-4f;
        Float g = 0;
        int32 max_bounces = 16;
        int32 samples = 1;

        for (auto child : node.children())
        {
            std::string name = child.attribute("name").value();
            if (name == "albedo")
            {
                albedo = ParseIntensity(child, dm);
            }
            else if (name == "thickness")
            {
                thickness = ParseFloat(child.attribute("value"), dm);
            }
            else if (name == "g")
            {
                g = ParseFloat(child.attribute("value"), dm);
            }
            else if (name == "max_bounces")
            {
                max_bounces = ParseInteger(child.attribute("value"), dm);
            }
            else if (name == "samples")
            {
                samples = ParseInteger(child.attribute("value"), dm);
            }
            else
            {
                std::cerr << "Unsupported parameter for layered material: " << name << std::endl;
            }
        }

        int32 index = 0;
        const Material* m[2];
        for (auto child : node.children())
        {
            std::string type = child.name();
            if (type == "bsdf")
            {
                m[index++] = ParseMaterial(child, dm, mm, scene, mi, id);
            }

            if (index == 2)
            {
                break;
            }
        }

        mat = scene->CreateMaterial<LayeredMaterial>(
            m[0], m[1], mi.two_sided, albedo, thickness, g, max_bounces, samples, mi.normal, mi.alpha
        );
    }
    else if (type == "diffuse")
    {
        SpectrumTexture* reflectance = CreateSpectrumConstantTexture(*scene, 1, 0, 1);
        FloatTexture* roughness = nullptr;
        for (auto child : node.children())
        {
            std::string name = child.attribute("name").value();
            if (name == "reflectance")
            {
                reflectance = ParseSpectrumTexture(child, dm, scene);
            }
            else if (name == "roughness")
            {
                roughness = ParseFloatTexture(child, dm, scene);
            }
            else
            {
                std::cerr << "Unsupported parameter for diffuse material: " << name << std::endl;
            }
        }

        mat = scene->CreateMaterial<DiffuseMaterial>(reflectance, roughness, mi.normal, mi.alpha);
    }
    else if (type == "plastic" || type == "roughplastic" || type == "rough_plastic")
    {
        Float int_ior = 1.49f;
        Float ext_ior = 1.000277f;

        const SpectrumTexture* basecolor = CreateSpectrumConstantTexture(*scene, 0.5f, 0.5f, 0.5f);
        const FloatTexture* metallic = CreateFloatConstantTexture(*scene, 0);
        const FloatTexture* roughness;
        if (type == "plastic")
        {
            roughness = CreateFloatConstantTexture(*scene, 0.0f);
        }
        else
        {
            roughness = CreateFloatConstantTexture(*scene, std::sqrt(0.1f));
        }

        for (auto child : node.children())
        {
            std::string name = child.attribute("name").value();
            if (name == "diffuse_reflectance" || name == "reflectance")
            {
                basecolor = ParseSpectrumTexture(child, dm, scene);
            }
            else if (name == "alpha")
            {
                Float alpha = ParseFloat(child.attribute("value"), dm);
                roughness = CreateFloatConstantTexture(*scene, std::sqrt(alpha));
            }
            else if (name == "int_ior")
            {
                int_ior = ParseFloat(child.attribute("value"), dm);
            }
            else if (name == "ext_ior")
            {
                ext_ior = ParseFloat(child.attribute("value"), dm);
            }
            else if (name == "distribution")
            {
                std::string mf = ParseString(child.attribute("value"), dm);
                if (mf != "ggx")
                {
                    std::cerr << "Only GGX distribution is supported: " << mf << std::endl;
                }
            }
            else
            {
                std::cerr << "Unsupported parameter for (rough)plastic material: " << name << std::endl;
            }
        }

        Float eta = int_ior / ext_ior;

        const SpectrumTexture* reflectance = CreateSpectrumConstantTexture(*scene, 1);
        const Material* top = scene->CreateMaterial<DielectricMaterial>(eta, roughness, roughness, reflectance);
        const Material* bottom = scene->CreateMaterial<DiffuseMaterial>(basecolor);

        mat = scene->CreateMaterial<LayeredMaterial>(
            top, bottom, mi.two_sided, Spectrum::black, 1e-4f, 0, 16, 1, mi.normal, mi.alpha
        );
    }
    else if (type == "conductor" || type == "roughconductor" || type == "tough_conductor")
    {
        const SpectrumTexture* reflectance = nullptr;
        const SpectrumTexture* eta = nullptr;
        const SpectrumTexture* k = nullptr;
        const FloatTexture* u_roughness;

        if (type == "conductor")
        {
            u_roughness = CreateFloatConstantTexture(*scene, 0.0f);
        }
        else
        {
            u_roughness = CreateFloatConstantTexture(*scene, 0.1f);
        }
        const FloatTexture* v_roughness = u_roughness;

        for (auto child : node.children())
        {
            std::string name = child.attribute("name").value();
            if (name == "material")
            {
                std::string conductor_type = child.attribute("value").value();
                if (conductor_type == "none")
                {
                    // Mirror
                    break;
                }
                else
                {
                    std::cerr << "Conductor type not supported: " << conductor_type << std::endl;
                }
            }
            else if (name == "eta")
            {
                eta = ParseSpectrumTexture(child, dm, scene, true);
            }
            else if (name == "k")
            {
                k = ParseSpectrumTexture(child, dm, scene, true);
            }
            else if (name == "specular_reflectance" || name == "reflectance")
            {
                reflectance = ParseSpectrumTexture(child, dm, scene);
            }
            else if (name == "alpha")
            {
                u_roughness = ParseFloatTexture(child, dm, scene, [](Float alpha) { return std::sqrt(alpha); });
                v_roughness = u_roughness;
            }
            else if (name == "alpha_u")
            {
                u_roughness = ParseFloatTexture(child, dm, scene, [](Float alpha) { return std::sqrt(alpha); });
            }
            else if (name == "alpha_v")
            {
                v_roughness = ParseFloatTexture(child, dm, scene, [](Float alpha) { return std::sqrt(alpha); });
            }
            else if (name == "distribution")
            {
                std::string mf = ParseString(child.attribute("value"), dm);
                if (mf != "ggx")
                {
                    std::cerr << "Only GGX distribution is supported: " << mf << std::endl;
                }
            }
            else
            {
                std::cerr << "Unsupported parameter for (rough)conductor material: " << name << std::endl;
            }
        }

        if (eta == nullptr && k == nullptr)
        {
            mat = scene->CreateMaterial<MirrorMaterial>(CreateSpectrumConstantTexture(*scene, 1), mi.normal, mi.alpha);
        }
        else
        {
            mat = scene->CreateMaterial<ConductorMaterial>(
                eta, k, u_roughness, v_roughness, reflectance, true, mi.normal, mi.alpha
            );
        }
    }
    else if (type == "dielectric" || type == "roughdielectric" || type == "rough_dielectric")
    {
        Float int_ior = 1.5046f;
        Float ext_ior = 1.000277f;

        const SpectrumTexture* reflectance = CreateSpectrumConstantTexture(*scene, 1);
        const FloatTexture* u_roughness;

        if (type == "dielectric")
        {
            u_roughness = CreateFloatConstantTexture(*scene, 0.0f);
        }
        else
        {
            u_roughness = CreateFloatConstantTexture(*scene, 0.1f);
        }
        const FloatTexture* v_roughness = u_roughness;

        for (auto child : node.children())
        {
            std::string name = child.attribute("name").value();
            if (name == "alpha")
            {
                u_roughness = ParseFloatTexture(child, dm, scene, [](Float alpha) { return std::sqrt(alpha); });
                v_roughness = u_roughness;
            }
            else if (name == "alpha_u")
            {
                u_roughness = ParseFloatTexture(child, dm, scene, [](Float alpha) { return std::sqrt(alpha); });
            }
            else if (name == "alpha_v")
            {
                v_roughness = ParseFloatTexture(child, dm, scene, [](Float alpha) { return std::sqrt(alpha); });
            }
            else if (name == "int_ior")
            {
                int_ior = ParseFloat(child.attribute("value"), dm);
            }
            else if (name == "ext_ior")
            {
                ext_ior = ParseFloat(child.attribute("value"), dm);
            }
            else if (name == "reflectance" || name == "specular_transmittance")
            {
                reflectance = ParseSpectrumTexture(child, dm, scene);
            }
            else if (name == "distribution")
            {
                std::string mf = ParseString(child.attribute("value"), dm);
                if (mf != "ggx")
                {
                    std::cerr << "Only GGX distribution is supported: " << mf << std::endl;
                }
            }
            else
            {
                std::cerr << "Unsupported parameter for (rough)dielectric material: " << name << std::endl;
            }
        }

        Float eta = int_ior / ext_ior;
        mat = scene->CreateMaterial<DielectricMaterial>(eta, u_roughness, v_roughness, reflectance, true, mi.normal);
    }
    else if (type == "thindielectric" || type == "thin_dielectric")
    {
        Float int_ior = 1.5046f;
        Float ext_ior = 1.000277f;
        const SpectrumTexture* reflectance = CreateSpectrumConstantTexture(*scene, 1);

        for (auto child : node.children())
        {
            std::string name = child.attribute("name").value();
            if (name == "int_ior")
            {
                int_ior = ParseFloat(child.attribute("value"), dm);
            }
            else if (name == "ext_ior")
            {
                ext_ior = ParseFloat(child.attribute("value"), dm);
            }
            else if (name == "reflectance")
            {
                reflectance = ParseSpectrumTexture(child, dm, scene);
            }
            else
            {
                std::cerr << "Unsupported parameter for thindielectric material: " << name << std::endl;
            }
        }

        Float eta = int_ior / ext_ior;
        mat = scene->CreateMaterial<ThinDielectricMaterial>(eta, reflectance);
    }
    else if (type == "principled")
    {
        const SpectrumTexture* basecolor = CreateSpectrumConstantTexture(*scene, 1);
        const FloatTexture* metallic = CreateFloatConstantTexture(*scene, 0);
        const FloatTexture* roughness = CreateFloatConstantTexture(*scene, 1);
        const FloatTexture* anisotropy = CreateFloatConstantTexture(*scene, 0);
        Float ior = 1.5f;
        const FloatTexture* transmission = CreateFloatConstantTexture(*scene, 0);
        const FloatTexture* clearcoat = CreateFloatConstantTexture(*scene, 0);
        const FloatTexture* clearcoat_roughness = CreateFloatConstantTexture(*scene, 0);
        const SpectrumTexture* clearcoat_color = CreateSpectrumConstantTexture(*scene, 1);
        const FloatTexture* sheen = CreateFloatConstantTexture(*scene, 0);
        const FloatTexture* sheen_roughness = CreateFloatConstantTexture(*scene, 0);
        const SpectrumTexture* sheen_color = CreateSpectrumConstantTexture(*scene, 1);

        for (auto child : node.children())
        {
            std::string name = child.attribute("name").value();
            if (name == "basecolor" || name == "reflectance")
            {
                basecolor = ParseSpectrumTexture(child, dm, scene);
            }
            else if (name == "metallic")
            {
                metallic = ParseFloatTexture(child, dm, scene);
            }
            else if (name == "roughness")
            {
                roughness = ParseFloatTexture(child, dm, scene);
            }
            else if (name == "anisotropy")
            {
                anisotropy = ParseFloatTexture(child, dm, scene);
            }
            else if (name == "ior")
            {
                ior = ParseFloat(child.attribute("value"), dm);
            }
            else if (name == "transmission")
            {
                transmission = ParseFloatTexture(child, dm, scene);
            }
            else if (name == "clearcoat")
            {
                clearcoat = ParseFloatTexture(child, dm, scene);
            }
            else if (name == "clearcoat_roughness")
            {
                clearcoat_roughness = ParseFloatTexture(child, dm, scene);
            }
            else if (name == "clearcoat_color")
            {
                clearcoat_color = ParseSpectrumTexture(child, dm, scene);
            }
            else if (name == "sheen")
            {
                sheen = ParseFloatTexture(child, dm, scene);
            }
            else if (name == "sheen_roughness")
            {
                sheen_roughness = ParseFloatTexture(child, dm, scene);
            }
            else if (name == "sheen_color")
            {
                sheen_color = ParseSpectrumTexture(child, dm, scene);
            }
            else
            {
                std::cerr << "Unsupported parameter for principled material: " << name << std::endl;
            }
        }

        mat = scene->CreateMaterial<PrincipledMaterial>(
            basecolor, metallic, roughness, anisotropy, ior, transmission, clearcoat, clearcoat_roughness, clearcoat_color, sheen,
            sheen_roughness, sheen_color, mi.normal, mi.alpha
        );
    }
    else if (type == "subsurface")
    {
        const SpectrumTexture* reflectance = CreateSpectrumConstantTexture(*scene, 1);
        Spectrum mfp(0.01f);
        Float eta = 1.5f;
        const FloatTexture* u_roughness = CreateFloatConstantTexture(*scene, 0.3f);
        const FloatTexture* v_roughness = u_roughness;
        Float g = 0;

        for (auto child : node.children())
        {
            std::string name = child.attribute("name").value();
            if (name == "reflectance")
            {
                reflectance = ParseSpectrumTexture(child, dm, scene);
            }
            else if (name == "mfp")
            {
                mfp = ParseColor(child, dm);
            }
            else if (name == "rougness")
            {
                u_roughness = ParseFloatTexture(child, dm, scene);
                v_roughness = u_roughness;
            }
            else if (name == "u_roughness")
            {
                u_roughness = ParseFloatTexture(child, dm, scene);
            }
            else if (name == "v_roughness")
            {
                v_roughness = ParseFloatTexture(child, dm, scene);
            }
            else if (name == "g")
            {
                g = ParseFloat(child.attribute("value"), dm);
            }
            else
            {
                std::cerr << "Unsupported parameter for subsurface material: " << name << std::endl;
            }
        }

        mat = scene->CreateMaterial<SubsurfaceRandomWalkMaterial>(reflectance, mfp, eta, u_roughness, v_roughness, g, mi.normal);
    }
    else if (type == "cloth")
    {
        const SpectrumTexture* basecolor = CreateSpectrumConstantTexture(*scene, 1);
        const SpectrumTexture* sheen_color = CreateSpectrumConstantTexture(*scene, 1, 0, 1);
        const FloatTexture* roughness = CreateFloatConstantTexture(*scene, 0.3f);

        for (auto child : node.children())
        {
            std::string name = child.attribute("name").value();
            if (name == "basecolor" || name == "reflectance")
            {
                basecolor = ParseSpectrumTexture(child, dm, scene);
            }
            else if (name == "sheen_color")
            {
                sheen_color = ParseSpectrumTexture(child, dm, scene);
            }
            else if (name == "roughness")
            {
                roughness = ParseFloatTexture(child, dm, scene);
            }
            else
            {
                std::cerr << "Unsupported parameter for cloth material: " << name << std::endl;
            }
        }

        mat = scene->CreateMaterial<ClothMaterial>(basecolor, sheen_color, roughness, mi.normal, mi.alpha);
    }
    else
    {
        std::cerr << "BSDF not supported: " << type << std::endl;
    }

    if (!id.empty() && mat != nullptr)
    {
        mm[id] = mat;
    }

    return mat;
}

static const Float ParsePhaseFunction(pugi::xml_node node, const DefaultMap& dm)
{
    std::string type = node.attribute("type").value();

    if (type == "isotropic")
    {
        return 0;
    }
    else if (type == "hg")
    {
        Float g = 0;
        for (auto child : node.children())
        {
            std::string name = child.attribute("name").value();
            if (name == "g")
            {
                g = ParseFloat(child.attribute("value"), dm);
            }
        }

        return g;
    }
    else
    {
        std::cerr << "Phase function not supported: " + type << std::endl;
        return 0;
    }
}

static const Medium* ParseMedium(pugi::xml_node node, const DefaultMap& dm, MediumMap& mdm, Scene* scene)
{
    Float g = 0;
    std::string type = node.attribute("type").value();
    std::string id = "";
    if (!node.attribute("id").empty())
    {
        id = node.attribute("id").value();
    }

    const Medium* medium = nullptr;

    if (type == "homogeneous")
    {
        Spectrum albedo(0.75f); // = sigma_s / sigma_t
        Spectrum sigma_t(1.0f); // = sigma_a + sigma_s
        Float scale = 1;
        Spectrum emission(0);

        for (auto child : node.children())
        {
            std::string name = child.attribute("name").value();
            if (name == "albedo")
            {
                albedo = ParseColor(child, dm);
            }
            else if (name == "sigma_t")
            {
                // Assume sigma_a = 0
                sigma_t = ParseColor(child, dm);
            }
            else if (name == "emission")
            {
                emission = ParseColor(child, dm);
            }
            else if (name == "scale")
            {
                scale = ParseFloat(child.attribute("value"), dm);
            }
            else if (std::string(child.name()) == "phase")
            {
                g = ParsePhaseFunction(child, dm);
            }
        }

        Spectrum sigma_s = albedo * sigma_t;
        Spectrum sigma_a = sigma_t - sigma_s;

        medium = scene->CreateMedium<HomogeneousMedium>(sigma_a * scale, sigma_s * scale, emission, g);
    }
    else if (type == "nvdb")
    {
        Transform transform = identity;
        Spectrum sigma_a(0);
        Spectrum sigma_s(1);
        Float sigma_scale = 1;
        Float g = 0;
        std::string filename = "";
        int32 density_index = 0;
        int32 temperature_index = -1;
        Float Le_scale = 1;
        Float temperature_offset = 0;
        Float temperature_scale = 1;

        for (auto child : node.children())
        {
            std::string name = child.attribute("name").value();
            if (name == "to_world")
            {
                if (std::string(child.name()) == "transform")
                {
                    transform = ParseTransform(child, dm);
                }
            }
            else if (name == "sigma_a")
            {
                sigma_a = ParseColor(child, dm);
            }
            else if (name == "sigma_s")
            {
                sigma_s = ParseColor(child, dm);
            }
            else if (name == "sigma_scale")
            {
                sigma_scale = ParseFloat(child.attribute("value"), dm);
            }
            else if (name == "g")
            {
                g = ParseFloat(child.attribute("value"), dm);
            }
            else if (name == "Le_scale")
            {
                Le_scale = ParseFloat(child.attribute("value"), dm);
            }
            else if (name == "temperature_offset")
            {
                temperature_offset = ParseFloat(child.attribute("value"), dm);
            }
            else if (name == "temperature_scale")
            {
                temperature_scale = ParseFloat(child.attribute("value"), dm);
            }
            else if (name == "grid")
            {
                filename = ParseString(child.attribute("value"), dm);
            }
            else if (name == "density_index")
            {
                density_index = ParseInteger(child.attribute("value"), dm);
            }
            else if (name == "temperature_index")
            {
                temperature_index = ParseInteger(child.attribute("value"), dm);
            }
        }

        if (filename.empty())
        {
            std::cerr << "grid not provided" << std::endl;
            return nullptr;
        }

        std::vector<nanovdb::GridHandle<nanovdb::HostBuffer>> handles;
        try
        {
            handles = nanovdb::io::readGrids(filename);
        }
        catch (std::exception& e)
        {
            std::cout << e.what() << std::endl;
        }

        if (handles.size() == 0)
        {
            std::cerr << "Failed to read NanoVDB file: " << filename << std::endl;
            return nullptr;
        }

        if (density_index >= handles.size())
        {
            std::cerr << "Invalid density grid index: " << density_index << std::endl;
        }

        if (temperature_index >= handles.size())
        {
            std::cerr << "Invalid temperature grid index: " << temperature_index << std::endl;
        }

        if (temperature_index < 0)
        {
            medium = scene->CreateMedium<NanoVDBMedium>(
                transform, sigma_a, sigma_s, sigma_scale, g, std::move(handles[density_index])
            );
        }
        else
        {
            medium = scene->CreateMedium<NanoVDBMedium>(
                transform, sigma_a, sigma_s, sigma_scale, g, std::move(handles[density_index]),
                std::move(handles[temperature_index]), Le_scale, temperature_offset, temperature_scale
            );
        }
    }
    else
    {
        std::cerr << "Medium not supported: " << type << std::endl;
    }

    if (!id.empty() && medium != nullptr)
    {
        mdm[id] = medium;
    }

    return medium;
}

static void ParseShape(pugi::xml_node node, const DefaultMap& dm, MaterialMap& mm, MediumMap& mdm, Scene* scene)
{
    const Material* mat = nullptr;
    MediumInterface mi(nullptr, nullptr);
    std::optional<AreaLightInfo> ali = {};

    for (auto child : node.children())
    {
        std::string name = child.name();

        if (name == "ref")
        {
            std::string name_value = child.attribute("name").value();
            std::string id_value = child.attribute("id").value();

            if (id_value.empty())
            {
                std::cerr << "Material/medium reference id not specified." << std::endl;
            }

            if (name_value == "interior")
            {
                if (!mdm.contains(id_value))
                {
                    std::cerr << "Medium interior reference not found: " + id_value << std::endl;
                }

                mi.inside = mdm.at(id_value);
            }
            else if (name_value == "exterior")
            {
                if (!mdm.contains(id_value))
                {
                    std::cerr << "Medium exterior reference not found: " + id_value << std::endl;
                }

                mi.outside = mdm.at(id_value);
            }
            else if (name_value == "two_sided")
            {
                if (!mdm.contains(id_value))
                {
                    std::cerr << "Medium two_sided reference not found: " + id_value << std::endl;
                }

                mi.inside = mdm.at(id_value);
                mi.outside = mi.inside;
            }
            else
            {
                if (!mm.contains(id_value))
                {
                    if (!mm.contains("fallback"))
                    {
                        std::cerr << "Material not found by id: " << id_value << std::endl;
                    }
                    else
                    {
                        std::cout << "here" << std::endl;
                    }
                }

                mat = mm.at(id_value);
            }
        }
        else if (name == "emitter")
        {
            ali.emplace();
            ali->is_directional = false;
            ali->two_sided = false;
            ali->emission = 0.0f;

            for (auto grand_child : child.children())
            {
                std::string name = grand_child.attribute("name").value();
                if (name == "radiance")
                {
                    ali->emission = ParseIntensity(grand_child, dm);
                }
            }
        }
        else if (name == "bsdf")
        {
            mat = ParseMaterial(child, dm, mm, scene, {});
        }
        else if (name == "medium")
        {
            std::string name_value = child.attribute("name").value();
            if (name_value == "interior")
            {
                mi.inside = ParseMedium(child, dm, mdm, scene);
            }
            else if (name_value == "exterior")
            {
                mi.outside = ParseMedium(child, dm, mdm, scene);
            }
            else if (name_value == "two_sided")
            {
                mi.inside = ParseMedium(child, dm, mdm, scene);
                mi.outside = mi.inside;
            }
            else
            {
                std::cerr << "Invalid medium name: " + name_value << std::endl;
            }
        }
    }

    std::string shape_type = node.attribute("type").value();
    if (shape_type == "sphere")
    {
        Point3 center = Point3::zero;
        Float radius = 1;

        for (auto child : node.children())
        {
            std::string name = child.attribute("name").value();
            if (name == "center")
            {
                center = Vec3{ ParseFloat(child.attribute("x"), dm), ParseFloat(child.attribute("y"), dm),
                               ParseFloat(child.attribute("z"), dm) };
            }
            else if (name == "radius")
            {
                radius = ParseFloat(child.attribute("value"), dm);
            }
        }

        CreateSphere(*scene, center, radius, mat, mi, ali);
    }
    else if (shape_type == "rectangle")
    {
        Transform to_world = identity;
        bool flip_normals = false;

        for (auto child : node.children())
        {
            std::string name = child.attribute("name").value();
            if (name == "to_world")
            {
                if (std::string(child.name()) == "transform")
                {
                    to_world = ParseTransform(child, dm);
                }
            }
            else if (name == "flip_normals")
            {
                flip_normals = ParseBoolean(child.attribute("value"), dm);
            }
        }

        CreateRectXY(*scene, Mul(to_world, Transform(0, 0, 0, identity, Point3{ 2 })), mat, mi, ali);
    }
    else if (shape_type == "cube")
    {
        Transform to_world = identity;

        for (auto child : node.children())
        {
            std::string name = child.attribute("name").value();
            if (name == "to_world")
            {
                if (std::string(child.name()) == "transform")
                {
                    to_world = ParseTransform(child, dm);
                }
            }
        }

        CreateBox(*scene, to_world * Transform(0, 0, 0, identity, { 2, 2, 2 }), mat, mi, ali);
    }
    else if (shape_type == "obj" || shape_type == "gltf")
    {
        std::string filename;
        Mat4 to_world = identity;
        bool face_normals = false;
        bool use_model_material = false;

        for (auto child : node.children())
        {
            std::string name = child.attribute("name").value();
            if (name == "filename")
            {
                filename = ParseString(child.attribute("value"), dm);
            }
            else if (name == "to_world")
            {
                if (std::string(child.name()) == "transform")
                {
                    to_world = ParseTransform(child, dm);
                }
            }
            else if (name == "face_normals")
            {
                face_normals = ParseBoolean(child.attribute("value"), dm);
            }
            else if (name == "use_model_material")
            {
                use_model_material = ParseBoolean(child.attribute("value"), dm);
            }
        }

        ModelLoaderOptions options;
        options.gen_smooth_normal = true;
        options.use_fallback_material = !use_model_material;
        options.fallback_material = mat;
        options.fallback_medium_interface = mi;

        LoadModel(*scene, filename, to_world, options);
    }
    else
    {
        std::cerr << "Shape type not supported: " << shape_type << std::endl;
    }
}

static void ParseLight(pugi::xml_node node, const DefaultMap& dm, Scene* scene)
{
    std::string type = node.attribute("type").value();

    if (type == "point")
    {
        Vec3 position(0);
        Spectrum intensity(1);
        Transform to_world = identity;

        for (auto child : node.children())
        {
            std::string name = child.attribute("name").value();
            if (name == "position")
            {
                position = ParseVec3(child.attribute("value"), dm);
            }
            else if (name == "intensity")
            {
                intensity = ParseIntensity(child, dm);
            }
            else if (name == "to_world")
            {
                to_world = ParseTransform(child, dm);
            }
        }

        scene->CreateLight<PointLight>(Mul(to_world, position), intensity, nullptr);
    }
    else if (type == "spot")
    {
        Spectrum intensity(1);
        Float cutoff_angle = 20;
        Float beam_width = cutoff_angle * 3 / 4.0f;
        Transform to_world = identity;

        for (auto child : node.children())
        {
            std::string name = child.attribute("name").value();
            if (name == "intensity")
            {
                intensity = ParseIntensity(child, dm);
            }
            else if (name == "cutoff_angle")
            {
                cutoff_angle = ParseFloat(child.attribute("value"), dm);
            }
            else if (name == "beam_width")
            {
                beam_width = ParseFloat(child.attribute("value"), dm);
            }
            else if (name == "to_world")
            {
                to_world = ParseTransform(child, dm);
            }
        }

        scene->CreateLight<SpotLight>(to_world.p, to_world.q.GetBasisZ(), intensity, cutoff_angle, beam_width, nullptr);
    }
    else if (type == "directional")
    {
        Spectrum irradiance(1);
        Transform to_world = identity;
        Vec3 direction(0, 0, 1);

        for (auto child : node.children())
        {
            std::string name = child.attribute("name").value();
            if (name == "irradiance")
            {
                irradiance = ParseIntensity(child, dm);
            }
            else if (name == "to_world")
            {
                to_world = ParseTransform(child, dm);
            }
            else if (name == "direction")
            {
                direction = ParseVec3(child.attribute("value"), dm);
            }
        }

        scene->CreateLight<DirectionalLight>(to_world.q.Rotate(direction), irradiance);
    }
    else if (type == "constant")
    {
        Spectrum radiance(1);

        for (auto child : node.children())
        {
            std::string name = child.attribute("name").value();
            if (name == "radiance")
            {
                radiance = ParseIntensity(child, dm);
            }
        }

        scene->CreateLight<UniformInfiniteLight>(radiance);
    }
    else if (type == "envmap")
    {
        std::string filename;
        Float scale = 1;
        Transform to_world = identity;

        for (auto child : node.children())
        {
            std::string name = child.attribute("name").value();

            if (name == "filename")
            {
                filename = ParseString(child.attribute("value"), dm);
            }
            else if (name == "to_world")
            {
                to_world = ParseTransform(child, dm);
            }
            else if (name == "scale")
            {
                scale = ParseFloat(child.attribute("value"), dm);
            }
        }

        if (filename.size() > 0)
        {
            CreateImageInfiniteLight(*scene, filename, to_world, scale);
        }
        else
        {
            std::cerr << "Filename unspecified for envmap" << std::endl;
        }
    }
    else
    {
        std::cerr << "Light not supported: " << type << std::endl;
    }
}

static bool ParseScene(RendererInfo* ri, pugi::xml_node scene_node)
{
    DefaultMap dm;
    MaterialMap mm;
    MediumMap mdm;

    for (auto node : scene_node.children())
    {
        std::string name = node.name();

        if (name == "default")
        {
            ParseDefault(node, dm);
        }
        else if (name == "integrator")
        {
            ParseIntegrator(node, dm, ri->integrator_info);
        }
        else if (name == "sensor")
        {
            ParseCamera(node, dm, ri->camera_info);
        }
        else if (name == "bsdf")
        {
            ParseMaterial(node, dm, mm, &ri->scene, {});
        }
        else if (name == "medium")
        {
            ParseMedium(node, dm, mdm, &ri->scene);
        }
        else if (name == "emitter")
        {
            ParseLight(node, dm, &ri->scene);
        }
        else if (name == "shape")
        {
            ParseShape(node, dm, mm, mdm, &ri->scene);
        }
    }

    return true;
}

bool LoadScene(RendererInfo* ri, std::filesystem::path filename)
{
    pugi::xml_document doc;
    pugi::xml_parse_result xml_result = doc.load_file(filename.c_str());
    if (!xml_result)
    {
        std::cerr << "Failed to load xml.." << std::endl;
        std::cerr << "Description: " << xml_result.description() << std::endl;
        return false;
    }

    fs::path old_path = fs::current_path();
    fs::current_path(filename.parent_path());
    bool result = ParseScene(ri, doc.child("scene"));
    fs::current_path(old_path);

    return result;
}

} // namespace bulbit

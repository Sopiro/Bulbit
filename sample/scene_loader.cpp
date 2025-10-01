#include "scene_loader.h"

#include "bulbit/color.h"
#include <pugixml.hpp>

#include "model_loader.h"
#include "string_util.h"

#include "light_builder.h"
#include "material_builder.h"
#include "scene_builder.h"
#include "texture_builder.h"

namespace fs = std::filesystem;

namespace bulbit
{

using DefaultMap = std::unordered_map<std::string, std::string>;
using MaterialMap = std::unordered_map<std::string, const Material*>;

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
            return CreateFloatImageTexture(*scene, t.filename.string(), 0, true, std::move(transform));
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
    else if (type == "normalmap")
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
    else if (type == "blendbsdf")
    {
        const FloatTexture* weight = nullptr;

        for (auto child : node.children())
        {
            std::string name = child.attribute("name").value();
            if (name == "weight")
            {
                weight = ParseFloatTexture(child, dm, scene);
            }
        }

        if (!weight)
        {
            std::cerr << "Weight texture not specified for blendbsdf" << std::endl;
        }

        int32 index = 0;
        const Material* mat[2];
        for (auto child : node.children())
        {
            std::string type = child.name();
            if (type == "bsdf")
            {
                mat[index++] = ParseMaterial(child, dm, mm, scene, mi, id);
            }

            if (index == 2)
            {
                break;
            }
        }

        return scene->CreateMaterial<MixtureMaterial>(mat[0], mat[1], weight, mi.alpha);
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
    else if (type == "twosided")
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
        }

        int32 index = 0;
        const Material* mat[2];
        for (auto child : node.children())
        {
            std::string type = child.name();
            if (type == "bsdf")
            {
                mat[index++] = ParseMaterial(child, dm, mm, scene, mi, id);
            }

            if (index == 2)
            {
                break;
            }
        }

        return scene->CreateMaterial<LayeredMaterial>(
            mat[0], mat[1], mi.two_sided, albedo, thickness, g, max_bounces, samples, mi.normal, mi.alpha
        );
    }
    else if (type == "diffuse")
    {
        SpectrumTexture* reflectance = CreateSpectrumConstantTexture(*scene, 1, 0, 1);
        for (auto child : node.children())
        {
            std::string name = child.attribute("name").value();
            if (name == "reflectance")
            {
                reflectance = ParseSpectrumTexture(child, dm, scene);
            }
        }

        const Material* mat = scene->CreateMaterial<DiffuseMaterial>(reflectance, nullptr, mi.normal, mi.alpha);
        mm[id] = mat;
    }
    else if (type == "roughplastic" || type == "plastic")
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
            if (name == "diffuse_reflectance")
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
        }

        Float eta = int_ior / ext_ior;

        const SpectrumTexture* reflectance = CreateSpectrumConstantTexture(*scene, 1);
        const Material* top = scene->CreateMaterial<DielectricMaterial>(eta, roughness, roughness, reflectance);
        const Material* bottom = scene->CreateMaterial<DiffuseMaterial>(basecolor);

        const Material* mat = scene->CreateMaterial<LayeredMaterial>(
            top, bottom, mi.two_sided, Spectrum::black, 1e-4f, 0, 16, 1, mi.normal, mi.alpha
        );
        mm[id] = mat;
    }
    else if (type == "roughconductor" || type == "conductor")
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
            else if (name == "specular_reflectance")
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
        }

        const Material* mat;
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

        mm[id] = mat;
        return mat;
    }
    else if (type == "roughdielectric" || type == "dielectric")
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
            else if (name == "reflectance")
            {
                reflectance = ParseSpectrumTexture(child, dm, scene);
            }
        }

        Float eta = int_ior / ext_ior;
        const Material* mat =
            scene->CreateMaterial<DielectricMaterial>(eta, u_roughness, v_roughness, reflectance, true, mi.normal);
        mm[id] = mat;
        return mat;
    }
    else if (type == "thindielectric")
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
        }

        Float eta = int_ior / ext_ior;
        const Material* mat = scene->CreateMaterial<ThinDielectricMaterial>(eta, reflectance);
        mm[id] = mat;
        return mat;
    }
    else
    {
        std::cerr << "BSDF not supported: " << type << std::endl;
    }

    return nullptr;
}

static void ParseShape(pugi::xml_node node, const DefaultMap& dm, MaterialMap& mm, Scene* scene)
{
    const Material* mat = nullptr;
    bool emitter = false;
    Spectrum radiance(1);

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
            else if (!mm.contains(id.value()))
            {
                std::cerr << "Material not found by id: " << id.value() << std::endl;
                mat = mm["fallback"];
            }
            else
            {
                mat = mm.at(id.value());
            }
        }
        else if (name == "emitter")
        {
            emitter = true;

            for (auto grand_child : child.children())
            {
                std::string name = grand_child.attribute("name").value();
                if (name == "radiance")
                {
                    radiance = ParseIntensity(grand_child, dm);
                }
            }

            mat = CreateDiffuseLightMaterial(*scene, radiance);
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

        CreateSphere(*scene, center, radius, mat, {}, emitter);
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

        CreateRectXY(*scene, Mul(to_world, Transform(0, 0, 0, identity, Point3{ 2 })), mat, {}, { 1, 1 }, emitter);
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

        CreateBox(*scene, to_world * Transform(0, 0, 0, identity, { 2, 2, 2 }), mat, {}, { 1, 1 }, emitter);
    }
    else if (shape_type == "obj" || shape_type == "gltf")
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
        }

        ModelLoaderOptions options;
        options.gen_smooth_normal = true;
        options.use_fallback_material = true;
        options.fallback_material = mat;

        LoadOBJ(*scene, filename, to_world, options);
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
}

static bool ParseScene(RendererInfo* ri, pugi::xml_node scene_node)
{
    DefaultMap dm;
    MaterialMap mm;

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
        else if (name == "shape")
        {
            ParseShape(node, dm, mm, &ri->scene);
        }
        else if (name == "emitter")
        {
            ParseLight(node, dm, &ri->scene);
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

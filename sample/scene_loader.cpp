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
            tf = Mul(Quat(angle, Vec3(x, y, z)), tf);
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

static FilmInfo ParseFilm(pugi::xml_node node, const DefaultMap& dm)
{
    FilmInfo fi;
    fi.filename = "bulbit_render.hdr";
    fi.resolution = { 1280, 720 };

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
            // TODO: Fill here
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
    ci.tf = identity;
    ci.fov = 35;
    ci.aperture = 0;
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
                ci.tf = ParseTransform(child, dm);
            }
            else if (name == "fov_axis")
            {
                fov_axis = child.attribute("value").value();
            }
            else if (name == "aperture_radius")
            {
                ci.aperture = ParseFloat(child.attribute("value"), dm);
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

static void ParseIntegrator(pugi::xml_node node, DefaultMap& dm, RendererInfo& ri)
{
    std::string type = ParseString(node.attribute("type"), dm);

    ri.type = IntegratorType::path;
    ri.max_bounces = 8;
    ri.rr_depth = 1;
    ri.ao_range = 0.1f;
    ri.n_photons = 100000;
    ri.initial_radius = -1;

    // TODO: add all types
    // clang-format off
    if (type == "path") { ri.type = IntegratorType::path; }
    else if (type == "volpath") { ri.type = IntegratorType::vol_path; }
    else if (type == "light_path") { ri.type = IntegratorType::light_path; }
    else if (type == "light_vol_path") { ri.type = IntegratorType::light_vol_path; }
    else if (type == "bdpt") { ri.type = IntegratorType::bdpt; }
    else if (type == "vol_bdpt") { ri.type = IntegratorType::vol_bdpt; }
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
            ri.rr_depth = ParseInteger(node.attribute("value"), dm);
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

static FloatTexture* ParseFloatTexture(pugi::xml_node node, const DefaultMap& dm, Scene* scene)
{
    std::string type = node.name();
    if (type == "float")
    {
        Float value = ParseFloat(node.attribute("value"), dm);
        return CreateFloatConstantTexture(*scene, value);
    }
    else if (type == "texture")
    {
        TextureInfo t = ParseTexture(node, dm);

        if (t.type == TextureType::bitmap)
        {
            return CreateFloatImageTexture(*scene, t.filename.string(), 0);
        }
        else if (t.type == TextureType::checkboard)
        {
            return CreateFloatCheckerTexture(*scene, t.color0.Average(), t.color1.Average(), t.scale);
        }
        else
        {
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

static SpectrumTexture* ParseSpectrumTexture(pugi::xml_node node, const DefaultMap& dm, Scene* scene)
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
        return CreateSpectrumConstantTexture(*scene, rgb.x, rgb.y, rgb.z);
    }
    else if (type == "srgb")
    {
        Vec3 rgb = RGB_from_sRGB(ParseVec3(node.attribute("value"), dm));
        return CreateSpectrumConstantTexture(*scene, rgb.x, rgb.y, rgb.z);
    }
    else if (type == "texture")
    {
        TextureInfo t = ParseTexture(node, dm);

        if (t.type == TextureType::bitmap)
        {
            return CreateSpectrumImageTexture(*scene, t.filename.string());
        }
        else if (t.type == TextureType::checkboard)
        {
            return CreateSpectrumCheckerTexture(*scene, t.color0, t.color1, t.scale);
        }
        else
        {
            return CreateSpectrumConstantTexture(*scene, 1, 0, 1);
        }
    }
    else if (type == "ref")
    {
        // TODO: implement
        return CreateSpectrumConstantTexture(*scene, 1, 0, 1);
    }
    else
    {
        std::cerr << "Unknown spectrum texture type: " << type << std::endl;
        return CreateSpectrumConstantTexture(*scene, 1, 0, 1);
    }
}

static bool ParseBSDF(pugi::xml_node node, DefaultMap& dm, MaterialMap& mm, Scene* scene, const std::string& parent_id = "")
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
                return ParseBSDF(child, dm, mm, scene, id);
            }
        }
    }
    else if (type == "normalmap")
    {
        // TODO: implement properly
        for (auto child : node.children())
        {
            if (std::string(child.name()) == "bsdf")
            {
                return ParseBSDF(child, dm, mm, scene, id);
            }
        }
    }
    else if (type == "blendbsdf")
    {
        // TODO: implement properly
        for (auto child : node.children())
        {
            if (std::string(child.name()) == "bsdf")
            {
                return ParseBSDF(child, dm, mm, scene, id);
            }
        }
    }
    else if (type == "null")
    {
    }
    else if (type == "mask")
    {
        // TODO: implement properly
        for (auto child : node.children())
        {
            if (std::string(child.name()) == "bsdf")
            {
                return ParseBSDF(child, dm, mm, scene, id);
            }
        }
    }
    else if (type == "twosided")
    {
        for (auto child : node.children())
        {
            if (std::string(child.name()) == "bsdf")
            {
                return ParseBSDF(child, dm, mm, scene, id);
            }
        }
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

        mm[id] = scene->CreateMaterial<DiffuseMaterial>(reflectance);
    }
    else if (type == "roughplastic" || type == "plastic")
    {
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
        }

        mm[id] = scene->CreateMaterial<MetallicRoughnessMaterial>(basecolor, metallic, roughness, roughness);
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
                    reflectance = CreateSpectrumConstantTexture(*scene, 1);
                }
                else
                {
                    std::cerr << "Conductor type not supported: " << conductor_type << std::endl;
                }
            }
            else if (name == "eta")
            {
                eta = ParseSpectrumTexture(child, dm, scene);
            }
            else if (name == "k")
            {
                k = ParseSpectrumTexture(child, dm, scene);
            }
            else if (name == "specular_reflectance")
            {
                reflectance = ParseSpectrumTexture(child, dm, scene);
            }
            else if (name == "alpha")
            {
                u_roughness = ParseFloatTexture(child, dm, scene);
                v_roughness = u_roughness;
            }
        }

        if (reflectance)
        {
            mm[id] = scene->CreateMaterial<ConductorMaterial>(reflectance, u_roughness, v_roughness);
        }
        else if (eta == nullptr || k == nullptr)
        {
            mm[id] = scene->CreateMaterial<MirrorMaterial>(reflectance);
        }
        else
        {
            mm[id] = scene->CreateMaterial<ConductorMaterial>(eta, k, u_roughness, v_roughness);
        }
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
                u_roughness = ParseFloatTexture(child, dm, scene);
                v_roughness = u_roughness;
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

        mm[id] = scene->CreateMaterial<DielectricMaterial>(eta, u_roughness, v_roughness, reflectance);
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
        mm[id] = scene->CreateMaterial<ThinDielectricMaterial>(eta, reflectance);
    }
    else
    {
        return true;
    }

    return true;
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

static bool ParseShape(pugi::xml_node node, DefaultMap& dm, MaterialMap& mm, Scene* scene)
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
        return true;
    }

    return true;
}

static SceneInfo ParseScene(pugi::xml_node scene_node)
{
    SceneInfo si;
    si.scene = std::make_unique<Scene>();

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
            ParseIntegrator(node, dm, si.renderer_info);
        }
        else if (name == "sensor")
        {
            ParseCamera(node, dm, si.camera_info);
        }
        else if (name == "bsdf")
        {
            ParseBSDF(node, dm, mm, si.scene.get());
        }
        else if (name == "shape")
        {
            ParseShape(node, dm, mm, si.scene.get());
        }
    }

    return si;
}

SceneInfo LoadScene(std::filesystem::path filename)
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

SceneInfo::operator bool() const
{
    return bool(scene);
}

} // namespace bulbit

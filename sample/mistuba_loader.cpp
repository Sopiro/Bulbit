#include "mistuba_loader.h"
#include "bulbit/color.h"
#include "light_builder.h"
#include "loader.h"
#include "material_builder.h"
#include "scene_builder.h"
#include "texture_builder.h"

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

static Float VFovToHFov(Float vfov, Float aspect)
{
    return RadToDeg(2.0f * std::atan(aspect * std::tan(DegToRad(vfov) * 0.5f)));
}

static Float HFovToVFov(Float hfov, Float aspect)
{
    return RadToDeg(2.0f * std::atan(std::tan(DegToRad(hfov) * 0.5f) / aspect));
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

Vec3 ParseSRGB(const std::string& value)
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

Vec3 ParseSRGB(pugi::xml_attribute attr, const DefaultMap& dm)
{
    std::string str = ParseString(attr, dm);
    return ParseSRGB(str);
}

static Transform ParseTransform(pugi::xml_node node, const DefaultMap& dm)
{
    Transform tf = identity;

    for (auto child : node.children())
    {
        std::string name = to_lowercase(child.name());

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
        Point3 f = Mul(to_world, Point3(0));
        Point3 t = Mul(to_world, Point3(0, 0, 1));

        Float aspect = Float(film.width) / Float(film.height);
        fov = HFovToVFov(fov, aspect);

        return std::make_unique<PerspectiveCamera>(f, t, y_axis, fov, 0, 1, Point2i{ film.width, film.height });
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

static Spectrum ParseColor(pugi::xml_node node, const DefaultMap& dm)
{
    std::string type = node.name();
    if (type == "spectrum")
    {
        std::cerr << "Not a supported color type: spectrum" << std::endl;
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
    BITMAP,
    CHECKERBOARD
};

struct ParsedTexture
{
    TextureType type;
    fs::path filename;
    Spectrum color0, color1; // for checkerboard
    Float uscale = 1, vscale = 1;
    Float uoffset = 0, voffset = 0;
};

ParsedTexture ParseTexture(pugi::xml_node node, const DefaultMap& dm)
{
    std::string type = node.attribute("type").value();
    if (type == "bitmap")
    {
        std::string filename = "";
        Float uscale = 1;
        Float vscale = 1;
        Float uoffset = 0;
        Float voffset = 0;
        for (auto child : node.children())
        {
            std::string name = child.attribute("name").value();
            if (name == "filename")
            {
                filename = ParseString(child.attribute("value"), dm);
            }
            else if (name == "uvscale")
            {
                uscale = vscale = ParseFloat(child.attribute("value"), dm);
            }
            else if (name == "uscale")
            {
                uscale = ParseFloat(child.attribute("value"), dm);
            }
            else if (name == "vscale")
            {
                vscale = ParseFloat(child.attribute("value"), dm);
            }
            else if (name == "uoffset")
            {
                uoffset = ParseFloat(child.attribute("value"), dm);
            }
            else if (name == "voffset")
            {
                voffset = ParseFloat(child.attribute("value"), dm);
            }
        }

        return ParsedTexture{
            TextureType::BITMAP, fs::path(filename), Spectrum(0), Spectrum(0), uscale, vscale, uoffset, voffset
        };
    }
    else if (type == "checkerboard")
    {
        Spectrum color0 = { 0.4f, 0.4f, 0.4f };
        Spectrum color1 = { 0.2f, 0.2f, 0.2f };
        Float uscale = 1;
        Float vscale = 1;
        Float uoffset = 0;
        Float voffset = 0;
        for (auto child : node.children())
        {
            std::string name = child.attribute("name").value();
            if (name == "color0")
            {
                color0 = ParseColor(child, dm);
            }
            else if (name == "color1")
            {
                color1 = ParseColor(child, dm);
            }
            else if (name == "uvscale")
            {
                uscale = vscale = ParseFloat(child.attribute("value"), dm);
            }
            else if (name == "uscale")
            {
                uscale = ParseFloat(child.attribute("value"), dm);
            }
            else if (name == "vscale")
            {
                vscale = ParseFloat(child.attribute("value"), dm);
            }
            else if (name == "uoffset")
            {
                uoffset = ParseFloat(child.attribute("value"), dm);
            }
            else if (name == "voffset")
            {
                voffset = ParseFloat(child.attribute("value"), dm);
            }
        }
        return ParsedTexture{ TextureType::CHECKERBOARD, "", color0, color1, uscale, vscale, uoffset, voffset };
    }

    std::cerr << "Unknown texture type: " << type;
    return ParsedTexture{};
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
        ParsedTexture t = ParseTexture(node, dm);
        if (t.type == TextureType::BITMAP)
        {
            return CreateFloatImageTexture(*scene, t.filename.string(), 0);
        }
        else if (t.type == TextureType::CHECKERBOARD)
        {
            return CreateFloatCheckerTexture(*scene, t.color0.Average(), t.color1.Average(), { t.uscale, t.vscale });
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
        ParsedTexture t = ParseTexture(node, dm);

        if (t.type == TextureType::BITMAP)
        {
            return CreateSpectrumImageTexture(*scene, t.filename.string());
        }
        else if (t.type == TextureType::CHECKERBOARD)
        {
            return CreateSpectrumCheckerTexture(*scene, t.color0, t.color1, { t.uscale, t.vscale });
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
        // TODO: implement properly
        for (auto child : node.children())
        {
            if (std::string(child.name()) == "bsdf")
            {
                return ParseBSDF(child, dm, mm, scene, id);
            }
        }
    }
    if (type == "mask")
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
            if (name == "diffuseReflectance" || name == "diffuse_reflectance")
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
                    std::cerr << "Unsupported conductor material: " << conductor_type << std::endl;
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
            else if (name == "reflectance")
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
            else if (name == "intIOR" || name == "int_ior")
            {
                int_ior = ParseFloat(child.attribute("value"), dm);
            }
            else if (name == "extIOR" || name == "ext_ior")
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
            if (name == "intIOR" || name == "int_ior")
            {
                int_ior = ParseFloat(child.attribute("value"), dm);
            }
            else if (name == "extIOR" || name == "ext_ior")
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

Spectrum ParseIntensity(pugi::xml_node node, const DefaultMap& dm)
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
                std::cerr << "Material not found by id: " << std::string(id.value()) << std::endl;
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
            if (name == "toWorld" || name == "to_world")
            {
                if (std::string(child.name()) == "transform")
                {
                    to_world = ParseTransform(child, dm);
                }
            }
            else if (name == "flipNormals" || name == "flip_normals")
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
            if (name == "toWorld" || name == "to_world")
            {
                if (std::string(child.name()) == "transform")
                {
                    to_world = ParseTransform(child, dm);
                }
            }
        }

        CreateBox(*scene, to_world * Transform(0, 0, 0, identity, { 2, 2, 2 }), mat, {}, { 1, 1 }, emitter);
    }
    else if (shape_type == "obj")
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
        SetLoaderUseForceFallbackMaterial(true);
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
        integrator = std::make_unique<PathIntegrator>(accel.get(), scene->GetLights(), sampler.get(), 8, false);
    }
    else if (integrator_type == "volpath")
    {
        integrator = std::make_unique<VolPathIntegrator>(accel.get(), scene->GetLights(), sampler.get(), 8, false);
    }
    else if (integrator_type == "bdpt")
    {
        integrator = std::make_unique<BiDirectionalPathIntegrator>(accel.get(), scene->GetLights(), sampler.get(), 8);
    }
    else if (integrator_type == "ao")
    {
        integrator = std::make_unique<AmbientOcclusion>(accel.get(), scene->GetLights(), sampler.get(), 0.1f);
    }
    else if (integrator_type == "debug")
    {
        integrator = std::make_unique<DebugIntegrator>(accel.get(), scene->GetLights(), sampler.get());
    }
    else
    {
        std::cout << "Not a supported integrator: " << integrator_type << std::endl;
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

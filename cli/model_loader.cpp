#include "model_loader.h"
#include "primitive_builder.h"

#define TINYOBJLOADER_IMPLEMENTATION
// #define TINYOBJLOADER_USE_MAPBOX_EARCUT
#include <tiny_obj_loader.h>

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-literal-operator"
#define TINYGLTF_NO_STB_IMAGE
#define TINYGLTF_IMPLEMENTATION
#include <tiny_gltf.h>
#pragma clang diagnostic pop

#include "material_builder.h"
#include "texture_builder.h"

namespace bulbit
{

constexpr int32 channel_ao = 0;
constexpr int32 channel_roughness = 1;
constexpr int32 channel_metallic = 2;
constexpr int32 channel_alpha = 3;
constexpr int32 channel_anisotropy_strength = 2;
constexpr int32 channel_transmission = 0;
constexpr int32 channel_clearcoat = 0;
constexpr int32 channel_sheen_roughness = 4;

static bool HasExtension(const tinygltf::Material& material, const char* extension_name)
{
    return material.extensions.contains(extension_name);
}

static void LoadMaterials(
    Scene& scene,
    tinygltf::Model& model,
    const std::string& base_path,
    std::vector<std::pair<const Material*, const SpectrumTexture*>> materials
)
{
    for (int32 i = 0; i < int32(model.materials.size()); i++)
    {
        tinygltf::Material& gltf_material = model.materials[i];
        tinygltf::PbrMetallicRoughness& pbr = gltf_material.pbrMetallicRoughness;

        SpectrumTexture* basecolor_texture = nullptr;
        FloatTexture* metallic_texture = nullptr;
        FloatTexture* roughness_texture = nullptr;
        FloatTexture* anisotropy_texture = nullptr;
        FloatTexture* transmission_texture = nullptr;
        FloatTexture* clearcoat_texture = nullptr;
        FloatTexture* clearcoat_roughness_texture = nullptr;
        SpectrumTexture* clearcoat_color_texture = nullptr;
        FloatTexture* sheen_texture = nullptr;
        SpectrumTexture* sheen_color_texture = nullptr;
        FloatTexture* sheen_roughness_texture = nullptr;
        SpectrumTexture* emission_texture = nullptr;
        SpectrumTexture* normal_texture = nullptr;
        FloatTexture* alpha_texture = nullptr;

        Spectrum basecolor_factor = { Float(pbr.baseColorFactor[0]), Float(pbr.baseColorFactor[1]),
                                      Float(pbr.baseColorFactor[2]) };
        Float metallic_factor = Float(pbr.metallicFactor);
        Float roughness_factor = Float(pbr.roughnessFactor);
        Float anisotropy_factor = 0.0f;
        Float ior_factor = 1.5f;
        Float transmission_factor = 0.0f;
        Float clearcoat_factor = 0.0f;
        Float clearcoat_roughness_factor = 0.0f;
        Spectrum clearcoat_color_factor = { 1.0f, 1.0f, 1.0f };
        Spectrum sheen_color_factor = { 0.0f, 0.0f, 0.0f };
        Float sheen_roughness_factor = 0.0f;
        Spectrum emission_factor = { Float(gltf_material.emissiveFactor[0]), Float(gltf_material.emissiveFactor[1]),
                                     Float(gltf_material.emissiveFactor[2]) };

        // basecolor, alpha
        {
            if (pbr.baseColorTexture.index > -1)
            {
                tinygltf::Texture& texture = model.textures[pbr.baseColorTexture.index];
                tinygltf::Image& image = model.images[texture.source];

                basecolor_texture = CreateSpectrumImageTexture(scene, base_path + image.uri, false);
                alpha_texture = CreateFloatImageTexture(scene, base_path + image.uri, channel_alpha, true);
            }
            else
            {
                basecolor_texture = CreateSpectrumConstantTexture(scene, basecolor_factor);
                alpha_texture = nullptr;
            }
        }

        // metallic, roughness
        {
            if (pbr.metallicRoughnessTexture.index > -1)
            {
                tinygltf::Texture& texture = model.textures[pbr.metallicRoughnessTexture.index];
                tinygltf::Image& image = model.images[texture.source];

                metallic_texture = CreateFloatImageTexture(scene, base_path + image.uri, channel_metallic, true, [=](Float v) {
                    return v * metallic_factor;
                });
                roughness_texture = CreateFloatImageTexture(scene, base_path + image.uri, channel_roughness, true, [=](Float v) {
                    return v * roughness_factor;
                });
            }
            else
            {
                metallic_texture = CreateFloatConstantTexture(scene, metallic_factor);
                roughness_texture = CreateFloatConstantTexture(scene, roughness_factor);
            }
        }

        // normal
        {
            if (gltf_material.normalTexture.index > -1)
            {
                tinygltf::Texture& texture = model.textures[gltf_material.normalTexture.index];
                tinygltf::Image& image = model.images[texture.source];

                normal_texture = CreateSpectrumImageTexture(scene, base_path + image.uri, true);
            }
            else
            {
                normal_texture = nullptr;
            }
        }

        // emission
        {
            if (emission_factor != Spectrum::black)
            {
                if (gltf_material.emissiveTexture.index > -1)
                {
                    tinygltf::Texture& texture = model.textures[gltf_material.emissiveTexture.index];
                    tinygltf::Image& image = model.images[texture.source];

                    emission_texture = CreateSpectrumImageTexture(scene, base_path + image.uri, false, [=](Spectrum v) {
                        return v * emission_factor;
                    });
                }
                else
                {
                    emission_texture = CreateSpectrumConstantTexture(scene, emission_factor);
                }
            }
        }

        // ior
        {
            // https://github.com/KhronosGroup/glTF/blob/main/extensions/2.0/Khronos/KHR_materials_ior/README.md
            if (HasExtension(gltf_material, "KHR_materials_ior"))
            {
                const tinygltf::Value& ext = gltf_material.extensions.at("KHR_materials_ior");
                if (ext.Has("ior"))
                {
                    ior_factor = Float(ext.Get("ior").Get<double>());
                }
            }
        }

        // anisotropy
        {
            // https://github.com/KhronosGroup/glTF/blob/main/extensions/2.0/Khronos/KHR_materials_anisotropy/README.md
            if (HasExtension(gltf_material, "KHR_materials_anisotropy"))
            {
                const tinygltf::Value& ext = gltf_material.extensions.at("KHR_materials_anisotropy");

                if (ext.Has("anisotropyStrength"))
                {
                    anisotropy_factor = Float(ext.Get("anisotropyStrength").Get<double>());
                }

                if (ext.Has("anisotropyTexture"))
                {
                    const tinygltf::Value& tex = ext.Get("anisotropyTexture");
                    int32 tex_index = -1;
                    if (tex.Has("index"))
                    {
                        tex_index = tex.Get("index").Get<int>();
                    }

                    if (tex_index > 0)
                    {
                        tinygltf::Texture& texture = model.textures[tex_index];
                        tinygltf::Image& image = model.images[texture.source];
                        anisotropy_texture = CreateFloatImageTexture(
                            scene, base_path + image.uri, channel_anisotropy_strength, true,
                            [=](Float v) { return v * anisotropy_factor; }
                        );
                    }
                }

                // Todo: support this
                // if (ext.Has("anisotropyRotation")) {}
            }

            if (anisotropy_texture == nullptr)
            {
                anisotropy_texture = CreateFloatConstantTexture(scene, anisotropy_factor);
            }
        }

        // transmission
        {
            // https://github.com/KhronosGroup/glTF/blob/main/extensions/2.0/Khronos/KHR_materials_transmission/README.md
            if (HasExtension(gltf_material, "KHR_materials_transmission"))
            {
                const tinygltf::Value& ext = gltf_material.extensions.at("KHR_materials_transmission");

                if (ext.Has("transmissionFactor"))
                {
                    transmission_factor = Float(ext.Get("transmissionFactor").Get<double>());
                }

                if (ext.Has("transmissionTexture"))
                {
                    const tinygltf::Value& tex = ext.Get("transmissionTexture");
                    int32 tex_index = -1;
                    if (tex.Has("index"))
                    {
                        tex_index = tex.Get("index").Get<int>();
                    }

                    if (tex_index > 0)
                    {
                        tinygltf::Texture& texture = model.textures[tex_index];
                        tinygltf::Image& image = model.images[texture.source];
                        transmission_texture =
                            CreateFloatImageTexture(scene, base_path + image.uri, channel_transmission, true, [=](Float v) {
                                return v * transmission_factor;
                            });
                    }
                }
            }

            if (transmission_texture == nullptr)
            {
                transmission_texture = CreateFloatConstantTexture(scene, transmission_factor);
            }
        }

        // clearcoat
        {
            // https://github.com/KhronosGroup/glTF/blob/main/extensions/2.0/Khronos/KHR_materials_clearcoat/README.md
            if (HasExtension(gltf_material, "KHR_materials_clearcoat"))
            {
                const tinygltf::Value& ext = gltf_material.extensions.at("KHR_materials_clearcoat");

                if (ext.Has("clearcoatFactor"))
                {
                    clearcoat_factor = Float(ext.Get("clearcoatFactor").Get<double>());
                }

                if (ext.Has("clearcoatTexture"))
                {
                    const tinygltf::Value& tex = ext.Get("clearcoatTexture");
                    int32 tex_index = -1;
                    if (tex.Has("index"))
                    {
                        tex_index = tex.Get("index").Get<int>();
                    }

                    if (tex_index > 0)
                    {
                        tinygltf::Texture& texture = model.textures[tex_index];
                        tinygltf::Image& image = model.images[texture.source];
                        clearcoat_texture =
                            CreateFloatImageTexture(scene, base_path + image.uri, channel_clearcoat, true, [=](Float v) {
                                return v * clearcoat_factor;
                            });
                    }
                }

                if (ext.Has("clearcoatRoughnessFactor"))
                {
                    clearcoat_roughness_factor = Float(ext.Get("clearcoatRoughnessFactor").Get<double>());
                }

                if (ext.Has("clearcoatRoughnessTexture"))
                {
                    const tinygltf::Value& tex = ext.Get("clearcoatRoughnessTexture");
                    int32 tex_index = -1;
                    if (tex.Has("index"))
                    {
                        tex_index = tex.Get("index").Get<int>();
                    }

                    if (tex_index > 0)
                    {
                        tinygltf::Texture& texture = model.textures[tex_index];
                        tinygltf::Image& image = model.images[texture.source];
                        clearcoat_roughness_texture =
                            CreateFloatImageTexture(scene, base_path + image.uri, channel_roughness, true, [=](Float v) {
                                return v * clearcoat_roughness_factor;
                            });
                    }
                }
            }

            if (clearcoat_texture == nullptr)
            {
                clearcoat_texture = CreateFloatConstantTexture(scene, clearcoat_factor);
            }

            if (clearcoat_roughness_texture == nullptr)
            {
                clearcoat_roughness_texture = CreateFloatConstantTexture(scene, clearcoat_roughness_factor);
            }

            if (clearcoat_color_texture == nullptr)
            {
                clearcoat_color_texture = CreateSpectrumConstantTexture(scene, clearcoat_color_factor);
            }
        }

        // sheen
        {
            // https://github.com/KhronosGroup/glTF/blob/main/extensions/2.0/Khronos/KHR_materials_sheen/README.md
            if (HasExtension(gltf_material, "KHR_materials_sheen"))
            {
                const tinygltf::Value& ext = gltf_material.extensions.at("KHR_materials_sheen");

                if (ext.Has("sheenColorFactor"))
                {
                    auto arr = ext.Get("sheenColorFactor").Get<tinygltf::Value::Array>();
                    sheen_color_factor = { Float(arr[0].Get<double>()), Float(arr[1].Get<double>()),
                                           Float(arr[2].Get<double>()) };
                }

                if (ext.Has("sheenColorTexture"))
                {
                    const tinygltf::Value& tex = ext.Get("sheenColorTexture");
                    int32 tex_index = -1;
                    if (tex.Has("index"))
                    {
                        tex_index = tex.Get("index").Get<int>();
                    }

                    if (tex_index > 0)
                    {
                        tinygltf::Texture& texture = model.textures[tex_index];
                        tinygltf::Image& image = model.images[texture.source];
                        sheen_color_texture = CreateSpectrumImageTexture(scene, base_path + image.uri, false, [=](Spectrum v) {
                            return v * sheen_color_factor;
                        });
                    }
                }

                if (ext.Has("sheenRoughnessFactor"))
                {
                    sheen_roughness_factor = Float(ext.Get("sheenRoughnessFactor").Get<double>());
                }

                if (ext.Has("sheenRoughnessTexture"))
                {
                    const tinygltf::Value& tex = ext.Get("sheenRoughnessTexture");
                    int32 tex_index = -1;
                    if (tex.Has("index"))
                    {
                        tex_index = tex.Get("index").Get<int>();
                    }

                    if (tex_index > 0)
                    {
                        tinygltf::Texture& texture = model.textures[tex_index];
                        tinygltf::Image& image = model.images[texture.source];
                        sheen_roughness_texture =
                            CreateFloatImageTexture(scene, base_path + image.uri, channel_sheen_roughness, true, [=](Float v) {
                                return v * sheen_roughness_factor;
                            });
                    }
                }
            }

            if (sheen_color_factor == Spectrum(0))
            {
                sheen_texture = CreateFloatConstantTexture(scene, 0);
            }
            else
            {
                sheen_texture = CreateFloatConstantTexture(scene, 1);
            }

            if (sheen_color_texture == nullptr)
            {
                sheen_color_texture = CreateSpectrumConstantTexture(scene, sheen_color_factor);
            }

            if (sheen_roughness_texture == nullptr)
            {
                sheen_roughness_texture = CreateFloatConstantTexture(scene, sheen_roughness_factor);
            }
        }

        materials.push_back(
            { scene.CreateMaterial<PrincipledMaterial>(
                  basecolor_texture, metallic_texture, roughness_texture, anisotropy_texture, ior_factor, transmission_texture,
                  clearcoat_texture, clearcoat_roughness_texture, clearcoat_color_texture, sheen_texture, sheen_roughness_texture,
                  sheen_color_texture, normal_texture, alpha_texture
              ),
              emission_texture }
        );
    }
}

static bool LoadMesh(
    Scene& scene,
    tinygltf::Model& model,
    tinygltf::Mesh& mesh,
    const Mat4& transform,
    std::vector<std::pair<const Material*, const SpectrumTexture*>> materials,
    const ModelLoaderOptions& options
)
{
    for (int32 prim = 0; prim < mesh.primitives.size(); ++prim)
    {
        tinygltf::Primitive& primitive = mesh.primitives[prim];

        if (primitive.mode != TINYGLTF_MODE_TRIANGLES)
        {
            continue;
        }

        int32 position_index = primitive.attributes.contains("POSITION") ? primitive.attributes["POSITION"] : -1;
        int32 normal_index = primitive.attributes.contains("NORMAL") ? primitive.attributes["NORMAL"] : -1;
        int32 tangent_index = primitive.attributes.contains("TANGENT") ? primitive.attributes["TANGENT"] : -1;
        int32 texcoord_index = primitive.attributes.contains("TEXCOORD_0") ? primitive.attributes["TEXCOORD_0"] : -1;
        int32 indices_index = int32(primitive.indices);

        if (position_index == -1 || normal_index == -1 || texcoord_index == -1)
        {
            return false;
        }

        // position
        tinygltf::Accessor& position_accessor = model.accessors[position_index];
        tinygltf::BufferView& position_buffer_view = model.bufferViews[position_accessor.bufferView];
        tinygltf::Buffer& position_buffer = model.buffers[position_buffer_view.buffer];

        // normal
        tinygltf::Accessor& normal_accessor = model.accessors[normal_index];
        tinygltf::BufferView& normal_buffer_view = model.bufferViews[normal_accessor.bufferView];
        tinygltf::Buffer& normal_buffer = model.buffers[normal_buffer_view.buffer];

        // texcoord
        tinygltf::Accessor& texcoord_accessor = model.accessors[texcoord_index];
        tinygltf::BufferView& texcoord_buffer_view = model.bufferViews[texcoord_accessor.bufferView];
        tinygltf::Buffer& texcoord_buffer = model.buffers[texcoord_buffer_view.buffer];

        // indices
        tinygltf::Accessor& indices_accessor = model.accessors[indices_index];
        tinygltf::BufferView& indices_buffer_view = model.bufferViews[indices_accessor.bufferView];
        tinygltf::Buffer& indices_buffer = model.buffers[indices_buffer_view.buffer];

        std::vector<Vec3> positions(position_accessor.count);
        std::vector<Vec3> normals(normal_accessor.count);
        std::vector<Vec3> tangents(normal_accessor.count);
        std::vector<Vec2> texcoords(texcoord_accessor.count);
        std::vector<int32> indices(indices_accessor.count);

        size_t position_size = tinygltf::GetComponentSizeInBytes(position_accessor.componentType) *
                               tinygltf::GetNumComponentsInType(position_accessor.type);
        memcpy(
            positions.data(), position_buffer.data.data() + position_buffer_view.byteOffset + position_accessor.byteOffset,
            position_accessor.count * position_size
        );

        size_t normal_size = tinygltf::GetComponentSizeInBytes(normal_accessor.componentType) *
                             tinygltf::GetNumComponentsInType(normal_accessor.type);
        memcpy(
            normals.data(), normal_buffer.data.data() + normal_buffer_view.byteOffset + normal_accessor.byteOffset,
            normal_accessor.count * normal_size
        );

        size_t texcoord_size = tinygltf::GetComponentSizeInBytes(texcoord_accessor.componentType) *
                               tinygltf::GetNumComponentsInType(texcoord_accessor.type);
        memcpy(
            texcoords.data(), texcoord_buffer.data.data() + texcoord_buffer_view.byteOffset + texcoord_accessor.byteOffset,
            texcoord_accessor.count * texcoord_size
        );

        if (tangent_index != -1)
        {
            // tangent
            tinygltf::Accessor& tangent_accessor = model.accessors[tangent_index];
            tinygltf::BufferView& tangent_buffer_view = model.bufferViews[tangent_accessor.bufferView];
            tinygltf::Buffer& tangent_buffer = model.buffers[tangent_buffer_view.buffer];

            size_t tangent_size = tinygltf::GetNumComponentsInType(tangent_accessor.type) *
                                  tinygltf::GetComponentSizeInBytes(tangent_accessor.componentType);

            std::vector<Vec4> temp(tangent_accessor.count);
            memcpy(
                temp.data(), tangent_buffer.data.data() + tangent_buffer_view.byteOffset + tangent_accessor.byteOffset,
                tangent_accessor.count * tangent_size
            );

            for (size_t i = 0; i < tangent_accessor.count; ++i)
            {
                if (temp[i].w == 1)
                {
                    tangents[i].Set(temp[i].x, temp[i].y, temp[i].z);
                }
                else
                {
                    tangents[i].Set(-temp[i].x, -temp[i].y, -temp[i].z);
                }
            }
        }
        else
        {
            for (size_t i = 0; i < normals.size(); ++i)
            {
                Vec3 bitangent;
                CoordinateSystem(normals[i], &tangents[i], &bitangent);
            }
        }

        if (tinygltf::GetNumComponentsInType(indices_accessor.type) != 1)
        {
            return false;
        }

        size_t indices_size = tinygltf::GetComponentSizeInBytes(indices_accessor.componentType);
        switch (indices_size)
        {
        case sizeof(uint8_t):
        {
            std::vector<uint8_t> temp(indices_accessor.count);
            memcpy(
                temp.data(), indices_buffer.data.data() + indices_buffer_view.byteOffset + indices_accessor.byteOffset,
                indices_accessor.count * indices_size
            );
            for (size_t i = 0; i < indices_accessor.count; ++i)
            {
                indices[i] = int32_t(temp[i]);
            }
        }
        break;
        case sizeof(uint16_t):
        {
            std::vector<uint16_t> temp(indices_accessor.count);
            memcpy(
                temp.data(), indices_buffer.data.data() + indices_buffer_view.byteOffset + indices_accessor.byteOffset,
                indices_accessor.count * indices_size
            );
            for (size_t i = 0; i < indices_accessor.count; ++i)
            {
                indices[i] = int32_t(temp[i]);
            }
        }
        break;
        case sizeof(uint32_t):
        {
            std::vector<uint32_t> temp(indices_accessor.count);
            memcpy(
                temp.data(), indices_buffer.data.data() + indices_buffer_view.byteOffset + indices_accessor.byteOffset,
                indices_accessor.count * indices_size
            );
            for (size_t i = 0; i < indices_accessor.count; ++i)
            {
                indices[i] = int32_t(temp[i]);
            }
        }
        break;

        default:
            return false;
        }

        // post-processes
        {
            if (options.flip_normal)
            {
                for (size_t i = 0; i < normals.size(); ++i)
                {
                    normals[i].Negate();
                }
            }

            if (!options.flip_tex_coord)
            {
                for (size_t i = 0; i < texcoords.size(); ++i)
                {
                    texcoords[i].y = -texcoords[i].y;
                }
            }
        }

        Mesh* m = scene.CreateMesh(
            std::move(positions), std::move(normals), std::move(tangents), std::move(texcoords), std::move(indices), transform
        );

        const Material* material;
        const SpectrumTexture* emission = nullptr;

        if (primitive.material < 0)
        {
            if (options.fallback_material)
            {
                material = options.fallback_material;
            }
            else
            {
                std::cerr << "No valid material found and no fallback specified" << std::endl;
                material = CreateDiffuseMaterial(scene, Spectrum{ 1, 0, 1 });
            }
        }
        else
        {
            material = materials[primitive.material].first;
            emission = materials[primitive.material].second;
        }

        std::optional<AreaLightInfo> ali = options.area_light;
        if (!ali && emission)
        {
            ali.emplace();
            ali->emission = emission;
        }

        CreateTriangles(
            scene, m, options.use_fallback_material ? options.fallback_material : material, options.fallback_medium_interface, ali
        );
    }

    return true;
}

static void ProcessNode(
    Scene& scene,
    tinygltf::Model& model,
    tinygltf::Node& node,
    const Mat4& parent_transform,
    std::vector<std::pair<const Material*, const SpectrumTexture*>> materials,
    const ModelLoaderOptions& options
)
{
    Mat4 transform;
    if (!node.matrix.empty())
    {
        transform = Mat4(
            Vec4(Float(node.matrix[0]), Float(node.matrix[1]), Float(node.matrix[2]), Float(node.matrix[3])),
            Vec4(Float(node.matrix[4]), Float(node.matrix[5]), Float(node.matrix[6]), Float(node.matrix[7])),
            Vec4(Float(node.matrix[8]), Float(node.matrix[9]), Float(node.matrix[10]), Float(node.matrix[11])),
            Vec4(Float(node.matrix[12]), Float(node.matrix[13]), Float(node.matrix[14]), Float(node.matrix[15]))
        );
    }
    else
    {
        transform = identity;
    }

    transform = Mul(parent_transform, transform);

    if ((node.mesh >= 0) && (node.mesh < model.meshes.size()))
    {
        LoadMesh(scene, model, model.meshes[node.mesh], transform, materials, options);
    }

    // Recursively process child nodes
    for (size_t i = 0; i < node.children.size(); i++)
    {
        BulbitAssert((node.children[i] >= 0) && (node.children[i] < model.nodes.size()));
        ProcessNode(scene, model, model.nodes[node.children[i]], transform, materials, options);
    }
}

static void LoadScene(
    Scene& scene,
    tinygltf::Model& model,
    const Transform& transform,
    std::vector<std::pair<const Material*, const SpectrumTexture*>> materials,
    const ModelLoaderOptions& options
)
{
    const tinygltf::Scene& gltf_scene = model.scenes[model.defaultScene];
    for (size_t i = 0; i < gltf_scene.nodes.size(); ++i)
    {
        BulbitAssert((gltf_scene.nodes[i] >= 0) && (gltf_scene.nodes[i] < model.nodes.size()));
        ProcessNode(scene, model, model.nodes[gltf_scene.nodes[i]], transform, materials, options);
    }
}

void LoadGLTF(Scene& scene, std::filesystem::path filename, const Transform& transform, const ModelLoaderOptions& options)
{
    if (options.use_fallback_material && options.fallback_material == nullptr)
    {
        std::cerr << "Fallback material not provided!" << std::endl;
        return;
    }

    tinygltf::TinyGLTF gltf;
    gltf.SetImageLoader(
        [](tinygltf::Image* image, const int, std::string*, const std::string*, int, int, const unsigned char*, int,
           void*) -> bool {
            image->image.clear();
            image->width = 0;
            image->height = 0;
            image->component = 0;
            image->bits = 0;
            return true;
        },
        nullptr
    );
    tinygltf::Model model;

    std::string err, warn;

    bool success;
    if (filename.extension() == ".gltf")
    {
        success = gltf.LoadASCIIFromFile(&model, &err, &warn, filename.string());
    }
    else if (filename.extension() == ".glb")
    {
        success = gltf.LoadBinaryFromFile(&model, &err, &warn, filename.string());
    }
    else
    {
        std::cout << "Failed to load model: " << filename.string() << std::endl;
        return;
    }

    if (!success)
    {
        std::cout << "gltf warning: " << warn << std::endl;
        std::cout << "gltf error: " << err << std::endl;
        return;
    }

    std::vector<std::pair<const Material*, const SpectrumTexture*>> materials;
    std::string base_path = filename.remove_filename().string();
    if (!options.use_fallback_material)
    {
        LoadMaterials(scene, model, base_path, materials);
    }
    LoadScene(scene, model, transform, materials, options);
}

std::pair<const Material*, const SpectrumTexture*> CreateOBJMaterial(
    Scene& scene, const tinyobj::material_t& mat, const std::string& root
)
{
    Spectrum basecolor_factor = { Float(mat.diffuse[0]), Float(mat.diffuse[1]), Float(mat.diffuse[2]) };
    Float metallic_factor = 0;
    Float roughness_factor = 1;
    Spectrum emission_factor = { Float(mat.emission[0]), Float(mat.emission[1]), Float(mat.emission[2]) };

    // Create a texture for the diffuse component if available; otherwise use a constant texture.
    SpectrumTexture* basecolor_texture = nullptr;
    FloatTexture* alpha_texture = nullptr;
    if (!mat.diffuse_texname.empty())
    {
        basecolor_texture = CreateSpectrumImageTexture(scene, root + mat.diffuse_texname);
        alpha_texture = CreateFloatImageTexture(scene, root + mat.diffuse_texname, channel_alpha, true);
    }
    else
    {
        basecolor_texture = CreateSpectrumConstantTexture(scene, basecolor_factor);
    }

    // Use constant textures for metallic/roughness as OBJ does not provide them.
    FloatTexture* metallic_texture = CreateFloatConstantTexture(scene, metallic_factor);
    FloatTexture* roughness_texture = CreateFloatConstantTexture(scene, roughness_factor);

    // Use the bump texture as a normal texture if available.
    SpectrumTexture* normal_texture = nullptr;
    if (!mat.bump_texname.empty())
    {
        normal_texture = CreateSpectrumImageTexture(scene, root + mat.bump_texname, true);
    }

    // Create an emission texture if provided, otherwise use a constant emission texture.
    SpectrumTexture* emission_texture = nullptr;
    if (!mat.emissive_texname.empty())
    {
        emission_texture = CreateSpectrumImageTexture(scene, root + mat.emissive_texname);
    }
    else
    {
        emission_texture = CreateSpectrumConstantTexture(scene, emission_factor);
    }

    return { scene.CreateMaterial<MetallicRoughnessMaterial>(
                 basecolor_texture, metallic_texture, roughness_texture, roughness_texture, normal_texture, alpha_texture
             ),
             emission_texture };
}

// Structure to accumulate mesh data grouped by material.
struct OBJMeshGroup
{
    std::vector<Vec3> positions;
    std::vector<Vec3> normals;
    std::vector<Vec3> tangents;
    std::vector<Vec2> texcoords;
    std::vector<int32> indices;
};

// Load OBJ model using tinyobj library.
void LoadOBJ(Scene& scene, std::filesystem::path filename, const Transform& transform, const ModelLoaderOptions& options)
{
    if (options.use_fallback_material && options.fallback_material == nullptr)
    {
        std::cerr << "Fallback material not provided!" << std::endl;
        return;
    }

    // Extract folder path for textures and MTL file loading.
    std::string base_path = filename.parent_path().string();
    if (!base_path.empty() && base_path.back() != '/')
    {
        base_path += "/";
    }

    tinyobj::ObjReaderConfig reader_config;
    reader_config.mtl_search_path = base_path; // MTL file is assumed to be in the same folder as the OBJ.

    tinyobj::ObjReader reader;
    if (!reader.ParseFromFile(filename.string(), reader_config))
    {
        if (!reader.Error().empty())
        {
            std::cerr << "TinyObjReader Error: " << reader.Error() << std::endl;
        }
        return;
    }
    if (!reader.Warning().empty())
    {
        std::cout << "TinyObjReader Warning: " << reader.Warning() << std::endl;
    }

    const auto& attrib = reader.GetAttrib();
    const auto& shapes = reader.GetShapes();
    const auto& obj_materials = reader.GetMaterials();

    // Convert OBJ materials to engine Materials. The index corresponds to the material ID.
    std::vector<std::pair<const Material*, const SpectrumTexture*>> materials;
    if (!options.use_fallback_material)
    {
        materials.resize(obj_materials.size());
        for (size_t i = 0; i < obj_materials.size(); i++)
        {
            materials[i] = CreateOBJMaterial(scene, obj_materials[i], base_path);
        }
    }

    // Process each shape in the OBJ file.
    for (const auto& shape : shapes)
    {
        size_t index_offset = 0;
        // Group faces by material ID (a shape may mix different materials).
        HashMap<int, OBJMeshGroup> groups;

        // Accumulate every normals
        std::vector<Vec3> smooth_normals;
        if (options.gen_smooth_normal)
        {
            smooth_normals = std::vector<Vec3>(attrib.vertices.size() / 3, Vec3(0));

            size_t index_offset = 0;
            for (size_t f = 0; f < shape.mesh.num_face_vertices.size(); ++f)
            {
                size_t fv = size_t(shape.mesh.num_face_vertices[f]);
                if (fv < 3)
                {
                    index_offset += fv;
                    continue;
                }

                tinyobj::index_t i0 = shape.mesh.indices[index_offset + 0];
                tinyobj::index_t i1 = shape.mesh.indices[index_offset + 1];
                tinyobj::index_t i2 = shape.mesh.indices[index_offset + 2];

                Vec3 v0(
                    attrib.vertices[3 * size_t(i0.vertex_index) + 0], attrib.vertices[3 * size_t(i0.vertex_index) + 1],
                    attrib.vertices[3 * size_t(i0.vertex_index) + 2]
                );
                Vec3 v1(
                    attrib.vertices[3 * size_t(i1.vertex_index) + 0], attrib.vertices[3 * size_t(i1.vertex_index) + 1],
                    attrib.vertices[3 * size_t(i1.vertex_index) + 2]
                );
                Vec3 v2(
                    attrib.vertices[3 * size_t(i2.vertex_index) + 0], attrib.vertices[3 * size_t(i2.vertex_index) + 1],
                    attrib.vertices[3 * size_t(i2.vertex_index) + 2]
                );

                Vec3 e1 = v1 - v0;
                Vec3 e2 = v2 - v0;
                Vec3 face_normal = Cross(e1, e2);
                face_normal.Normalize();

                smooth_normals[i0.vertex_index] += face_normal;
                smooth_normals[i1.vertex_index] += face_normal;
                smooth_normals[i2.vertex_index] += face_normal;

                index_offset += fv;
            }

            for (auto& n : smooth_normals)
            {
                n.Normalize();
            }
        }

        // Iterate over each face.
        for (size_t f = 0; f < shape.mesh.num_face_vertices.size(); ++f)
        {
            int material_id = shape.mesh.material_ids[f]; // Material ID for this face (-1 if none)
            if (!groups.Contains(material_id))
            {
                groups.Insert(material_id, {});
            }

            OBJMeshGroup& group = groups.At(material_id);
            size_t fv = size_t(shape.mesh.num_face_vertices[f]);

            std::vector<int> face_indices;
            face_indices.reserve(fv); // Reserve space for vertex indices in the face

            // Process each vertex of the face.
            for (size_t v = 0; v < fv; v++)
            {
                const tinyobj::index_t& idx = shape.mesh.indices[index_offset + v];

                // Retrieve vertex position.
                Vec3 position;
                position.x = attrib.vertices[3 * size_t(idx.vertex_index) + 0];
                position.y = attrib.vertices[3 * size_t(idx.vertex_index) + 1];
                position.z = attrib.vertices[3 * size_t(idx.vertex_index) + 2];

                // Retrieve normal if available.
                Vec3 normal(0);
                if (idx.normal_index >= 0)
                {
                    normal.x = attrib.normals[3 * size_t(idx.normal_index) + 0];
                    normal.y = attrib.normals[3 * size_t(idx.normal_index) + 1];
                    normal.z = attrib.normals[3 * size_t(idx.normal_index) + 2];
                }
                else if (options.gen_smooth_normal)
                {
                    normal = smooth_normals[idx.vertex_index];
                }

                if (options.flip_normal)
                {
                    normal.Negate();
                }

                // Retrieve texture coordinates if available.
                Vec2 texcoord(0);
                if (idx.texcoord_index >= 0)
                {
                    texcoord.x = attrib.texcoords[2 * size_t(idx.texcoord_index) + 0];
                    texcoord.y = attrib.texcoords[2 * size_t(idx.texcoord_index) + 1];

                    if (options.flip_tex_coord)
                    {
                        texcoord.y = 1 - texcoord.y;
                    }
                }

                // Calculate tangent from the normal (since OBJ usually doesn't provide tangents).
                Vec3 tangent(0);
                if (normal != Vec3::zero)
                {
                    Vec3 bitangent;
                    CoordinateSystem(normal, &tangent, &bitangent);
                }

                group.positions.push_back(position);
                group.normals.push_back(normal);
                group.tangents.push_back(tangent);
                group.texcoords.push_back(texcoord);
                face_indices.push_back(int32(group.positions.size() - 1));
            }

            // Triangulate the face using a triangle fan approach if necessary.
            if (fv >= 3)
            {
                if (fv == 3)
                {
                    group.indices.insert(group.indices.end(), face_indices.begin(), face_indices.end());
                }
                else
                {
                    for (size_t v = 1; v < fv - 1; v++)
                    {
                        group.indices.push_back(face_indices[0]);
                        group.indices.push_back(face_indices[v]);
                        group.indices.push_back(face_indices[v + 1]);
                    }
                }
            }
            index_offset += fv;
        }

        // Create a mesh for each material group and add it to the scene.
        for (auto& [material_id, group] : groups)
        {
            Mesh* m = scene.CreateMesh(
                std::move(group.positions), std::move(group.normals), std::move(group.tangents), std::move(group.texcoords),
                std::move(group.indices), transform
            );

            const Material* material = nullptr;
            const SpectrumTexture* emission = nullptr;

            if (material_id < 0 || size_t(material_id) >= materials.size())
            {
                if (options.fallback_material)
                {
                    material = options.fallback_material;
                }
                else
                {
                    std::cerr << "No valid material found and no fallback specified" << std::endl;
                    material = CreateDiffuseMaterial(scene, Spectrum{ 1, 0, 1 });
                }
            }
            else if (options.fallback_material)
            {
                material = options.fallback_material;
            }
            else
            {
                material = materials[material_id].first;
                emission = materials[material_id].second;
            }

            std::optional<AreaLightInfo> ali = options.area_light;
            if (!ali && emission)
            {
                ali.emplace();
                ali->emission = emission;
            }

            CreateTriangles(
                scene, m, options.use_fallback_material ? options.fallback_material : material, options.fallback_medium_interface,
                ali
            );
        }
    }
}

void LoadModel(Scene& scene, std::filesystem::path filename, const Transform& transform, const ModelLoaderOptions& options)
{
    auto ext = filename.extension();
    if (ext == ".gltf" || ext == ".glb")
    {
        LoadGLTF(scene, filename, transform, options);
    }
    else if (filename.extension() == ".obj")
    {
        LoadOBJ(scene, filename, transform, options);
    }
    else
    {
        std::cerr << "Failed to load model: " << filename.string() << '\n';
        std::cerr << "Supported extensions: .obj .gltf" << std::endl;
    }
}

} // namespace bulbit

#include "Scene.h"
#include "Config.h"
#include <iostream>
#include <filesystem>

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

// Vec3 Scene::trace(const Ray &ray, int bouncesLeft, bool discardEmission) {
//     if constexpr(DEBUG) {
//         assert (ray.isNormalized());
//     }
//     if (bouncesLeft < 0) return {};

//     // TODO...
//     Intersection inter = getIntersection(ray);
//     if (!inter.happened) return {};

//     Vec3 diffuseColor = inter.getDiffuseColor();
//     return diffuseColor;
// }

// Vec3 Scene::trace(const Ray &ray, int bouncesLeft, bool discardEmission) {
//     if constexpr(DEBUG) {
//         assert (ray.isNormalized());
//     }
//     if (bouncesLeft < 0) return {};

//     // TODO...
//     Intersection inter = getIntersection(ray);
//     if (!inter.happened) return {};
//     Vec3 emission = inter.getEmission();
//     Ray nextRay(inter.pos, Random::randomHemisphereDirection(inter.getNormal()));
//     Intersection nextInter = getIntersection(nextRay);
//     if (!nextInter.happened) return emission;

//     Vec3 L_i = nextInter.getEmission();
//     Vec3 brdf = inter.calcBRDF(-nextRay.dir, -ray.dir);
//     float cosineTerm = nextRay.dir.dot(inter.getNormal());
//     return emission + 2 * PI * L_i * brdf * cosineTerm;
// }

// Vec3 Scene::trace(const Ray &ray, int bouncesLeft, bool discardEmission) {
//     if constexpr(DEBUG) {
//         assert (ray.isNormalized());
//     }
//     if (bouncesLeft < 0) return {};

//     // TODO...
//     Intersection inter = getIntersection(ray);
//     if (!inter.happened) return {};
//     Vec3 emission = inter.getEmission();
//     Ray nextRay(inter.pos, Random::randomHemisphereDirection(inter.getNormal()));
//     Intersection nextInter = getIntersection(nextRay);
//     if (!nextInter.happened) return emission;

//     Vec3 L_i = trace(nextRay, bouncesLeft - 1, false);
//     Vec3 brdf = inter.calcBRDF(-nextRay.dir, -ray.dir);
//     float cosineTerm = nextRay.dir.dot(inter.getNormal());
//     return emission + 2 * PI * L_i * brdf * cosineTerm;
// }

Vec3 Scene::trace(const Ray &ray, int bouncesLeft, bool discardEmission) {
    if constexpr(DEBUG) {
        assert (ray.isNormalized());
    }
    if (bouncesLeft < 0) return {};

    // TODO...
    Intersection inter = getIntersection(ray);
    if (!inter.happened) return {};
    Vec3 emission = inter.getEmission();
    Ray nextRay(inter.pos, Random::cosWeightedHemisphere(inter.getNormal()));
    Intersection nextInter = getIntersection(nextRay);
    Vec3 L_total;
    if (!discardEmission) L_total = emission;
    if (!nextInter.happened) return L_total;

    Vec3 L_indirect = trace(nextRay, bouncesLeft - 1, true);
    Vec3 brdf = inter.calcBRDF(-nextRay.dir, -ray.dir);
    L_indirect = PI * L_indirect * brdf;
    L_total += L_indirect;

    float pdfLightSample = 1 / lightArea;
    Intersection lightSample = sampleLight();
    Vec3 lightDir = lightSample.pos - inter.pos;
    float distanceToLight = lightDir.getLength();
    lightDir.normalize();
    Ray rayToLight(inter.pos, lightDir);
    Intersection lightInter = getIntersection(rayToLight);
    if (!lightInter.happened || lightInter.object != lightSample.object) return L_total;
    brdf = inter.calcBRDF(-lightDir, -ray.dir);
    float cosineTerm = lightDir.dot(inter.getNormal()) * lightDir.dot(-lightSample.getNormal());
    Vec3 L_direct = lightSample.getEmission() * brdf * cosineTerm / (distanceToLight * distanceToLight) / pdfLightSample;

    return L_total + L_direct;
}

tinyobj::ObjReader Scene::reader {};

void Scene::addObjects(std::string_view modelPath, std::string_view searchPath) {
    tinyobj::ObjReaderConfig config;
    config.mtl_search_path = searchPath;
    if (!reader.ParseFromFile(std::string(modelPath), config)) {
        if (!reader.Error().empty()) {
            std::cerr << "TinyObjReader: " << reader.Error();
            std::filesystem::path relative(modelPath);
            std::cerr << "Reading file " << std::filesystem::absolute(relative) << " error. File may be malformed or not exist.\n";
        }
        exit(1);
    }

    auto& attrib = reader.GetAttrib();
    auto& shapes = reader.GetShapes();
    auto& materials = reader.GetMaterials();

    // Loop over shapes
    for (size_t s = 0; s < shapes.size(); s++) {
        auto* object = new Object();
        object->name = shapes[s].name;
        // Loop over faces(polygon)
        size_t index_offset = 0;
        for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
            size_t fv = size_t(shapes[s].mesh.num_face_vertices[f]);
            std::vector<Vec3> positions;
            // Loop over vertices in the face.
            for (size_t v = 0; v < fv; v++) {
                // access to vertex
                tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
                tinyobj::real_t vx = attrib.vertices[3*size_t(idx.vertex_index)+0];
                tinyobj::real_t vy = attrib.vertices[3*size_t(idx.vertex_index)+1];
                tinyobj::real_t vz = attrib.vertices[3*size_t(idx.vertex_index)+2];

                positions.push_back({vx, vy, vz});
            } // per-vertex
            index_offset += fv;
            Mesh mesh {positions[0], positions[1], positions[2]};
            object->area += mesh.area;
            object->meshes.push_back(std::move(mesh));
        } // per-face
        object->constructBoundingBox();
        // we assume each object uses only a single material for all meshes
        auto materialId = shapes[s].mesh.material_ids[0];
        auto& material = materials[materialId];
        object->kd = Vec3 {
            material.diffuse[0],
            material.diffuse[1],
            material.diffuse[2],
        };
        if (material.emission[0] + 
            material.emission[1] + 
            material.emission[2] > 0) { // is light
            object->ke = Vec3 {
                material.emission[0], 
                material.emission[1],
                material.emission[2]
            };
            object->hasEmission = true;
            lights.push_back(object);
            lightArea += object->area;
        }
        objects.push_back(object);
    } // per-shape
}

void Scene::constructBVH() {
    assert (!objects.empty());
    bvh.root = BVH::build(objects);
}

Intersection Scene::getIntersection(const Ray &ray) {
    assert (bvh.root);
    return bvh.root->intersect(ray);
}

Intersection Scene::sampleLight() const {
    assert (lights.size() == 1 && "Currently only support a single light object");
    assert (lightArea > 0.0f);
    Intersection inter;
    return lights[0]->sample();
}

Scene::~Scene() {
    for (auto obj : objects) {
        delete obj;
    }
}

#include <cstdint>

#include "image.hpp"
#include "loader.hpp"
#include "rasterizer.hpp"

#include "../thirdparty/glm/glm.hpp"

// TODO
void Rasterizer::DrawPixel(uint32_t x, uint32_t y, Triangle trig, AntiAliasConfig config, uint32_t spp, Image& image, Color color)
{
    // if the pixel is inside the triangle
    auto InsideTriangle = [](float x, float y, Triangle trig) -> bool
    {
        std::vector<glm::vec3> cross_products(3);
        for (size_t i = 0; i < 3; ++i) 
        {
            cross_products[i] = glm::cross(glm::vec3(x + 0.5 - trig.pos[i].x, y + 0.5 - trig.pos[i].y, 0), glm::vec3(trig.pos[(i + 1) % 3].x - trig.pos[i].x, trig.pos[(i + 1) % 3].y - trig.pos[i].y, 0));
        }
        if ((cross_products[0][2] >= 0 && cross_products[1][2] >= 0 && cross_products[2][2] >= 0) || (cross_products[0][2] <= 0 && cross_products[1][2] <= 0 && cross_products[2][2] <= 0))
        {
            return true;
        }
        return false;
    };

    if (config == AntiAliasConfig::NONE)            // if anti-aliasing is off
    {
        if (InsideTriangle(double(x), double(y), trig))
        {
            image.Set(x, y, color);
        }
    }
    else if (config == AntiAliasConfig::SSAA)       // if anti-aliasing is on
    {
        uint32_t count = 0;
        uint32_t sample_dim = sqrt(spp);
        for (size_t i = 0; i < sample_dim; ++i)
        {
            for (size_t j = 0; j < sample_dim; ++j)
            {
                if (InsideTriangle(double(x) + double(i) / sample_dim, double(y) + double(j) / sample_dim, trig))
                {
                    count++;
                }
            }
        }   
        printf("count: %d\n", count);
        image.Set(x, y, ((double)count / spp) * color);
    }

    return;
}

// TODO
void Rasterizer::AddModel(MeshTransform transform, glm::mat4 rotation)
{
    /* model.push_back( model transformation constructed from translation, rotation and scale );*/
    glm::mat4x4 model_transform = glm::mat4(
        glm::vec4(transform.scale[0], 0., 0., 0.),
        glm::vec4(0., transform.scale[1], 0., 0.),
        glm::vec4(0., 0., transform.scale[2], 0.),
        glm::vec4(0., 0., 0., 1.)
    );
    model_transform *= rotation;
    model_transform[3] = glm::vec4(transform.translation, 1.);
    model.push_back(model_transform);
    return;
}

// TODO
void Rasterizer::SetView()
{
    const Camera& camera = this->loader.GetCamera();
    glm::vec3 cameraPos = camera.pos;
    glm::vec3 cameraLookAt = camera.lookAt;
    glm::vec3 cameraUp = camera.up;

    // TODO change this line to the correct view matrix
    this->view = glm::mat4(
        glm::vec4(glm::normalize(glm::cross(cameraUp, cameraPos - cameraLookAt)), 0),
        glm::vec4(glm::normalize(cameraUp), 0),
        glm::vec4(glm::normalize(cameraPos - cameraLookAt), 0),
        glm::vec4(0, 0, 0, 1)
    );
    this->view = glm::transpose(this->view);
    glm::mat4x4 view_t = glm::mat4(
        glm::vec4(1, 0, 0, 0),
        glm::vec4(0, 1, 0, 0),
        glm::vec4(0, 0, 1, 0),
        glm::vec4(-cameraPos, 1)
    );
    this->view = this->view * view_t;
    return;
}

// TODO
void Rasterizer::SetProjection()
{
    const Camera& camera = this->loader.GetCamera();

    float nearClip = camera.nearClip;                   // near clipping distance, strictly positive
    float farClip = camera.farClip;                     // far clipping distance, strictly positive
    
    float width = camera.width;
    float height = camera.height;
    
    // TODO change this line to the correct projection matrix
    glm::mat4x4 ortho_r = glm::mat4(
        glm::vec4(2 / camera.width, 0, 0, 0),
        glm::vec4(0, 2 / camera.height, 0, 0),
        glm::vec4(0, 0, -2 / (nearClip - farClip), 0),
        glm::vec4(0, 0, 0, 1)
    );

    glm::mat4x4 ortho_t = glm::mat4(
        glm::vec4(1, 0, 0, 0),
        glm::vec4(0, 1, 0, 0),
        glm::vec4(0, 0, 1,  (nearClip + farClip) / 2),
        glm::vec4(0, 0, 0, 1)
    );

    glm::mat4x4 perspective = glm::mat4(
        glm::vec4(-nearClip, 0, 0, 0),
        glm::vec4(0, -nearClip, 0, 0),
        glm::vec4(0, 0, -(nearClip + farClip), -nearClip * farClip),
        glm::vec4(0, 0, 1, 0)
    );

    this->projection = glm::transpose(ortho_r) * glm::transpose(ortho_t) * glm::transpose(perspective);
    return;
}

// TODO
void Rasterizer::SetScreenSpace()
{
    float width = this->loader.GetWidth();
    float height = this->loader.GetHeight();

    // TODO change this line to the correct screenspace matrix
    this->screenspace = glm::mat4(
        glm::vec4(width / 2, 0, 0, width / 2),
        glm::vec4(0, height / 2, 0, height / 2),
        glm::vec4(0, 0, 1, 0),
        glm::vec4(0, 0, 0, 1)
    );
    this->screenspace = glm::transpose(this->screenspace);
    return;
}

// TODO
glm::vec3 Rasterizer::BarycentricCoordinate(glm::vec2 pos, Triangle trig)
{
    glm::vec3 pos3 = glm::vec3(pos, 1);
    glm::mat3x3 A = glm::mat3x3(
        glm::vec3(trig.pos[0].x, trig.pos[1].x, trig.pos[2].x), 
        glm::vec3(trig.pos[0].y, trig.pos[1].y, trig.pos[2].y), 
        glm::vec3(1, 1, 1)
    );
    return glm::inverse(glm::transpose(A)) * pos3;
}

// TODO
float Rasterizer::zBufferDefault = 1.;

// TODO
void Rasterizer::UpdateDepthAtPixel(uint32_t x, uint32_t y, Triangle original, Triangle transformed, ImageGrey& ZBuffer)
{

    // if the pixel is inside the triangle
    auto InsideTriangle = [](float x, float y, Triangle trig) -> bool
    {
        std::vector<glm::vec3> cross_products(3);
        for (size_t i = 0; i < 3; ++i) 
        {
            cross_products[i] = glm::cross(glm::vec3(x + 0.5 - trig.pos[i].x, y + 0.5 - trig.pos[i].y, 0), glm::vec3(trig.pos[(i + 1) % 3].x - trig.pos[i].x, trig.pos[(i + 1) % 3].y - trig.pos[i].y, 0));
        }
        if ((cross_products[0][2] >= 0 && cross_products[1][2] >= 0 && cross_products[2][2] >= 0) || (cross_products[0][2] <= 0 && cross_products[1][2] <= 0 && cross_products[2][2] <= 0))
        {
            return true;
        }
        return false;
    };

    if (!InsideTriangle(double(x), double(y), transformed))
    {
        return;
    }

    glm::vec3 bary_coord = BarycentricCoordinate(glm::vec2(x, y), transformed);
    float depth = -1 / (bary_coord.x * 1 / transformed.pos[0].z + bary_coord.y * 1 / transformed.pos[1].z + bary_coord.z * 1 / transformed.pos[2].z);
    if (depth < ZBuffer.Get(x, y))
    {
        ZBuffer.Set(x, y, depth);
    }
    return;
}

// TODO
void Rasterizer::ShadeAtPixel(uint32_t x, uint32_t y, Triangle original, Triangle transformed, Image& image)
{
    glm::vec3 bary_coord = BarycentricCoordinate(glm::vec2(x, y), transformed);
    float depth = -1 / (bary_coord.x * 1 / transformed.pos[0].z + bary_coord.y * 1 / transformed.pos[1].z + bary_coord.z * 1 / transformed.pos[2].z);
    if (depth == ZBuffer.Get(x, y))
    {
        glm::vec3 pos = bary_coord.x * original.pos[0] + bary_coord.y * original.pos[1] + bary_coord.z * original.pos[2];
        glm::vec3 normal = glm::normalize(bary_coord.x * original.normal[0] + bary_coord.y * original.normal[1] + bary_coord.z * original.normal[2]);
        Color result = this->loader.GetAmbientColor();
        std::vector<Light> lights = this->loader.GetLights();
        float specular_exponent = this->loader.GetSpecularExponent();
        for (size_t i = 0; i < lights.size(); ++i)
        {
            float r = glm::distance(pos, lights[i].pos);
            glm::vec3 l = lights[i].pos - pos;
            float diffuse_coeff = lights[i].intensity * (1 / (r * r)) * glm::max(0.f, glm::dot(glm::normalize(l), normal));
            glm::vec3 v = this->loader.GetCamera().pos - pos;
            glm::vec3 h = l + v;
            float specular_coeff = lights[i].intensity * (1 / (r * r)) * pow(glm::max(0.f, glm::dot(normal, glm::normalize(h))), specular_exponent);
            result = result + diffuse_coeff * lights[i].color + specular_coeff * lights[i].color;
        }
        image.Set(x, y, result);
    }
    return;
}

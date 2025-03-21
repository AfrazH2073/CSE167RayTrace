#ifndef HYBRID_GLOSSY_MATERIAL_H
#define HYBRID_GLOSSY_MATERIAL_H

#include "GlossyMaterial.h"
#include <glm/glm.hpp>

// HybridGlossyMaterial blends mirror-like specular reflection with diffuse scattering 
// in the reflection direction. The 'roughness' parameter (in [0,1]) controls this blend:
// - roughness = 0.0 -> Perfect mirror reflection (pure specular)
// - roughness = 1.0 -> Fully perturbed reflection (more diffuse, blurry highlight)
class HybridGlossyMaterial : public GlossyMaterial {
public:
    // Roughness factor for glossy reflections.
    float roughness;

    // Constructor: takes diffuse and specular coefficients, a roughness parameter, and shininess.
    HybridGlossyMaterial(const glm::vec3 &diffuse, 
                         const glm::vec3 &specular, 
                         float roughness, 
                         float shininess)
        : GlossyMaterial(diffuse, specular, shininess), roughness(roughness) {}

    // Override sample_ray_and_update_radiance to implement the glossy reflection.
    virtual Ray sample_ray_and_update_radiance(Ray &ray, Intersection &intersection) override;
};

#endif // HYBRID_GLOSSY_MATERIAL_H

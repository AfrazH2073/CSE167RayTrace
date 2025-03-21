#define GLM_ENABLE_EXPERIMENTAL
#include "GlossyMaterial.h"

#include <glm/glm.hpp>
#include <glm/gtc/random.hpp>
#include <glm/gtx/string_cast.hpp>
#include <iostream>

using namespace glm;

Ray GlossyMaterial::sample_ray_and_update_radiance(Ray &ray, Intersection &intersection) {
    /**
     * Calculate the next ray after intersection with the model.
     * This will be used for recursive ray tracing.
     */

    // Decide if diffuse or specular reflection
    float random = linearRand(0.0f, 1.0f);
    vec3 normal = intersection.normal;
    vec3 point = intersection.point;

    // Diffuse reflection
    if (random > shininess) {
        // Step 1: Sample ray direction
        /**
         * TODO: Task 6.1
         * Implement cosine-weighted hemisphere sampling
         */
        // cosin sample next ray
        float s = linearRand(0.0f, 1.0f);
        float t = linearRand(0.0f, 1.0f);

        // TODO: Update u, v based on Equation (8) in handout
        float u = 0.0f;
        float v = 0.0f;

        vec3 hemisphere_sample = vec3(0.0f);  // TODO: Update value to cosine-weighted sampled direction

        // The direction we sampled above is in local co-ordinate frame
        // we need to align it with the surface normal
        vec3 new_dir = align_with_normal(hemisphere_sample, normal);

        // Step 2: Calculate radiance
        /**
         * TODO: Task 6.1
         * Note:
         * - C_diffuse = `this->diffuse`
         */
        vec3 W_diffuse = vec3(0.0f);  // TODO: Calculate the radiance for current bounce

        // update radiance
        ray.W_wip = ray.W_wip * W_diffuse;

        // update ray direction and position
        ray.p0 = point + 0.001f * normal;  // offset point slightly to avoid self intersection
        ray.dir = new_dir;
        ray.is_diffuse_bounce = true;
        ray.n_bounces++;

        return ray;
    }

    // Specular Reflection

    // Step 1: Calculate reflection direction
    /**
     * TODO: Task 6.2
     * Calculate the perfect mirror reflection direction
     */
    vec3 reflection_dir = vec3(0.0f);  // TODO: Update with reflection direction

    // Step 2: Calculate radiance
    /**
     * TODO: Task 6.2
     * Note:
     * - C_specular = `this->specular`
     */
    vec3 W_specular = vec3(0.0f);  // TODO: Calculate the radiance for current bounce

    // update radiance
    ray.W_wip = ray.W_wip * W_specular;
    ray.p0 = point + 0.001f * normal;  // offset point slightly to avoid self intersection
    ray.dir = reflection_dir;
    ray.is_diffuse_bounce = false;
    ray.n_bounces++;

    return ray;
}

glm::vec3 GlossyMaterial::get_direct_lighting(Intersection &intersection, Scene const &scene) {
    using namespace glm;

    // Accumulate contributions from all lights
    vec3 cummulative_direct_light = vec3(0.0f);

    // Surface point and normal
    const vec3 P = intersection.point;
    const vec3 N = intersection.normal;

    // Loop over each light source
    for (unsigned int idx = 0; idx < scene.light_sources.size(); idx++) {
        // Skip if this intersection is with the light source itself
        if (scene.light_sources[idx] == intersection.model)
            continue;

        // 1) Get a point on the light source
        vec3 light_pos = scene.light_sources[idx]->get_surface_point();

        // 2) Construct the shadow ray
        // Offset the origin slightly to avoid self-shadowing
        Ray shadow_ray;
        shadow_ray.p0 = P + 1e-4f * N;
        shadow_ray.dir = normalize(light_pos - P);

        // 3) Intersect the shadow ray with all models
        for (auto &model : scene.models) {
            model->intersect(shadow_ray);
        }

        // Find the closest intersection along this shadow ray
        Intersection closest_intersection;
        closest_intersection.t = std::numeric_limits<float>::max();

        for (auto &isect : shadow_ray.intersections) {
            if (isect.t < closest_intersection.t) {
                closest_intersection = isect;
            }
        }

        // 4) Check if the light is visible
        if (closest_intersection.model == scene.light_sources[idx]) {
            // Light is visible, so compute diffuse contribution using Lambert’s law
            vec3 light_emission = scene.light_sources[idx]->material->emission;

            // Direction from surface point to light
            vec3 L = shadow_ray.dir;
            // Lambert’s cosine factor
            float nDotL = std::max(0.0f, dot(N, L));

            // Example diffuse term (assuming purely diffuse response):
            // If your material has a diffuse color (e.g., this->diffuse_color),
            // multiply that in as well: diffuse_color * light_emission * nDotL
            vec3 direct_light = light_emission * nDotL;

            // Light attenuation based on distance
            float attenuation_factor =
                scene.light_sources[idx]->material->get_light_attenuation_factor(closest_intersection.t);

            // Accumulate contribution
            cummulative_direct_light += direct_light / attenuation_factor;
        }
    }

    return cummulative_direct_light;
}



vec3 GlossyMaterial::color_of_last_bounce(Ray &ray, Intersection &intersection, Scene const &scene) {
    using namespace glm;
    /**
     * Color after last bounce will be `W_wip * last_bounce_color`
     * We shade last bounce for this Glossy material using direct diffuse lighting
     */

    vec3 direct_diff_light = this->get_direct_lighting(intersection, scene);

    return ray.W_wip * diffuse * (1 - shininess) * direct_diff_light;
}
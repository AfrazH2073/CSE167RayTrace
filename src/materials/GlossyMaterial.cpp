#define GLM_ENABLE_EXPERIMENTAL
#include "GlossyMaterial.h"

#include <glm/glm.hpp>
#include <glm/gtc/random.hpp>
#include <glm/gtx/string_cast.hpp>
#include <iostream>
#include <algorithm>

using namespace glm;

Ray GlossyMaterial::sample_ray_and_update_radiance(Ray &ray, Intersection &intersection) {
    // Decide if the bounce is diffuse or specular based on a random value versus shininess.
    float random = linearRand(0.0f, 1.0f);
    vec3 normal = intersection.normal;
    vec3 point = intersection.point;

    // --- Diffuse Reflection (Task 6.1) ---
    if (random > shininess) {
        // Step 1: Cosine-weighted hemisphere sampling.
        // Generate two random numbers in [0,1]
        float s = linearRand(0.0f, 1.0f);
        float t = linearRand(0.0f, 1.0f);
        // Compute angle u from 0 to 2π
        float u_angle = 2.0f * pi<float>() * s;
        // For cosine-weighted sampling, set:
        // x = sqrt(1 - t) * cos(u_angle), y = sqrt(t), z = sqrt(1 - t) * sin(u_angle)
        // Here the 'y' component represents the component along the surface normal.
        float r = sqrt(1.0f - t);
        float y_comp = sqrt(t);
        vec3 hemisphere_sample = vec3(r * cos(u_angle), y_comp, r * sin(u_angle));

        // The sample above is in the local coordinate frame (with y-up).
        // Align it with the actual surface normal.
        vec3 new_dir = align_with_normal(hemisphere_sample, normal);

        // Step 2: Compute the diffuse radiance for this bounce.
        // Multiply the material's diffuse coefficient (C_diffuse) by the cosine of the angle between normal and new_dir.
        float cos_theta = std::max(dot(normal, new_dir), 0.0f);
        vec3 W_diffuse = diffuse * cos_theta;

        // Update the ray's radiance.
        ray.W_wip = ray.W_wip * W_diffuse;

        // Update the ray's origin and direction.
        // Offset the origin a bit along the normal to avoid self-intersection.
        ray.p0 = point + 0.001f * normal;
        ray.dir = new_dir;
        ray.is_diffuse_bounce = true;
        ray.n_bounces++;

        return ray;
    }

    // --- Specular Reflection (Task 6.2) ---
    // Step 1: Compute the perfect mirror reflection direction.
    // Reflection direction: r = ray.dir - 2 * dot(ray.dir, normal) * normal.
    vec3 reflection_dir = normalize(ray.dir - 2.0f * dot(ray.dir, normal) * normal);

    // Step 2: Compute the specular radiance.
    // Here, the specular coefficient (C_specular) is directly used.
    vec3 W_specular = specular;

    // Update the ray's radiance.
    ray.W_wip = ray.W_wip * W_specular;

    // Update the ray's origin and direction.
    ray.p0 = point + 0.001f * normal;  // offset to avoid self-intersection
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
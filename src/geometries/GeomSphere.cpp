#include "GeomSphere.h"

#include <iostream>
#include <utility>
#include <glm/glm.hpp>
#include <cmath>

#include "Intersection.h"
#include "Ray.h"

std::vector<Intersection> GeomSphere::intersect(Ray &ray) {
    // Vector to store the intersections
    std::vector<Intersection> intersections;

    // Compute the vector from the ray origin to the sphere center.
    // Assumes that 'center' and 'radius' are members of GeomSphere.
    glm::vec3 oc = ray.p0 - center;

    // Coefficients for the quadratic equation:
    // a*t^2 + b*t + c = 0, where t is the distance along the ray.
    float a = glm::dot(ray.dir, ray.dir);
    float b = 2.0f * glm::dot(oc, ray.dir);
    float c = glm::dot(oc, oc) - (radius * radius);

    // Compute the discriminant.
    float discriminant = b * b - 4.0f * a * c;

    // If discriminant is negative, there is no real intersection.
    if (discriminant < 0.0f)
        return intersections;

    // Compute the square root of the discriminant.
    float sqrt_disc = std::sqrt(discriminant);

    // Compute the two possible t values.
    float t0 = (-b - sqrt_disc) / (2.0f * a);
    float t1 = (-b + sqrt_disc) / (2.0f * a);

    // Only add intersections where t > 0.
    if (t0 > 0.0f) {
        glm::vec3 point = ray.p0 + t0 * ray.dir;
        glm::vec3 normal = glm::normalize(point - center);
        intersections.push_back({ t0, point, normal, this, nullptr });
    }
    if (t1 > 0.0f && t1 != t0) {
        glm::vec3 point = ray.p0 + t1 * ray.dir;
        glm::vec3 normal = glm::normalize(point - center);
        intersections.push_back({ t1, point, normal, this, nullptr });
    }

    return intersections;
}

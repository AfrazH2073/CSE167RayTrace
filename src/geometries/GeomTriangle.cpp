#define GLM_FORCE_RADIANS
#define GLM_ENABLE_EXPERIMENTAL
#include "GeomTriangle.h"

#include <glm/glm.hpp>
#include <glm/gtx/string_cast.hpp>
#include <cmath>
#include <iostream>

#include "Intersection.h"
#include "Ray.h"

GeomTriangle::GeomTriangle(std::vector<glm::vec3> &vertices, std::vector<glm::vec3> &normals) {
    this->vertices[0] = vertices[0];
    this->vertices[1] = vertices[1];
    this->vertices[2] = vertices[2];

    this->normals[0] = normals[0];
    this->normals[1] = normals[1];
    this->normals[2] = normals[2];
}

std::vector<Intersection> GeomTriangle::intersect(Ray &ray) {
    using namespace glm;
    std::vector<Intersection> intersections;
    const float EPSILON = 1e-6f;

    // Retrieve the triangle vertices
    vec3 v0 = vertices[0];
    vec3 v1 = vertices[1];
    vec3 v2 = vertices[2];

    // Compute the two edges sharing v0
    vec3 edge1 = v1 - v0;
    vec3 edge2 = v2 - v0;

    // Begin calculating determinant - also used to calculate u parameter.
    vec3 h = cross(ray.dir, edge2);
    float a = dot(edge1, h);

    // If a is near zero, the ray is parallel to the triangle.
    if (fabs(a) < EPSILON)
        return intersections;

    float f = 1.0f / a;
    vec3 s = ray.p0 - v0;
    float u = f * dot(s, h);
    // Check if the intersection is outside the triangle.
    if (u < 0.0f || u > 1.0f)
        return intersections;

    vec3 q = cross(s, edge1);
    float v = f * dot(ray.dir, q);
    // Check if the intersection is outside the triangle.
    if (v < 0.0f || (u + v) > 1.0f)
        return intersections;

    // Compute the distance along the ray to the intersection point.
    float t = f * dot(edge2, q);
    if (t < EPSILON) // Ensure the intersection is in front of the camera.
        return intersections;

    // Compute the intersection point.
    vec3 point = ray.p0 + t * ray.dir;

    // Interpolate the normal using barycentric coordinates.
    // The barycentrics are: alpha = 1 - u - v, beta = u, gamma = v.
    vec3 interpolatedNormal = normalize(normals[0] * (1.0f - u - v) +
                                          normals[1] * u +
                                          normals[2] * v);

    intersections.push_back({t, point, interpolatedNormal, this, nullptr});
    return intersections;
}
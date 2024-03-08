//
// Created by LEI XU on 5/13/19.
//

#ifndef RAYTRACING_SPHERE_H
#define RAYTRACING_SPHERE_H

#include "Object.hpp"
#include "Vector.hpp"
#include "Bounds3.hpp"
#include "Material.hpp"
#include "global.hpp"

class Sphere : public Object{
public:
    Vector3f center;
    double radius, radius2;
    Material *m;
    double area;
    Sphere(const Vector3f & c, const double & r, Material* mt = new Material()) : center(c), radius(r), radius2(r * r), m(mt), area(4 * M_PI * r * r) {}
    bool intersect(const Ray& ray) override {
        // analytic solution
        Vector3f L = ray.origin - center;
        double a = dotProduct(ray.direction, ray.direction);
        double b = 2 * dotProduct(ray.direction, L);
        double c = dotProduct(L, L) - radius2;
        double t0, t1;
        double area = 4 * M_PI * radius2;
        if (!solveQuadratic(a, b, c, t0, t1)) return false;
        if (t0 < 0) t0 = t1;
        if (t0 < 0) return false;
        return true;
    }
    bool intersect(const Ray& ray, double &tnear, uint32_t &index) const override
    {
        // analytic solution
        Vector3f L = ray.origin - center;
        double a = dotProduct(ray.direction, ray.direction);
        double b = 2 * dotProduct(ray.direction, L);
        double c = dotProduct(L, L) - radius2;
        double t0, t1;
        if (!
        solveQuadratic(a, b, c, t0, t1)) return false;
        if (t0 < 0) t0 = t1;
        if (t0 < 0) return false;
        tnear = t0;

        return true;
    }
    Intersection getIntersection(Ray ray) override {
        Intersection result;
        result.happened = false;
        Vector3f L = ray.origin - center;
        double a = dotProduct(ray.direction, ray.direction);
        double b = 2 * dotProduct(ray.direction, L);
        double c = dotProduct(L, L) - radius2;
        double t0, t1;
        if (!solveQuadratic(a, b, c, t0, t1)) return result;
        if (t0 < EPSILON) t0 = t1;
        if (t0 < EPSILON) return result;
        result.happened=true;

        result.coords = Vector3f(ray.origin + ray.direction * t0);
        result.normal = normalize(Vector3f(result.coords - center));
        result.m = this->m;
        result.obj = this;
        result.distance = t0;
        return result;

    }
    void getSurfaceProperties(const Vector3f &P, const Vector3f &I, const uint32_t &index, const Vector2f &uv, Vector3f &N, Vector2f &st) const override
    { N = normalize(P - center); }

    
    Vector3f evalDiffuseColor(const Vector2f &st)const override {
        //return m->getColor();
        return {0};
    }
    Bounds3 getBounds() override {
        return Bounds3(Vector3f(center.x-radius, center.y-radius, center.z-radius),
                       Vector3f(center.x+radius, center.y+radius, center.z+radius));
    }
    void Sample(Intersection &pos, double &pdf) override {
        double theta = 2.0 * M_PI * get_random_double(), phi = M_PI * get_random_double();
        Vector3f dir(std::cos(phi), std::sin(phi)*std::cos(theta), std::sin(phi)*std::sin(theta));
        pos.coords = center + radius * dir;
        pos.normal = dir;
        pos.emit = m->getEmission();
        pdf = 1.0f / area;
    }
    double getArea() override {
        return area;
    }
    bool hasEmit() override {
        return m->hasEmission();
    }
};




#endif //RAYTRACING_SPHERE_H

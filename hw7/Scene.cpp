//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"
#include "Vector.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, double &pdf) const
{
    double emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    double p = get_random_double() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

/*
bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        double &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        double tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}*/

// Intersection Scene::intersect(const Ray& ray) const;
// void sampleLight(Intersection& pos, double& pdf); light is MeshTriangle
// Vector3f Material::sample(const Vector3f &wi, const Vector3f &N); uniformly sampling
// double Material::pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N);      pdf = 1 / (2 * pi)
// Vector3f Material::eval(const Vector3f &wi, const Vector3f &wo, const Vector3f &N);  brdf = 1 / pi
// Scene::RussianRoulette = 0.8
//
// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray) const
{
    // TO DO Implement Path Tracing Algorithm here
    Intersection inter_first = intersect(ray);

    if (!inter_first.happened) {
        return {0};
    }

    Intersection inter_light;
    double pdf_light; 
    sampleLight(inter_light, pdf_light); 


    Vector3f emit_light = inter_light.m->getEmission(); // radiance of light
    Vector3f normal_light = inter_light.normal;
    Vector3f normal_hit   = inter_first.normal;

    Vector3f pos_light = inter_light.coords + EPSILON * normal_light;
    Vector3f pos_hit   = inter_first.coords + EPSILON * normal_hit;
    Vector3f shadow_dir = pos_light - pos_hit;

    Vector3f wo = -ray.direction;
    Vector3f wi = normalize(shadow_dir);

    Vector3f L_emit;
    Vector3f L_dir;
    Vector3f L_indir;
    if (inter_first.m->hasEmission()) {
        // camera ray hit a light source
        L_emit = inter_first.m->getEmission();
        L_dir = {0};
    } else {
        // camera ray hit a non-light object
        L_emit = {0};
        // test shadow ray
        Ray shadow_ray = Ray(pos_hit, wi); 
        Intersection inter_shadow = intersect(shadow_ray);
        if (inter_shadow.distance < shadow_dir.norm() - EPSILON) {
            L_dir = {0}; 
        } else { 
            L_dir = (pdf_light < 0.0001 * EPSILON) ? Vector3f(0) 
                    : emit_light * inter_first.m->eval(wi, wo, normal_hit) * dotProduct(wi, normal_hit) 
                    * dotProduct(normal_light, -wi) / dotProduct(shadow_dir, shadow_dir) / pdf_light; 
        }
    }
    // test RussianRoulette
    if (get_random_double() > RussianRoulette) {
        return L_emit + L_dir;
    }
    Vector3f reflected_dir = normalize(inter_first.m->sample(wo,normal_hit));
    Ray reflected_ray = Ray(pos_hit, reflected_dir);
    double pdf_hemi = inter_first.m->pdf(wo, reflected_dir, normal_hit);
    Intersection inter_reflect = intersect(reflected_ray);
    if (!inter_reflect.happened || inter_reflect.m->hasEmission()) {
        // 
        L_indir = {0};
    } else {
        L_indir = (pdf_hemi < 0.0001 * EPSILON) ? Vector3f(0) 
                  : inter_first.m->eval(reflected_dir, wo, normal_hit) * castRay(reflected_ray)
                  * dotProduct(normal_hit, reflected_dir) / pdf_hemi / RussianRoulette;
    }
    return L_emit + L_dir + L_indir;
}

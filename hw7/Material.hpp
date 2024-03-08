//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_MATERIAL_H
#define RAYTRACING_MATERIAL_H

#include "Vector.hpp"
#include "global.hpp"
#include <cmath>

enum MaterialType { DIFFUSE, MICROFACET };

class Material{
private:

    // Compute reflection direction
    Vector3f reflect(const Vector3f &I, const Vector3f &N) const
    {
        return I - 2 * dotProduct(I, N) * N;
    }

    // Compute refraction direction using Snell's law
    //
    // We need to handle with care the two possible situations:
    //
    //    - When the ray is inside the object
    //
    //    - When the ray is outside.
    //
    // If the ray is outside, you need to make cosi positive cosi = -N.I
    //
    // If the ray is inside, you need to invert the refractive indices and negate the normal N
    Vector3f refract(const Vector3f &I, const Vector3f &N, const double &ior) const
    {
        double cosi = clamp(-1, 1, dotProduct(I, N));
        double etai = 1, etat = ior;
        Vector3f n = N;
        if (cosi < 0) { cosi = -cosi; } else { std::swap(etai, etat); n= -N; }
        double eta = etai / etat;
        double k = 1 - eta * eta * (1 - cosi * cosi);
        return k < 0 ? 0 : eta * I + (eta * cosi - sqrtf(k)) * n;
    }

    // Compute Fresnel equation
    //
    // \param I is the incident view direction
    //
    // \param N is the normal at the intersection point
    //
    // \param ior is the material refractive index
    //
    // \param[out] kr is the amount of light reflected
    
    void fresnel(const Vector3f &I, const Vector3f &N, const double &ior, double &kr) const
    {
        double cosi = clamp(-1, 1, dotProduct(I, N));
        double etai = 1, etat = ior;
        if (cosi > 0) {  std::swap(etai, etat); }
        // Compute sini using Snell's law
        double sint = etai / etat * sqrtf(std::max(0.0, 1 - cosi * cosi));
        // Total internal reflection
        if (sint >= 1) {
            kr = 1;
        }
        else {
            double cost = sqrtf(std::max(0.0, 1 - sint * sint));
            cosi = fabs(cosi);
            double Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
            double Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
            kr = (Rs * Rs + Rp * Rp) / 2;
        }
        // As a consequence of the conservation of energy, transmittance is given by:
        // kt = 1 - kr;
    }

    Vector3f toWorld(const Vector3f &a, const Vector3f &N){
        Vector3f B, C;
        if (std::fabs(N.x) > std::fabs(N.y)){
            double invLen = 1.0f / std::sqrt(N.x * N.x + N.z * N.z);
            C = Vector3f(N.z * invLen, 0.0, -N.x *invLen);
        }
        else {
            double invLen = 1.0 / std::sqrt(N.y * N.y + N.z * N.z);
            C = Vector3f(0.0, N.z * invLen, -N.y *invLen);
        }
        B = crossProduct(C, N);
        return a.x * B + a.y * C + a.z * N;
    }

public:
    MaterialType m_type;
    //Vector3f m_color;
    Vector3f m_emission;
    double ior;
    Vector3f Kd, Ks;
    double specularExponent;
    //Texture tex;
    double roughness;

    inline Material(MaterialType t=DIFFUSE, Vector3f e=Vector3f(0,0,0));
    inline MaterialType getType();
    //inline Vector3f getColor();
    inline Vector3f getColorAt(double u, double v);
    inline Vector3f getEmission();
    inline bool hasEmission();

    // sample a ray by Material properties
    inline Vector3f sample(const Vector3f &wi, const Vector3f &N);
    // given a ray, calculate the PdF of this ray
    inline double pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N);
    // given a ray, calculate the contribution of this ray
    inline Vector3f eval(const Vector3f &wi, const Vector3f &wo, const Vector3f &N);

};

Material::Material(MaterialType t, Vector3f e){
    m_type = t;
    //m_color = c;
    m_emission = e;
}

MaterialType Material::getType(){return m_type;}
///Vector3f Material::getColor(){return m_color;}
Vector3f Material::getEmission() {return m_emission;}
bool Material::hasEmission() {
    if (m_emission.norm() > EPSILON) return true;
    else return false;
}

Vector3f Material::getColorAt(double u, double v) {
    return Vector3f();
}


Vector3f Material::sample(const Vector3f &wi, const Vector3f &N){
    switch(m_type){
        case DIFFUSE:
        {
            // uniform sample on the hemisphere
            double x_1 = get_random_double(), x_2 = get_random_double();
            double z = std::fabs(1.0 - 2.0 * x_1);
            double r = std::sqrt(1.0 - z * z), phi = 2 * M_PI * x_2;
            Vector3f localRay(r*std::cos(phi), r*std::sin(phi), z);
            return toWorld(localRay, N);
            
            break;
        }
        case MICROFACET:
        {
            // uniform sample on the hemisphere
            double x_1 = get_random_double(), x_2 = get_random_double();
            double z = std::fabs(1.0 - 2.0 * x_1);
            double r = std::sqrt(1.0 - z * z), phi = 2 * M_PI * x_2;
            Vector3f localRay(r*std::cos(phi), r*std::sin(phi), z);
            return toWorld(localRay, N);
            
            break;
        }
        default:
        {
            return {0};
            break;
        }
    }
}

double Material::pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N){
    switch(m_type){
        case DIFFUSE:
        {
            // uniform sample probability 1 / (2 * PI)
            if (dotProduct(wo, N) > -EPSILON)
                return 0.5 / M_PI;
            else
                return 0.0;
            break;
        }
        case MICROFACET:
        {
            if (dotProduct(wo, N) > -EPSILON)
                return 0.5 / M_PI;
            else
                return 0.0;
            break;
        }
        default:
        {
            return 0;
            break;
        }
    }
}

Vector3f Material::eval(const Vector3f &wi, const Vector3f &wo, const Vector3f &N){
    switch(m_type){
        case DIFFUSE:
        {
            // calculate the contribution of diffuse model
            double cosalpha = dotProduct(N, wo);
            if (cosalpha > -EPSILON) {
                // on hemisphere, albedo = Kd and brdf = 1 / pi;
                Vector3f diffuse = Kd / M_PI;
                return diffuse;
            }
            else
                return Vector3f(0.0);
            break;
        }
        case MICROFACET: {
           double cosalpha = dotProduct(N, wo); 
            if (cosalpha > -EPSILON) {
                double F, G, D;
                // F term 
                fresnel(wi, N, ior, F);
                double alpha_squared = roughness * roughness;
                // GGX model
                auto D_term = [&](const Vector3f & h, const Vector3f & N) {
                    double cos_theta_squared = pow(dotProduct(h, N), 2);
                    double denominator = M_PI * pow((1 + cos_theta_squared * (alpha_squared - 1)), 2);
                    if (denominator < EPSILON) {
                        return 1.0;
                    }
                    return alpha_squared / denominator;
                };
                auto G_term = [&](const Vector3f & wi, const Vector3f & wo, const Vector3f & N) {
                    double delta_wi, delta_wo;
                    double tan_theta_wi_sq = 1.0 / pow(dotProduct(wi, N), 2) - 1;
                    double tan_theta_wo_sq = 1.0 / pow(dotProduct(wo, N), 2) - 1;
                    delta_wi = (-1 + sqrt(1 + alpha_squared * tan_theta_wi_sq)) / 2;
                    delta_wo = (-1 + sqrt(1 + alpha_squared * tan_theta_wo_sq)) / 2;
                    double denom = (1 + delta_wi + delta_wo);
                    return 1.0 / denom;
                };
                D = D_term(normalize(wi + wo), N);
                G = G_term(wi, wo, N);
                Vector3f diffuse = (1 - F) * Kd / M_PI;
                Vector3f specular;
                double denom = 4 * dotProduct(N, wi) * dotProduct(N, wo);
                if (denom < EPSILON) {
                    specular = {0};
                } else {
                    specular = F * G * D / denom;
                }
                return diffuse + specular;
            } else {
                return {0};        
            }
        } break;
        default: { return 0; } break;
    }
}

#endif //RAYTRACING_MATERIAL_H

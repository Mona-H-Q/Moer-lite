#include "Cone.h"
#include "ResourceLayer/Factory.h"

bool Cone::rayIntersectShape(Ray &ray, int *primID, float *u, float *v) const {
    //* todo 完成光线与圆柱的相交 填充primId,u,v.如果相交，更新光线的tFar
    //* 1.光线变换到局部空间
    //* 2.联立方程求解
    //* 3.检验交点是否在圆锥范围内
    //* 4.更新ray的tFar,减少光线和其他物体的相交计算次数
    //* Write your code here.
    Ray invRay = transform.inverseRay(ray);

    float tantheta = radius / height, cos2theta = 1.f / (tantheta * tantheta + 1.f);
    Vector3f V(0.f, 0.f, -1.f);
    Vector3f CO = invRay.origin - Point3f(0.f, 0.f, height);
    float D_V = dot(invRay.direction, V);
    float CO_V = dot(CO, V);
    float D_CO = dot(invRay.direction, CO);
    float CO_CO = dot(CO, CO);

    float A = (D_V * D_V - cos2theta);
    float B = 2.f * (D_V * CO_V - D_CO * cos2theta);
    float C = CO_V * CO_V - CO_CO * cos2theta;
    float t0, t1;
    if(!Quadratic(A, B, C, &t0, &t1)) return false;

    auto check = [](Ray invRay, int *primID, float *u, float *v, float t, const Cone *C){
        if(t < invRay.tNear || t > invRay.tFar) return false;
        
        Point3f ins = invRay.at(t);
        if(ins[2] < 0 || ins[2] > C->height) return false;

        float phi = atan2(ins[1], ins[0]);
        if(phi < 0) phi += 2.f * PI;
        if(phi > C->phiMax) return false;

        *primID = 0;
        *u = phi / C->phiMax;
        *v = ins[2] / C->height;

        return true;
    };

    if(check(invRay, primID, u, v, t0, this)){
        ray.tFar = t0;
        return true;
    } else if(check(invRay, primID, u, v, t1, this)){
        ray.tFar = t1;
        return true;
    }
    return false;
}

void Cone::fillIntersection(float distance, int primID, float u, float v, Intersection *intersection) const {
    /// ----------------------------------------------------
    //* todo 填充圆锥相交信息中的法线以及相交位置信息
    //* 1.法线可以先计算出局部空间的法线，然后变换到世界空间
    //* 2.位置信息可以根据uv计算出，同样需要变换
    //* Write your code here.
    /// ----------------------------------------------------

    float phi = u * phiMax;
    float hei = v * height;
    float rad = (1 - v) * radius;
    float dirx = cos(phi), diry = sin(phi);
    Point3f pos(dirx * rad, diry * rad, hei);
    intersection->position = transform.toWorld(pos);
    Point3f tmp(0.f, 0.f, hei - rad * rad / (height - hei));
    Vector3f nor = pos - tmp;
    intersection->normal = normalize(transform.toWorld(nor));

    intersection->shape = this;
    intersection->distance = distance;
    intersection->texCoord = Vector2f{u, v};
    Vector3f tangent{1.f, 0.f, .0f};
    Vector3f bitangent;
    if (std::abs(dot(tangent, intersection->normal)) > .9f) {
        tangent = Vector3f(.0f, 1.f, .0f);
    }
    bitangent = normalize(cross(tangent, intersection->normal));
    tangent = normalize(cross(intersection->normal, bitangent));
    intersection->tangent = tangent;
    intersection->bitangent = bitangent;
}

void Cone::uniformSampleOnSurface(Vector2f sample, Intersection *result, float *pdf) const {

}

Cone::Cone(const Json &json) : Shape(json) {
    radius = fetchOptional(json, "radius", 1.f);
    height = fetchOptional(json, "height", 1.f);
    phiMax = fetchOptional(json, "phi_max", 2 * PI);
    float tanTheta = radius / height;
    cosTheta = sqrt(1/(1+tanTheta * tanTheta));
    //theta = fetchOptional(json,)
    AABB localAABB = AABB(Point3f(-radius,-radius,0),Point3f(radius,radius,height));
    boundingBox = transform.toWorld(localAABB);
    boundingBox = AABB(Point3f(-100,-100,-100),Point3f(100,100,100));
}

REGISTER_CLASS(Cone, "cone")

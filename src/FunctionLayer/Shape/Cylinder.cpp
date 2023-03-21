#include "Cylinder.h"
#include "ResourceLayer/Factory.h"


bool Cylinder::rayIntersectShape(Ray &ray, int *primID, float *u, float *v) const {
    //* todo 完成光线与圆柱的相交 填充primId,u,v.如果相交，更新光线的tFar
    //* 1.光线变换到局部空间
    //* 2.联立方程求解
    //* 3.检验交点是否在圆柱范围内
    //* 4.更新ray的tFar,减少光线和其他物体的相交计算次数
    //* Write your code here.
    Ray invRay = transform.inverseRay(ray);

    float A = invRay.direction[0] * invRay.direction[0] + invRay.direction[1] * invRay.direction[1];
    float B = 2.f * (invRay.origin[0] * invRay.direction[0] + invRay.origin[1] * invRay.direction[1]);
    float C = invRay.origin[0] * invRay.origin[0] + invRay.origin[1] * invRay.origin[1] - radius * radius;
    float t0, t1;
    if(!Quadratic(A, B, C, &t0, &t1)) return false;

    auto check = [](Ray invRay, int *primID, float *u, float *v, float t, const Cylinder *C){
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

void Cylinder::fillIntersection(float distance, int primID, float u, float v, Intersection *intersection) const {
    /// ----------------------------------------------------
    //* todo 填充圆柱相交信息中的法线以及相交位置信息
    //* 1.法线可以先计算出局部空间的法线，然后变换到世界空间
    //* 2.位置信息可以根据uv计算出，同样需要变换
    //* Write your code here.
    /// ----------------------------------------------------

    float phi = u * phiMax;
    float hei = v * height;
    float dirx = cos(phi), diry = sin(phi);
    Vector3f nor = Vector3f(dirx, diry, 0.f);
    intersection->normal = normalize(transform.toWorld(nor));
    Point3f pos(dirx * radius, diry * radius, hei);
    intersection->position = transform.toWorld(pos);

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

void Cylinder::uniformSampleOnSurface(Vector2f sample, Intersection *result, float *pdf) const {

}

Cylinder::Cylinder(const Json &json) : Shape(json) {
    radius = fetchOptional(json,"radius",1.f);
    height = fetchOptional(json,"height",1.f);
    phiMax = fetchOptional(json,"phi_max",2 * PI);
    AABB localAABB = AABB(Point3f(-radius,-radius,0),Point3f(radius,radius,height));
    boundingBox = transform.toWorld(localAABB);
}

REGISTER_CLASS(Cylinder,"cylinder")

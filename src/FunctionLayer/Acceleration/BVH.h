#pragma once
#include "Acceleration.h"
class BVH : public Acceleration{
public:
    BVH() = default;
    void build() override;
    bool rayIntersect(Ray &ray, int *geomID, int *primID, float *u, float *v) const override;
protected:
    struct BVHNode;
    static constexpr int bvhLeafMaxSize = 64;
    std::shared_ptr<BVH::BVHNode> 
    recursiveBuild(const AABB &aabb, std::vector<int> &primIdxBuffer);
    std::shared_ptr<BVH::BVHNode> root;
    void Query(std::shared_ptr<BVHNode> now, Ray &ray, 
              int *geomID, int *primID, float *u, float *v) const;
};
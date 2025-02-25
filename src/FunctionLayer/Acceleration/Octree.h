#pragma once
#include "Acceleration.h"
class Octree : public Acceleration {
public:
  Octree() = default;
  void build() override;
  bool rayIntersect(Ray &ray, int *geomID, int *primID, float *u,
                            float *v) const override;
protected:
  struct OctreeNode;
    static constexpr int ocLeafMaxSize = 64;
    std::shared_ptr<Octree::OctreeNode>
  recursiveBuild(const AABB &aabb, const std::vector<int> &primIdxBuffer);
  std::shared_ptr<Octree::OctreeNode> root;
  void Query(std::shared_ptr<OctreeNode> now, Ray &ray, 
              int *geomID, int *primID, float *u, float *v) const;
};

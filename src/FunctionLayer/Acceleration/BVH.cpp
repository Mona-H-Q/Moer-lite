#include "BVH.h"
struct  BVH::BVHNode{
    //* todo BVH节点结构设计
    AABB boundingBox;
    std::shared_ptr<BVHNode> left, right;
    int primCount = -1;
    int primIdxBuffer[bvhLeafMaxSize];
    int splitAxis;
};

std::shared_ptr<BVH::BVHNode> BVH::recursiveBuild(const AABB &aabb,
                       std::vector<int> &primIdxBuffer){
    std::shared_ptr<BVHNode> now = std::make_shared<BVHNode>();
    now->boundingBox = aabb;
    now->left = now->right = NULL;
    now->primCount = 0;
    now->splitAxis = 0;

    int Mx = 0;
    for(int i = 0; i < 3; ++i)
        if(aabb.pMax[i] - aabb.pMin[i] > Mx)
            Mx = aabb.pMax[i] - aabb.pMin[i], now->splitAxis = i;
    std::sort(primIdxBuffer.begin(), primIdxBuffer.end(), [=](int A, int B){
        Point3f pA = shapes[A]->getAABB().Center(), pB = shapes[B]->getAABB().Center();
        return pA[now->splitAxis] < pB[now->splitAxis];
    });

    // leaf
    if(primIdxBuffer.size() <= bvhLeafMaxSize){
        for(int index : primIdxBuffer)
            now->primIdxBuffer[now->primCount++] = index;
    } else{
        std::vector<int> leftBuffer, rightBuffer;
        AABB laabb, raabb;
        int sz = primIdxBuffer.size();
        for(int i = 0; i < sz / 2; ++i)
            leftBuffer.push_back(primIdxBuffer[i]),
            laabb.Expand(shapes[primIdxBuffer[i]]->getAABB());
        if(!leftBuffer.empty()) now->left = recursiveBuild(laabb, leftBuffer);
        for(int i = sz / 2; i < sz; ++i)
            rightBuffer.push_back(primIdxBuffer[i]),
            raabb.Expand(shapes[primIdxBuffer[i]]->getAABB());
        if(!rightBuffer.empty()) now->right = recursiveBuild(raabb, rightBuffer);
    }
    return now;
}
void BVH::build() {
    for (const auto & shape : shapes) {
        //* 自行实现的加速结构请务必对每个shape调用该方法，以保证TriangleMesh构建内部加速结构
        //* 由于使用embree时，TriangleMesh::getAABB不会被调用，因此出于性能考虑我们不在TriangleMesh
        //* 的构造阶段计算其AABB，因此当我们将TriangleMesh的AABB计算放在TriangleMesh::initInternalAcceleration中
        //* 所以请确保在调用TriangleMesh::getAABB之前先调用TriangleMesh::initInternalAcceleration
        shape->initInternalAcceleration();
        boundingBox.Expand(shape->getAABB());
    }
    //* todo 完成BVH构建
    std::vector<int> primIdxBuffer(shapes.size());
    std::iota(primIdxBuffer.begin(), primIdxBuffer.end(), 0);
    root = recursiveBuild(boundingBox, primIdxBuffer);
}

void BVH::Query(std::shared_ptr<BVHNode> now, Ray &ray, 
            int *geomID, int *primID, float *u, float *v) const{
    if(!now->boundingBox.RayIntersect(ray)) return ;
    if(now->primCount == 0){
        if(ray.direction[now->splitAxis] > 0){
            if(now->left != NULL) Query(now->left, ray, geomID, primID, u, v);
            if(now->right != NULL) Query(now->right, ray, geomID, primID, u, v);
        } else{
            if(now->right != NULL) Query(now->right, ray, geomID, primID, u, v);
            if(now->left != NULL) Query(now->left, ray, geomID, primID, u, v);
        }
    } else{
        for(int i = 0; i < now->primCount; ++i){
            const auto shape = shapes[now->primIdxBuffer[i]];
            if (shape->rayIntersectShape(ray, primID, u, v)) {
               *geomID = shape->geometryID;
            }
        }
    }
    return ;
}

bool BVH::rayIntersect(Ray &ray, int *geomID, int *primID, float *u, float *v) const {
    //* todo 完成BVH求交
    Query(root, ray, geomID, primID, u, v);
    return (*geomID != -1);
}



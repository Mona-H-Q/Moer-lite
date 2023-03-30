#include "Octree.h"
#include <queue>
struct Octree::OctreeNode {
    AABB boundingBox;
    std::shared_ptr<OctreeNode> subNodes[8];
    int primCount = -1;
    int primIdxBuffer[ocLeafMaxSize];
};
std::shared_ptr<Octree::OctreeNode> Octree::recursiveBuild(const AABB &aabb,
                       const std::vector<int> &primIdxBuffer) {
    //* todo 完成递归构建八叉树
    //* 构建方法请看实验手册
    //* 要注意的一种特殊是当节点的某个子包围盒和当前节点所有物体都相交，我们就不用细分了，当前节点作为叶子节点即可。
    // initial
    std::shared_ptr<OctreeNode> now = std::make_shared<OctreeNode>();
    now->boundingBox = aabb;
    for(int i = 0; i < 8; ++i) now->subNodes[i] = NULL;
    now->primCount = 0;
    // leaf
    if(primIdxBuffer.size() <= ocLeafMaxSize){
        for(int index : primIdxBuffer)
            now->primIdxBuffer[now->primCount++] = index;
    } else{
        Point3f pMin = aabb.pMin, pMax = aabb.pMax, pMid = aabb.Center();
        std::vector<int> subBuffer[8];
        AABB naabb[8];
        int Mx = 0;
        // solve all sonNode information and get Mx
        for(int i = 0; i < 8; ++i){
            for(int j = 0; j < 3; ++j)
                if(i & (1 << j)){
                    naabb[i].pMin[j] = pMid[j];
                    naabb[i].pMax[j] = pMax[j];
                } else{
                    naabb[i].pMin[j] = pMin[j];
                    naabb[i].pMax[j] = pMid[j];
                }
            for(int index : primIdxBuffer)
                if(shapes[index]->getAABB().Overlap(naabb[i]))
                    subBuffer[i].push_back(index);
            Mx = std::max(Mx, (int)subBuffer[i].size());
        }
        if(Mx < (int)primIdxBuffer.size()){
            for(int i = 0; i < 8; ++i)
                if(!subBuffer[i].empty())
                    now->subNodes[i] = recursiveBuild(naabb[i], subBuffer[i]);
        } else // set this node as a leaf
            for(int index : primIdxBuffer)
                now->primIdxBuffer[now->primCount++] = index;
    }
    return now;
}
void Octree::build() {
    //* 首先计算整个场景的范围
    for (const auto & shape : shapes) {
        //* 自行实现的加速结构请务必对每个shape调用该方法，以保证TriangleMesh构建内部加速结构
        //* 由于使用embree时，TriangleMesh::getAABB不会被调用，因此出于性能考虑我们不在TriangleMesh
        //* 的构造阶段计算其AABB，因此当我们将TriangleMesh的AABB计算放在TriangleMesh::initInternalAcceleration中
        //* 所以请确保在调用TriangleMesh::getAABB之前先调用TriangleMesh::initInternalAcceleration
        shape->initInternalAcceleration();

        boundingBox.Expand(shape->getAABB());
    }

    //* 构建八叉树
    std::vector<int> primIdxBuffer(shapes.size());
    std::iota(primIdxBuffer.begin(), primIdxBuffer.end(), 0);
    root = recursiveBuild(boundingBox, primIdxBuffer);
}

void Octree::Query(std::shared_ptr<OctreeNode> now, Ray &ray, 
                    int *geomID, int *primID, float *u, float *v) const{
    if(!now->boundingBox.RayIntersect(ray)) return ;
    if(now->primCount == 0){
        for(int i = 0; i < 8; ++i)
            if(now->subNodes[i] != NULL)
                Query(now->subNodes[i], ray, geomID, primID, u, v);
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
bool Octree::rayIntersect(Ray &ray, int *geomID, int *primID,
                          float *u, float *v) const {
    //*todo 完成八叉树求交
    Query(root, ray, geomID, primID, u, v);
    return (*geomID != -1);
}
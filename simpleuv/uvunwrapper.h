#ifndef SIMPLEUV_UV_UNWRAPPER_H
#define SIMPLEUV_UV_UNWRAPPER_H
#include <vector>
#include <map>
#include <simpleuv/meshdatatype.h>

namespace simpleuv 
{

class UvUnwrapper
{
public:
    void setMesh(const Mesh &mesh);
    void unwrap();
    const std::vector<FaceTextureCoords> &getFaceUvs();

private:
    void partition();
    void unwrapSinglePartition(const std::vector<size_t> &group, bool skipCheckHoles=false);
    void parametrizeSingleGroup(const std::vector<Vertex> &verticies,
        const std::vector<Face> &faces,
        std::map<size_t, size_t> &localToGlobalFacesMap,
        size_t faceNumToChart);
    // The hole filling faces should be put in the back of faces vector, so these uv coords of appended faces will be disgarded.
    void fixHolesExceptTheRing(const std::vector<Vertex> &verticies, std::vector<Face> &faces, size_t *remainingHoleNum=nullptr);
    void makeSeamAndCut(const std::vector<Vertex> &verticies, const std::vector<Face> &face, std::vector<size_t> &firstGroup, std::vector<size_t> &secondGroup);
    void packCharts();
    void finalizeUv();

    Mesh m_mesh;
    std::vector<FaceTextureCoords> m_faceUvs;
    std::map<int, std::vector<size_t>> m_partitions;
    std::vector<std::pair<std::vector<size_t>, std::vector<FaceTextureCoords>>> m_charts;
};

}

#endif

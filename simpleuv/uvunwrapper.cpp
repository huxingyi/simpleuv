#include <map>
#include <simpleuv/uvunwrapper.h>
#include <simpleuv/parametrize.h>

namespace simpleuv 
{

void UvUnwrapper::setMesh(const Mesh &mesh)
{
    m_mesh = mesh;
}

const std::vector<FaceTextureCoords> &UvUnwrapper::getFaceUvs()
{
    return m_faceUvs;
}

void UvUnwrapper::fixHolesExceptTheRing(const std::vector<Vertex> &verticies, std::vector<Face> &faces, size_t *remainingHoleNum)
{
    // TODO:
}

void UvUnwrapper::makeSeamAndCut(const std::vector<Vertex> &verticies, const std::vector<Face> &face, std::vector<size_t> &firstGroup, std::vector<size_t> &secondGroup)
{
    // TODO:
}

void UvUnwrapper::packCharts()
{
    // TODO:
}

void UvUnwrapper::finalizeUv()
{
    // TODO:
}

void UvUnwrapper::partition()
{
    m_partitions.clear();
    if (m_mesh.facePartitions.empty()) {
        for (decltype(m_mesh.faces.size()) i = 0; i < m_mesh.faces.size(); i++) {
            m_partitions[0].push_back(i);
        }
    } else {
        for (decltype(m_mesh.faces.size()) i = 0; i < m_mesh.faces.size(); i++) {
            int partition = m_mesh.facePartitions[i];
            m_partitions[partition].push_back(i);
        }
    }
}

void UvUnwrapper::unwrapSinglePartition(const std::vector<size_t> &group, bool skipCheckHoles)
{
    std::vector<Vertex> localVertices;
    std::vector<Face> localFaces;
    std::map<size_t, size_t> globalToLocalVerticesMap;
    std::map<size_t, size_t> localToGlobalFacesMap;
    for (decltype(group.size()) i = 0; i < group.size(); ++i) {
        const auto &globalFace = m_mesh.faces[group[i]];
        Face localFace;
        for (size_t j = 0; j < 3; j++) {
            int globalVertexIndex = globalFace.indicies[j];
            if (globalToLocalVerticesMap.find(globalVertexIndex) == globalToLocalVerticesMap.end()) {
                localVertices.push_back(m_mesh.verticies[globalVertexIndex]);
                globalToLocalVerticesMap[globalVertexIndex] = (int)localVertices.size() - 1;
            }
            localFace.indicies[j] = globalToLocalVerticesMap[globalVertexIndex];
        }
        localFaces.push_back(localFace);
        localToGlobalFacesMap[localFaces.size() - 1] = group[i];
    }

    if (skipCheckHoles) {
        parametrizeSingleGroup(localVertices, localFaces, localToGlobalFacesMap, localFaces.size());
        return;
    }

    decltype(localFaces.size()) faceNumBeforeFix = localFaces.size();
    size_t remainingHoleNumAfterFix = 0;
    fixHolesExceptTheRing(localVertices, localFaces, &remainingHoleNumAfterFix);
    if (1 == remainingHoleNumAfterFix) {
        parametrizeSingleGroup(localVertices, localFaces, localToGlobalFacesMap, faceNumBeforeFix);
        return;
    }
    
    if (0 == remainingHoleNumAfterFix) {
        std::vector<size_t> firstGroup;
        std::vector<size_t> secondGroup;
        makeSeamAndCut(localVertices, localFaces, firstGroup, secondGroup);
        for (auto &index: firstGroup) {
            index = localToGlobalFacesMap[index];
        }
        for (auto &index: secondGroup) {
            index = localToGlobalFacesMap[index];
        }
        unwrapSinglePartition(firstGroup, true);
        unwrapSinglePartition(secondGroup, true);
        return;
    }
}

void UvUnwrapper::parametrizeSingleGroup(const std::vector<Vertex> &verticies,
        const std::vector<Face> &faces,
        std::map<size_t, size_t> &localToGlobalFacesMap,
        size_t faceNumToChart)
{
    std::vector<TextureCoord> localVertexUvs;
    parametrize(verticies, faces, localVertexUvs);
    std::pair<std::vector<size_t>, std::vector<FaceTextureCoords>> chart;
    for (size_t i = 0; i < faceNumToChart; ++i) {
        const auto &localFace = faces[i];
        auto globalFaceIndex = localToGlobalFacesMap[i];
        FaceTextureCoords faceUv;
        for (size_t j = 0; j < 3; j++) {
            const auto &localVertexIndex = localFace.indicies[j];
            const auto &vertexUv = localVertexUvs[localVertexIndex];
            faceUv.coords[j].uv[0] = vertexUv.uv[0];
            faceUv.coords[j].uv[1] = vertexUv.uv[1];
        }
        chart.first.push_back(globalFaceIndex);
        chart.second.push_back(faceUv);
    }
    m_charts.push_back(chart);
}

void UvUnwrapper::unwrap()
{
    partition();

    m_faceUvs.resize(m_mesh.faces.size());
    for (const auto &group: m_partitions) {
        unwrapSinglePartition(group.second);
    }

    packCharts();
    finalizeUv();
}

}

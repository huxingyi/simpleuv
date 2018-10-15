#include <map>
#include <unordered_set>
#include <queue>
#include <simpleuv/uvunwrapper.h>
#include <simpleuv/parametrize.h>
#include <simpleuv/chartpacker.h>
#include <simpleuv/triangulate.h>

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

void UvUnwrapper::buildEdgeToFaceMap(const std::vector<Face> &faces, std::map<std::pair<size_t, size_t>, size_t> &edgeToFaceMap)
{
    edgeToFaceMap.clear();
    for (decltype(faces.size()) index = 0; index < faces.size(); ++index) {
        const auto &face = faces[index];
        for (size_t i = 0; i < 3; i++) {
            size_t j = (i + 1) % 3;
            edgeToFaceMap[{face.indicies[i], face.indicies[j]}] = index;
        }
    }
}

void UvUnwrapper::buildEdgeToFaceMap(const std::vector<size_t> &group, std::map<std::pair<size_t, size_t>, size_t> &edgeToFaceMap)
{
    edgeToFaceMap.clear();
    for (const auto &index: group) {
        const auto &face = m_mesh.faces[index];
        for (size_t i = 0; i < 3; i++) {
            size_t j = (i + 1) % 3;
            edgeToFaceMap[{face.indicies[i], face.indicies[j]}] = index;
        }
    }
}

void UvUnwrapper::splitPartitionToIslands(const std::vector<size_t> &group, std::vector<std::vector<size_t>> &islands)
{
    std::map<std::pair<size_t, size_t>, size_t> edgeToFaceMap;
    buildEdgeToFaceMap(group, edgeToFaceMap);
    
    std::unordered_set<size_t> processedFaces;
    std::queue<size_t> waitFaces;
    for (const auto &indexInGroup: group) {
        if (processedFaces.find(indexInGroup) != processedFaces.end())
            continue;
        waitFaces.push(indexInGroup);
        std::vector<size_t> island;
        while (!waitFaces.empty()) {
            size_t index = waitFaces.front();
            waitFaces.pop();
            if (processedFaces.find(index) != processedFaces.end())
                continue;
            const auto &face = m_mesh.faces[index];
            for (size_t i = 0; i < 3; i++) {
                size_t j = (i + 1) % 3;
                auto findOppositeFaceResult = edgeToFaceMap.find({face.indicies[j], face.indicies[i]});
                if (findOppositeFaceResult == edgeToFaceMap.end())
                    continue;
                waitFaces.push(findOppositeFaceResult->second);
            }
            island.push_back(index);
            processedFaces.insert(index);
        }
        if (island.empty())
            continue;
        islands.push_back(island);
    }
}

double UvUnwrapper::distanceBetweenVertices(const Vertex &first, const Vertex &second)
{
    float x = first.xyz[0] - second.xyz[0];
    float y = first.xyz[1] - second.xyz[1];
    float z = first.xyz[2] - second.xyz[2];
    return std::sqrt(x*x + y*y + z*z);
}

void UvUnwrapper::calculateFaceTextureBoundingBox(const std::vector<FaceTextureCoords> &faceTextureCoords,
        float &left, float &top, float &right, float &bottom)
{
    bool leftFirstTime = true;
    bool topFirstTime = true;
    bool rightFirstTime = true;
    bool bottomFirstTime = true;
    for (const auto &item: faceTextureCoords) {
        for (int i = 0; i < 3; ++i) {
            const auto &x = item.coords[i].uv[0];
            const auto &y = item.coords[i].uv[1];
            if (leftFirstTime || x < left) {
                left = x;
                leftFirstTime = false;
            }
            if (rightFirstTime || x > right) {
                right = x;
                rightFirstTime = false;
            }
            if (topFirstTime || y < top) {
                top = y;
                topFirstTime = false;
            }
            if (bottomFirstTime || y > bottom) {
                bottom = y;
                bottomFirstTime = false;
            }
        }
    }
}

void UvUnwrapper::triangulateRing(const std::vector<Vertex> &verticies,
        std::vector<Face> &faces, const std::vector<size_t> &ring)
{
    triangulate(verticies, faces, ring);
}

// The hole filling faces should be put in the back of faces vector, so these uv coords of appended faces will be disgarded.
bool UvUnwrapper::fixHolesExceptTheLongestRing(const std::vector<Vertex> &verticies, std::vector<Face> &faces, size_t *remainingHoleNum)
{
    std::map<std::pair<size_t, size_t>, size_t> edgeToFaceMap;
    buildEdgeToFaceMap(faces, edgeToFaceMap);
    
    std::map<size_t, size_t> holeVertexLink;
    for (const auto &face: faces) {
        for (size_t i = 0; i < 3; i++) {
            size_t j = (i + 1) % 3;
            auto findOppositeFaceResult = edgeToFaceMap.find({face.indicies[j], face.indicies[i]});
            if (findOppositeFaceResult != edgeToFaceMap.end())
                continue;
            holeVertexLink[face.indicies[j]] = face.indicies[i];
        }
    }
    
    std::vector<std::pair<std::vector<size_t>, double>> holeRings;
    std::unordered_set<size_t> visited;
    while (!holeVertexLink.empty()) {
        std::vector<size_t> ring;
        double ringLength = 0;
        auto first = holeVertexLink.begin()->first;
        auto index = first;
        auto prev = first;
        ring.push_back(first);
        visited.insert(first);
        while (true) {
            auto findLinkResult = holeVertexLink.find(index);
            if (findLinkResult == holeVertexLink.end()) {
                return false;
            }
            index = findLinkResult->second;
            if (index == first) {
                break;
            }
            if (visited.find(index) != visited.end()) {
                return false;
            }
            ring.push_back(index);
            ringLength += distanceBetweenVertices(verticies[index], verticies[prev]);
            prev = index;
        }
        holeRings.push_back({ring, ringLength});
        for (const auto &index: ring)
            holeVertexLink.erase(index);
    }
    
    if (holeRings.size() > 1) {
        // Sort by ring length, the longer ring sit in the lower array indicies
        std::sort(holeRings.begin(), holeRings.end(), [](const std::pair<std::vector<size_t>, double> &first, const std::pair<std::vector<size_t>, double> &second) {
            return first.second > second.second;
        });
        for (size_t i = 1; i < holeRings.size(); ++i) {
            triangulateRing(verticies, faces, holeRings[i].first);
        }
        holeRings.resize(1);
    }
    
    if (remainingHoleNum)
        *remainingHoleNum = holeRings.size();
    return true;
}

// FIXME: Currently we cut off the mesh by the middle y value for the sake of easy implementation.
void UvUnwrapper::makeSeamAndCut(const std::vector<Vertex> &verticies,
        const std::vector<Face> &faces,
        std::map<size_t, size_t> &localToGlobalFacesMap,
        std::vector<size_t> &firstGroup, std::vector<size_t> &secondGroup)
{
    double sumY = 0;
    size_t countY = 0;
    for (decltype(faces.size()) i = 0; i < faces.size(); ++i) {
        const auto &face = faces[i];
        for (int j = 0; j < 3; ++j) {
            const auto &vertex = verticies[face.indicies[j]];
            sumY += vertex.xyz[2];
            ++countY;
        }
    }
    if (countY == 0)
        return;
    float averageY = (float)(sumY / countY);
    for (decltype(faces.size()) i = 0; i < faces.size(); ++i) {
        const auto &face = faces[i];
        bool insertToFirst = true;
        for (int j = 0; j < 3; ++j) {
            const auto &vertex = verticies[face.indicies[j]];
            if (vertex.xyz[2] >= averageY) {
                insertToFirst = false;
                break;
            }
        }
        if (insertToFirst)
            firstGroup.push_back(localToGlobalFacesMap[i]);
        else
            secondGroup.push_back(localToGlobalFacesMap[i]);
    }
}

void UvUnwrapper::packCharts()
{
    std::vector<std::pair<float, float>> chartSizes;
    for (auto &chart: m_charts) {
        float left, top, right, bottom;
        left = top = right = bottom = 0;
        calculateFaceTextureBoundingBox(chart.second, left, top, right, bottom);
        for (auto &item: chart.second) {
            for (int i = 0; i < 3; ++i) {
                item.coords[i].uv[0] -= left;
                item.coords[i].uv[1] -= top;
            }
        }
        chartSizes.push_back({right - left, bottom - top});
    }
    ChartPacker chartPacker;
    chartPacker.setCharts(chartSizes);
    const std::vector<std::tuple<float, float, float, float, bool>> &packedResult = chartPacker.getResult();
    for (decltype(m_charts.size()) i = 0; i < m_charts.size(); ++i) {
        auto &chart = m_charts[i];
        const auto &result = packedResult[i];
        auto &left = std::get<0>(result);
        auto &top = std::get<1>(result);
        auto &width = std::get<2>(result);
        auto &height = std::get<3>(result);
        auto &flipped = std::get<4>(result);
        if (flipped) {
            for (auto &item: chart.second) {
                for (int i = 0; i < 3; ++i) {
                    std::swap(item.coords[i].uv[0], item.coords[i].uv[1]);
                }
            }
        }
        for (auto &item: chart.second) {
            for (int i = 0; i < 3; ++i) {
                item.coords[i].uv[0] /= width;
                item.coords[i].uv[1] /= height;
                item.coords[i].uv[0] += left;
                item.coords[i].uv[1] += top;
            }
        }
    }
}

void UvUnwrapper::finalizeUv()
{
    m_faceUvs.resize(m_mesh.faces.size());
    for (const auto &chart: m_charts) {
        for (decltype(chart.second.size()) i = 0; i < chart.second.size(); ++i) {
            auto &faceUv = m_faceUvs[chart.first[i]];
            faceUv = chart.second[i];
        }
    }
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

void UvUnwrapper::unwrapSingleIsland(const std::vector<size_t> &group, bool skipCheckHoles)
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
    if (!fixHolesExceptTheLongestRing(localVertices, localFaces, &remainingHoleNumAfterFix)) {
        return;
    }
    if (1 == remainingHoleNumAfterFix) {
        parametrizeSingleGroup(localVertices, localFaces, localToGlobalFacesMap, faceNumBeforeFix);
        return;
    }
    
    if (0 == remainingHoleNumAfterFix) {
        std::vector<size_t> firstGroup;
        std::vector<size_t> secondGroup;
        makeSeamAndCut(localVertices, localFaces, localToGlobalFacesMap, firstGroup, secondGroup);
        for (auto &index: firstGroup) {
            index = localToGlobalFacesMap[index];
        }
        for (auto &index: secondGroup) {
            index = localToGlobalFacesMap[index];
        }
        unwrapSingleIsland(firstGroup, true);
        unwrapSingleIsland(secondGroup, true);
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
    if (chart.first.empty())
        return;
    m_charts.push_back(chart);
}

void UvUnwrapper::unwrap()
{
    partition();

    m_faceUvs.resize(m_mesh.faces.size());
    for (const auto &group: m_partitions) {
        std::vector<std::vector<size_t>> islands;
        splitPartitionToIslands(group.second, islands);
        for (const auto &island: islands)
            unwrapSingleIsland(island);
    }

    packCharts();
    finalizeUv();
}

}

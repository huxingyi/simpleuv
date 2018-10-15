#include <simpleuv/chartpacker.h>
#include <cmath>
extern "C" {
#include <maxrects.h>
}

namespace simpleuv
{

void ChartPacker::setCharts(const std::vector<std::pair<float, float>> &chartSizes)
{
    m_chartSizes = chartSizes;
}

const std::vector<std::tuple<float, float, float, float, bool>> &ChartPacker::getResult()
{
    return m_result;
}

double ChartPacker::calculateTotalArea()
{
    double totalArea = 0;
    for (const auto &chartSize: m_chartSizes) {
        totalArea += chartSize.first * chartSize.second;
    }
    return totalArea;
}

bool ChartPacker::tryPack(float textureSize)
{
    std::vector<maxRectsSize> rects;
    int width = textureSize * m_floatToIntFactor;
    int height = width;
    for (const auto &chartSize: m_chartSizes) {
        maxRectsSize r;
        r.width = chartSize.first * m_floatToIntFactor;
        r.height = chartSize.second * m_floatToIntFactor;
        rects.push_back(r);
    }
    const maxRectsFreeRectChoiceHeuristic methods[] = {
        rectBestShortSideFit,
        rectBestLongSideFit,
        rectBestAreaFit,
        rectBottomLeftRule,
        rectContactPointRule
    };
    float occupancy = 0;
    float bestOccupancy = 0;
    std::vector<maxRectsPosition> bestResult;
    for (size_t i = 0; i < sizeof(methods) / sizeof(methods[0]); ++i) {
        std::vector<maxRectsPosition> result(rects.size());
        if (!maxRects(width, height, rects.size(), rects.data(), methods[i], true, result.data(), &occupancy)) {
            continue;
        }
        if (occupancy >= bestOccupancy) {
            bestResult = result;
            bestOccupancy = occupancy;
        }
    }
    if (bestResult.size() != rects.size())
        return false;
    m_result.resize(bestResult.size());
    for (decltype(bestResult.size()) i = 0; i < bestResult.size(); ++i) {
        const auto &result = bestResult[i];
        const auto &rect = rects[i];
        m_result[i] = {result.left / textureSize, result.top / textureSize, rect.width / textureSize, rect.height / textureSize, result.rotated};
    }
    return true;
}

void ChartPacker::pack()
{
    float initialGuessSize = std::sqrt(calculateTotalArea() * m_initialAreaGuessFactor);
    float textureSize = initialGuessSize;
    while (true) {
        if (tryPack(textureSize))
            break;
        textureSize *= 1.0 + m_textureSizeGrowFactor;
    }
}

}

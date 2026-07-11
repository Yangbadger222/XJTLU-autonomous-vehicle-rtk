#include "scan_context_index.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <utility>
#include <yaml-cpp/yaml.h>

namespace
{
constexpr float kPi = 3.14159265358979323846F;

float squaredDistance(const std::vector<float> &a, const std::vector<float> &b)
{
    if (a.size() != b.size())
        return std::numeric_limits<float>::infinity();
    float value = 0.0F;
    for (std::size_t index = 0; index < a.size(); ++index)
    {
        const float delta = a[index] - b[index];
        value += delta * delta;
    }
    return value;
}
}  // namespace

bool ScanContextIndex::load(const std::string &path)
{
    YAML::Node root = YAML::LoadFile(path);
    if (!root || !root["candidates"])
        return false;
    m_rings = root["rings"].as<int>();
    m_sectors = root["sectors"].as<int>();
    m_max_radius_m = root["max_radius_m"].as<float>();
    if (m_rings <= 0 || m_sectors <= 0 || m_max_radius_m <= 0.0F)
        return false;
    m_candidates.clear();
    for (const auto &node : root["candidates"])
    {
        Candidate candidate;
        candidate.patch = node["patch"].as<std::string>();
        candidate.region = node["region"] ? node["region"].as<std::string>() : "";
        candidate.ring_key = node["ring_key"].as<std::vector<float>>();
        candidate.descriptor = node["descriptor"].as<std::vector<float>>();
        const auto position = node["position"].as<std::vector<float>>();
        const auto quaternion = node["quaternion_wxyz"].as<std::vector<float>>();
        if (position.size() != 3 || quaternion.size() != 4 ||
            candidate.ring_key.size() != static_cast<std::size_t>(m_rings) ||
            candidate.descriptor.size() != static_cast<std::size_t>(m_rings * m_sectors))
            continue;
        Eigen::Quaternionf q(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
        if (q.squaredNorm() < 1.0e-6F)
            continue;
        candidate.pose.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
        candidate.pose.block<3, 1>(0, 3) = Eigen::Vector3f(position[0], position[1], position[2]);
        m_candidates.push_back(std::move(candidate));
    }
    return !m_candidates.empty();
}

std::vector<float> ScanContextIndex::descriptor(const CloudType::Ptr &cloud) const
{
    std::vector<float> result(static_cast<std::size_t>(m_rings * m_sectors), 0.0F);
    if (!cloud || cloud->empty())
        return result;
    float min_z = std::numeric_limits<float>::infinity();
    for (const auto &point : *cloud)
        min_z = std::min(min_z, point.z);
    for (const auto &point : *cloud)
    {
        const float radius = std::hypot(point.x, point.y);
        if (!std::isfinite(radius) || radius <= 0.0F || radius > m_max_radius_m)
            continue;
        float angle = std::atan2(point.y, point.x);
        if (angle < 0.0F)
            angle += 2.0F * kPi;
        const int ring = std::min(m_rings - 1, static_cast<int>(radius / m_max_radius_m * m_rings));
        const int sector = std::min(m_sectors - 1, static_cast<int>(angle / (2.0F * kPi) * m_sectors));
        float &bin = result[static_cast<std::size_t>(ring * m_sectors + sector)];
        bin = std::max(bin, point.z - min_z + 1.0F);
    }
    return result;
}

std::vector<float> ScanContextIndex::ringKey(const std::vector<float> &input) const
{
    std::vector<float> key(static_cast<std::size_t>(m_rings), 0.0F);
    for (int ring = 0; ring < m_rings; ++ring)
    {
        float sum = 0.0F;
        for (int sector = 0; sector < m_sectors; ++sector)
            sum += input[static_cast<std::size_t>(ring * m_sectors + sector)];
        key[static_cast<std::size_t>(ring)] = sum / static_cast<float>(m_sectors);
    }
    return key;
}

float ScanContextIndex::descriptorDistance(
    const std::vector<float> &query,
    const std::vector<float> &candidate,
    int &best_shift) const
{
    float best = std::numeric_limits<float>::infinity();
    best_shift = 0;
    for (int shift = 0; shift < m_sectors; ++shift)
    {
        float similarity_sum = 0.0F;
        int valid_columns = 0;
        for (int sector = 0; sector < m_sectors; ++sector)
        {
            float dot = 0.0F;
            float query_norm = 0.0F;
            float candidate_norm = 0.0F;
            const int shifted_sector = (sector + shift) % m_sectors;
            for (int ring = 0; ring < m_rings; ++ring)
            {
                const float q = query[static_cast<std::size_t>(ring * m_sectors + sector)];
                const float c = candidate[static_cast<std::size_t>(ring * m_sectors + shifted_sector)];
                dot += q * c;
                query_norm += q * q;
                candidate_norm += c * c;
            }
            if (query_norm > 1.0e-6F && candidate_norm > 1.0e-6F)
            {
                similarity_sum += dot / std::sqrt(query_norm * candidate_norm);
                ++valid_columns;
            }
        }
        const float distance = valid_columns > 0
            ? 1.0F - similarity_sum / static_cast<float>(valid_columns)
            : 1.0F;
        if (distance < best)
        {
            best = distance;
            best_shift = shift;
        }
    }
    return best;
}

std::vector<GlobalPoseCandidate> ScanContextIndex::query(
    const CloudType::Ptr &cloud,
    const std::string &region,
    int max_candidates,
    float &score_gap) const
{
    const auto query_descriptor = descriptor(cloud);
    const auto query_key = ringKey(query_descriptor);
    struct Ranked
    {
        float ring_score;
        const Candidate *candidate;
    };
    std::vector<Ranked> ranked;
    for (const auto &candidate : m_candidates)
    {
        if (!region.empty() && candidate.region != region)
            continue;
        ranked.push_back({squaredDistance(query_key, candidate.ring_key), &candidate});
    }
    std::sort(ranked.begin(), ranked.end(), [](const Ranked &a, const Ranked &b) {
        return a.ring_score < b.ring_score;
    });
    const int shortlist_size = std::min<int>(ranked.size(), std::max(1, max_candidates) * 3);
    std::vector<GlobalPoseCandidate> output;
    for (int index = 0; index < shortlist_size; ++index)
    {
        int shift = 0;
        const float score = descriptorDistance(
            query_descriptor, ranked[static_cast<std::size_t>(index)].candidate->descriptor, shift);
        const float yaw = -static_cast<float>(shift) * 2.0F * kPi / static_cast<float>(m_sectors);
        M4F yaw_correction = M4F::Identity();
        yaw_correction.block<3, 3>(0, 0) =
            Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
        output.push_back(
            {ranked[static_cast<std::size_t>(index)].candidate->pose * yaw_correction,
             score,
             ranked[static_cast<std::size_t>(index)].candidate->patch});
    }
    std::sort(output.begin(), output.end(), [](const GlobalPoseCandidate &a, const GlobalPoseCandidate &b) {
        return a.descriptor_score < b.descriptor_score;
    });
    if (output.size() > static_cast<std::size_t>(std::max(1, max_candidates)))
        output.resize(static_cast<std::size_t>(std::max(1, max_candidates)));
    score_gap = output.size() >= 2 ? output[1].descriptor_score - output[0].descriptor_score : 1.0F;
    return output;
}

#pragma once

#include "commons.h"

#include <string>
#include <vector>

struct GlobalPoseCandidate
{
    M4F pose = M4F::Identity();
    float descriptor_score = 1.0F;
    std::string patch;
};

class ScanContextIndex
{
public:
    bool load(const std::string &path);
    std::vector<GlobalPoseCandidate> query(
        const CloudType::Ptr &cloud,
        const std::string &region,
        int max_candidates,
        float &score_gap) const;
    bool empty() const { return m_candidates.empty(); }

private:
    struct Candidate
    {
        M4F pose = M4F::Identity();
        std::string patch;
        std::string region;
        std::vector<float> ring_key;
        std::vector<float> descriptor;
    };

    std::vector<float> descriptor(const CloudType::Ptr &cloud) const;
    std::vector<float> ringKey(const std::vector<float> &descriptor) const;
    float descriptorDistance(
        const std::vector<float> &query,
        const std::vector<float> &candidate,
        int &best_shift) const;

    int m_rings = 20;
    int m_sectors = 60;
    float m_max_radius_m = 20.0F;
    std::vector<Candidate> m_candidates;
};

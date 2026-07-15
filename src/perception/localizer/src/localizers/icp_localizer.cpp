#include "icp_localizer.h"

#include <iostream>

#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>

ICPLocalizer::ICPLocalizer(const ICPConfig &config) : m_config(config)
{
    m_refine_inp.reset(new CloudType);
    m_refine_tgt.reset(new CloudType);
    m_rough_inp.reset(new CloudType);
    m_rough_tgt.reset(new CloudType);
}
bool ICPLocalizer::loadMap(const std::string &path)
{
    if (!std::filesystem::exists(path))
    {
        std::cerr << "Map file not found: " << path << std::endl;
        return false;
    }
    pcl::PCDReader reader;
    CloudType::Ptr cloud(new CloudType);
    if (reader.read(path, *cloud) < 0 || cloud->empty())
    {
        std::cerr << "Map PCD is unreadable or empty: " << path << std::endl;
        return false;
    }
    if (m_config.refine_map_resolution > 0)
    {
        m_voxel_filter.setLeafSize(m_config.refine_map_resolution, m_config.refine_map_resolution, m_config.refine_map_resolution);
        m_voxel_filter.setInputCloud(cloud);
        m_voxel_filter.filter(*m_refine_tgt);
    }
    else
    {
        pcl::copyPointCloud(*cloud, *m_refine_tgt);
    }

    if (m_config.rough_map_resolution > 0)
    {
        m_voxel_filter.setLeafSize(m_config.rough_map_resolution, m_config.rough_map_resolution, m_config.rough_map_resolution);
        m_voxel_filter.setInputCloud(cloud);
        m_voxel_filter.filter(*m_rough_tgt);
    }
    else
    {
        pcl::copyPointCloud(*cloud, *m_rough_tgt);
    }
    return true;
}
void ICPLocalizer::setInput(const CloudType::Ptr &cloud)
{
    if (m_config.refine_scan_resolution > 0)
    {
        m_voxel_filter.setLeafSize(m_config.refine_scan_resolution, m_config.refine_scan_resolution, m_config.refine_scan_resolution);
        m_voxel_filter.setInputCloud(cloud);
        m_voxel_filter.filter(*m_refine_inp);
    }
    else
    {
        pcl::copyPointCloud(*cloud, *m_refine_inp);
    }

    if (m_config.rough_scan_resolution > 0)
    {
        m_voxel_filter.setLeafSize(m_config.rough_scan_resolution, m_config.rough_scan_resolution, m_config.rough_scan_resolution);
        m_voxel_filter.setInputCloud(cloud);
        m_voxel_filter.filter(*m_rough_inp);
    }
    else
    {
        pcl::copyPointCloud(*cloud, *m_rough_inp);
    }
}

bool ICPLocalizer::align(M4F &guess)
{
    m_last_rough_score = std::numeric_limits<double>::infinity();
    m_last_refine_score = std::numeric_limits<double>::infinity();
    m_last_overlap_ratio = 0.0;
    CloudType::Ptr aligned_cloud(new CloudType);
    if (m_refine_tgt->empty() || m_rough_tgt->empty() ||
        m_refine_inp->empty() || m_rough_inp->empty())
        return false;
    m_rough_icp.setMaximumIterations(m_config.rough_max_iteration);
    m_rough_icp.setInputSource(m_rough_inp);
    m_rough_icp.setInputTarget(m_rough_tgt);
    m_rough_icp.align(*aligned_cloud, guess);
    m_last_rough_score = m_rough_icp.getFitnessScore();
    if (!m_rough_icp.hasConverged() || m_last_rough_score > m_config.rough_score_thresh)
        return false;
    m_refine_icp.setMaximumIterations(m_config.refine_max_iteration);
    m_refine_icp.setInputSource(m_refine_inp);
    m_refine_icp.setInputTarget(m_refine_tgt);
    m_refine_icp.align(*aligned_cloud, m_rough_icp.getFinalTransformation());
    m_last_refine_score = m_refine_icp.getFitnessScore();
    if (!m_refine_icp.hasConverged() || m_last_refine_score > m_config.refine_score_thresh)
        return false;
    guess = m_refine_icp.getFinalTransformation();

    CloudType::Ptr transformed(new CloudType);
    pcl::transformPointCloud(*m_refine_inp, *transformed, guess);
    pcl::KdTreeFLANN<PointType> tree;
    tree.setInputCloud(m_refine_tgt);
    std::vector<int> indices(1);
    std::vector<float> squared_distances(1);
    std::size_t inliers = 0;
    for (const auto &point : *transformed)
    {
        if (tree.nearestKSearch(point, 1, indices, squared_distances) > 0 && squared_distances[0] <= 0.09F)
            ++inliers;
    }
    m_last_overlap_ratio = transformed->empty()
        ? 0.0
        : static_cast<double>(inliers) / static_cast<double>(transformed->size());
    return true;
}

bool ICPLocalizer::alignCandidates(const std::vector<M4F> &guesses, M4F &best_guess, int &tested)
{
    bool found = false;
    double best_score = std::numeric_limits<double>::infinity();
    double best_rough = best_score;
    double best_overlap = 0.0;
    tested = 0;
    for (const auto &candidate : guesses)
    {
        M4F aligned = candidate;
        ++tested;
        if (!align(aligned))
            continue;
        if (m_last_refine_score < best_score)
        {
            found = true;
            best_score = m_last_refine_score;
            best_rough = m_last_rough_score;
            best_overlap = m_last_overlap_ratio;
            best_guess = aligned;
        }
    }
    if (found)
    {
        m_last_refine_score = best_score;
        m_last_rough_score = best_rough;
        m_last_overlap_ratio = best_overlap;
    }
    return found;
}

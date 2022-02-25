/**
 * @copyright Copyright (c) 2017 B-com http://www.b-com.com/
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "SolARSLAMMapping.h"
#include "core/Log.h"


namespace xpcf = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::TOOLS::SolARSLAMMapping);
#define MIN_POINT_DISTANCE 0.04


namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {


SolARSLAMMapping::SolARSLAMMapping() :ConfigurableBase(xpcf::toUUID<SolARSLAMMapping>())
{
	addInterface<api::slam::IMapping>(this);
	declareInjectable<api::storage::IMapManager>(m_mapManager);
	declareInjectable<api::storage::IPointCloudManager>(m_pointCloudManager);
	declareInjectable<api::storage::IKeyframesManager>(m_keyframesManager);
	declareInjectable<api::storage::ICovisibilityGraphManager>(m_covisibilityGraphManager);
	declareInjectable<api::reloc::IKeyframeRetriever>(m_keyframeRetriever);
	declareInjectable<api::solver::map::ITriangulator>(m_triangulator);
	declareInjectable<api::solver::map::IMapFilter>(m_mapFilter);
	declareInjectable<api::features::IDescriptorMatcherGeometric>(m_matcher);
	declareProperty("minWeightNeighbor", m_minWeightNeighbor);
	declareProperty("maxNbNeighborKfs", m_maxNbNeighborKfs);
	declareProperty("saveImage", m_isSaveImage);
	LOG_DEBUG("SolARSLAMMapping constructor");
}

void SolARSLAMMapping::setCameraParameters(const CameraParameters & camParams) {
	m_camParams = camParams;
	m_triangulator->setCameraParameters(m_camParams.intrinsic, m_camParams.distortion);
}

bool SolARSLAMMapping::idle()
{
    std::unique_lock<std::mutex> lock(m_mutexIdle);
    return m_idle;
}

void SolARSLAMMapping::setIdle(bool flag)
{
    std::unique_lock<std::mutex> lock(m_mutexIdle);
    m_idle = flag;
}

FrameworkReturnCode SolARSLAMMapping::process(const SRef<Frame> frame, SRef<Keyframe> & keyframe)
{
    setIdle(false);
    keyframe = processNewKeyframe(frame);
    setIdle(true);
    return FrameworkReturnCode::_SUCCESS;
}

SRef<Keyframe> SolARSLAMMapping::processNewKeyframe(const SRef<Frame>& frame)
{
	// create a new keyframe from the current frame
	if (!m_isSaveImage)
		frame->setView(nullptr);
	SRef<Keyframe> newKeyframe = xpcf::utils::make_shared<Keyframe>(frame);
	// Add to keyframe manager
	m_keyframesManager->addKeyframe(newKeyframe);
	// Add to BOW retrieval
	m_keyframeRetriever->addKeyframe(newKeyframe);
	// Update keypoint visibility, descriptor in cloud point and connections between new keyframe with other keyframes
	updateAssociateCloudPoint(newKeyframe);
	// Map point culling
	cloudPointsCulling(newKeyframe);
	// get best neighbor keyframes
	std::vector<uint32_t> idxNeighborKfs, idxBestNeighborKfs;
	m_covisibilityGraphManager->getNeighbors(newKeyframe->getId(), m_minWeightNeighbor, idxNeighborKfs);
	if (idxNeighborKfs.size() < m_maxNbNeighborKfs)
		idxBestNeighborKfs.swap(idxNeighborKfs);
	else
		idxBestNeighborKfs.insert(idxBestNeighborKfs.begin(), idxNeighborKfs.begin(), idxNeighborKfs.begin() + m_maxNbNeighborKfs);
	// find matches between unmatching keypoints in the new keyframe and the best neighboring keyframes
	std::vector<SRef<CloudPoint>> newCloudPoint;
	LOG_DEBUG("Nb of neighbors for mapping: {}", idxBestNeighborKfs.size());
	findMatchesAndTriangulation(newKeyframe, idxBestNeighborKfs, newCloudPoint);
    LOG_DEBUG("Nb of new triangulated 3D cloud points: {}", newCloudPoint.size());
	// add new points to point cloud manager, update visibility map and covisibility graph
	for (auto const &point : newCloudPoint) {
		m_mapManager->addCloudPoint(point);
		m_recentAddedCloudPoints[point->getId()] = std::make_pair(point, newKeyframe->getId());
	}
	return newKeyframe;
}

void SolARSLAMMapping::updateAssociateCloudPoint(const SRef<Keyframe>& keyframe)
{
	const std::map<uint32_t, uint32_t> &newkf_mapVisibility = keyframe->getVisibility();
	std::map<uint32_t, int> kfCounter;
	// calculate the number of connections to other keyframes
	for (auto const &it : newkf_mapVisibility) {
		SRef<CloudPoint> cloudPoint;
		if (m_pointCloudManager->getPoint(it.second, cloudPoint) == FrameworkReturnCode::_SUCCESS) {
			const std::map<uint32_t, uint32_t> &cpKfVisibility = cloudPoint->getVisibility();
			for (auto const &it_kf : cpKfVisibility)
				kfCounter[it_kf.first]++;			
			// update view direction
			const Transform3Df& poseNewKf = keyframe->getPose();
			Vector3f newViewDirection(poseNewKf(0, 3) - cloudPoint->getX(), poseNewKf(1, 3) - cloudPoint->getY(), poseNewKf(2, 3) - cloudPoint->getZ());
			cloudPoint->addNewViewDirection(newViewDirection.normalized());
			// update descriptor
			cloudPoint->addNewDescriptor(keyframe->getDescriptors()->getDescriptor(it.first));
			// add new visibility to cloud point
			cloudPoint->addVisibility(keyframe->getId(), it.first);
		}
	}

	// Add to covisibility graph
	for (auto const &it : kfCounter)
		if (it.first != keyframe->getId())
			m_covisibilityGraphManager->increaseEdge(keyframe->getId(), it.first, it.second);
}

void SolARSLAMMapping::findMatchesAndTriangulation(const SRef<Keyframe>& keyframe, const std::vector<uint32_t>& idxBestNeighborKfs, std::vector<SRef<CloudPoint>>& cloudPoint)
{
	const std::map<unsigned int, unsigned int> &newKf_mapVisibility = keyframe->getVisibility();
	const SRef<DescriptorBuffer> &newKf_des = keyframe->getDescriptors();
	const std::vector<Keypoint> & newKf_kpUn = keyframe->getUndistortedKeypoints();
	const Transform3Df& newKf_pose = keyframe->getPose();

	// Vector indices keypoints have no visibility to map point
	std::vector<bool> checkMatches(newKf_kpUn.size(), false);
	for (const auto& it: newKf_mapVisibility)
		checkMatches[it.first] = true;

	// Triangulate to neighboring keyframes
	for (int i = 0; i < idxBestNeighborKfs.size(); ++i) {				
		// get neighbor keyframe i
		SRef<Keyframe> tmpKf;
		if (m_keyframesManager->getKeyframe(idxBestNeighborKfs[i], tmpKf) != FrameworkReturnCode::_SUCCESS)
			continue;
		const Transform3Df &tmpKf_pose = tmpKf->getPose();
		const std::map<uint32_t, uint32_t> & tmpMapVisibility = tmpKf->getVisibility();
		// get median depth of neighbor keyframe
		float tmpKfMedDepth;
		{
			std::vector<float> depths;
			Transform3Df tmpKfPoseInv = tmpKf_pose.inverse();
			for (const auto& it : tmpMapVisibility) {
				SRef<CloudPoint> cp;
				if (m_pointCloudManager->getPoint(it.second, cp) != FrameworkReturnCode::_SUCCESS)
					continue;
				float depth = tmpKfPoseInv(2, 0) * cp->getX() + tmpKfPoseInv(2, 1) * cp->getY() + tmpKfPoseInv(2, 2) * cp->getZ() + tmpKfPoseInv(2, 3);
				depths.push_back(depth);
			}
			if (depths.size() == 0)
				continue;
			std::sort(depths.begin(), depths.end());
			tmpKfMedDepth = depths[depths.size() / 2];
		}
		
		// get keypoints don't have associated cloud points
		std::vector<uint32_t> newKf_indexKeypoints;
		for (int j = 0; j < checkMatches.size(); ++j)
			if (!checkMatches[j])
				newKf_indexKeypoints.push_back(j);

		// Feature matching based on epipolar constraint
		std::vector < DescriptorMatch> tmpMatches, goodMatches;
		m_matcher->match(keyframe, tmpKf, m_camParams, tmpMatches, newKf_indexKeypoints);

		// find info to triangulate						
		for (int j = 0; j < tmpMatches.size(); ++j) {
			unsigned int idx_newKf = tmpMatches[j].getIndexInDescriptorA();
			unsigned int idx_tmpKf = tmpMatches[j].getIndexInDescriptorB();
			if ((!checkMatches[idx_newKf]) && (tmpMapVisibility.find(idx_tmpKf) == tmpMapVisibility.end())) {
				goodMatches.push_back(tmpMatches[j]);
			}
		}
		if (goodMatches.size() == 0)
			continue;
		
		// triangulation		
		// check baseline: if baseline is very short, 3D points are defined by using only depth information of keypoints 
		std::vector<SRef<CloudPoint>> tmpCloudPoint;
		if ((tmpKf_pose.translation() - newKf_pose.translation()).norm() / tmpKfMedDepth < 0.02)
			m_triangulator->triangulate(keyframe, tmpKf, goodMatches, std::make_pair(keyframe->getId(), idxBestNeighborKfs[i]), tmpCloudPoint, true);
		else
			m_triangulator->triangulate(keyframe, tmpKf, goodMatches, std::make_pair(keyframe->getId(), idxBestNeighborKfs[i]), tmpCloudPoint, false);
		if (tmpCloudPoint.size() == 0)
			continue;

		// filter cloud points
		std::vector<SRef<CloudPoint>> tmpFilteredCloudPoint;
		std::vector<int> indexFiltered;
		if (tmpCloudPoint.size() > 0)
			m_mapFilter->filter(newKf_pose, tmpKf_pose, tmpCloudPoint, tmpFilteredCloudPoint, indexFiltered);
		for (int i = 0; i < indexFiltered.size(); ++i) {
			checkMatches[goodMatches[indexFiltered[i]].getIndexInDescriptorA()] = true;
			cloudPoint.push_back(tmpFilteredCloudPoint[i]);
		}
	}
}

void SolARSLAMMapping::cloudPointsCulling(const SRef<Keyframe>& keyframe)
{
	const uint32_t& currentKfId = keyframe->getId();
	int nbRemove(0);
	std::vector<uint32_t> toRemove;
	for (const auto &it : m_recentAddedCloudPoints) {		
		const SRef<CloudPoint>& cp = it.second.first;
		const uint32_t& cpIdKf = it.second.second;
		if (!m_pointCloudManager->isExistPoint(cp->getId())) {
			toRemove.push_back(it.first);
			continue;
		}
		if (((currentKfId - cpIdKf) >= 2) && (cp->getVisibility().size() < 3)) {
			//std::cout << "Erase point: " << it.first << " " << cp->getId() << std::endl;
			m_mapManager->removeCloudPoint(cp);
			toRemove.push_back(it.first);
			nbRemove++;
		}
		else if ((currentKfId - cpIdKf) > 2)
			toRemove.push_back(it.first);
	}
	for (const auto& it : toRemove)
		m_recentAddedCloudPoints.erase(it);
	
	LOG_DEBUG("Nb of culling points: {}", nbRemove);
	LOG_DEBUG("Nb of good points: {}", toRemove.size() - nbRemove);
}

}
}
}

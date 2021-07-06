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

#include "SolARMapUpdate.h"
#include "core/Log.h"

namespace xpcf  = org::bcom::xpcf;
XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::TOOLS::SolARMapUpdate);

#define RATIO_BORDER_IMAGE 0.08
#define RATIO_UPDATE_WINDOWS 0.03

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {

SolARMapUpdate::SolARMapUpdate():ConfigurableBase(xpcf::toUUID<SolARMapUpdate>())
{
    addInterface<api::solver::map::IMapUpdate>(this);
	declareInjectable<api::storage::IMapManager>(m_mapManager);
	declareInjectable<api::geom::IProject>(m_projector);
	declareInjectable<api::features::IDescriptorMatcherRegion>(m_matcher);
	declareProperty("thresAngleViewDirection", m_thresAngleViewDirection);
	LOG_DEBUG("SolARMapUpdate constructor");
}

xpcf::XPCFErrorCode SolARMapUpdate::onConfigured()
{
	LOG_DEBUG("SolARMapUpdate onConfigured");
	m_thresConfidence = m_mapManager->bindTo<xpcf::IConfigurable>()->getProperty("thresConfidence")->getFloatingValue();
	LOG_DEBUG("ThresConfidence: {}", m_thresConfidence);
	LOG_DEBUG("ThresAngleViewDirection: {}", m_thresAngleViewDirection);
	return xpcf::XPCFErrorCode::_SUCCESS;
}

void SolARMapUpdate::setCameraParameters(const CamCalibration & intrinsicParams, const CamDistortion & distortionParams) 
{
	m_projector->setCameraParameters(intrinsicParams, distortionParams);
}

float SolARMapUpdate::cosineViewDirectionAngle(const SRef<Frame>& frame, const SRef<CloudPoint>& cloudPoint)
{
	const Transform3Df &pose = frame->getPose();
	Vector3f frameViewDir(pose(0, 3) - cloudPoint->getX(), pose(1, 3) - cloudPoint->getY(), pose(2, 3) - cloudPoint->getZ());
	const Vector3f& cpViewDir = cloudPoint->getViewDirection();
	return cpViewDir.dot(frameViewDir.normalized());
}

FrameworkReturnCode SolARMapUpdate::update(SRef<datastructure::Map> globalMap, const std::vector<uint32_t>& newKeyframeIds)
{
	m_mapManager->setMap(globalMap);
	m_covisibilityGraph = globalMap->getConstCovisibilityGraph();
	m_pointCloud = globalMap->getConstPointCloud();
	m_keyframeCollection = globalMap->getConstKeyframeCollection();
	std::set<uint32_t> setKfIds;
	for (const auto &kfId : newKeyframeIds)
		setKfIds.insert(kfId);
	// update
	for (const auto& kfId : newKeyframeIds) {
		// get new keyframe
		SRef<Keyframe> newKeyframe;
		if (m_keyframeCollection->getKeyframe(kfId, newKeyframe) != FrameworkReturnCode::_SUCCESS)
			continue;
		// get neighbor keyframes of the new keyframe in the old global map
		std::vector<uint32_t> neighbors;
		m_covisibilityGraph->getNeighbors(kfId, 1.f, neighbors);
		std::vector<uint32_t> validNeighbors;
		for (const auto& it : neighbors)
			if (setKfIds.find(it) == setKfIds.end())
				validNeighbors.push_back(it);
		if (validNeighbors.size() == 0)
			continue;
		std::vector<SRef<Keyframe>> neighborKeyframes;
		m_keyframeCollection->getKeyframes(validNeighbors, neighborKeyframes);
		// get cloud points seen by neighbor keyframes
		std::set<uint32_t> idxLocalCloudPoints;
		for (const auto &kf : neighborKeyframes) {
			const std::map<uint32_t, uint32_t>& mapPointVisibility = kf->getVisibility();
			for (auto const &it_pc : mapPointVisibility) {
				idxLocalCloudPoints.insert(it_pc.second);
			}
		}
		std::vector<SRef<CloudPoint>> localCloudPoints;
		for (const auto& it : idxLocalCloudPoints) {
			SRef<CloudPoint> cp;
			if (m_pointCloud->getPoint(it, cp) == FrameworkReturnCode::_SUCCESS)
				localCloudPoints.push_back(cp);
		}
		if (localCloudPoints.size() == 0)
			continue;
		// match local cloud points to new keyframe
		std::vector<SRef<CloudPoint>> notMatchedCloudPoints;
		matchLocalMapPoints(localCloudPoints, newKeyframe, notMatchedCloudPoints);
		// define invalid cloud points
		defineInvalidCloudPoints(newKeyframe, notMatchedCloudPoints);		
	}
	// prune
	int nbRemovedCP = m_mapManager->pointCloudPruning();
	int nbRemovedKf = m_mapManager->keyframePruning();
	LOG_DEBUG("Number of pruning cloud points / keyframes: {} / {}", nbRemovedCP, nbRemovedKf);
	return FrameworkReturnCode::_SUCCESS;
}

void SolARMapUpdate::matchLocalMapPoints(const std::vector<SRef<CloudPoint>>& localCloudPoints, SRef<Keyframe> newKeyframe, std::vector<SRef<CloudPoint>>& notMatchedCloudPoints)
{
	uint32_t imgWidth = newKeyframe->getView()->getWidth();
	uint32_t imgHeight = newKeyframe->getView()->getHeight();
	const std::map<uint32_t, uint32_t>& keyframeVisibilities = newKeyframe->getVisibility();
	std::set<uint32_t> seenCPIds;
	for (const auto& it : keyframeVisibilities)
		seenCPIds.insert(it.second);	
	// find other visiblities from local map
	std::vector<SRef<CloudPoint>> localMapUnseen;
	for (auto &itCP : localCloudPoints)
		if (itCP->isValid() && (seenCPIds.find(itCP->getId()) == seenCPIds.end()) && (cosineViewDirectionAngle(newKeyframe, itCP) > m_thresAngleViewDirection))
			localMapUnseen.push_back(itCP);
	//  projection points and filter point out of frame
	std::vector<SRef<CloudPoint>> localMapUnseenCandidates;
	std::vector< Point2Df > projected2DPtsCandidates;
	if (localMapUnseen.size() > 0) {
		std::vector< Point2Df > projected2DPts;
		m_projector->project(localMapUnseen, projected2DPts, newKeyframe->getPose());
		for (int idx = 0; idx < projected2DPts.size(); idx++)
			if ((projected2DPts[idx].getX() > 0) && (projected2DPts[idx].getX() < imgWidth) && (projected2DPts[idx].getY() > 0) && (projected2DPts[idx].getY() < imgHeight)) {
				projected2DPtsCandidates.push_back(projected2DPts[idx]);
				localMapUnseenCandidates.push_back(localMapUnseen[idx]);
			}
	}
	if (localMapUnseenCandidates.size() > 0) {
		// find local map matches
		std::vector<SRef<DescriptorBuffer>> desAllLocalMapUnseen;
		for (auto &it_cp : localMapUnseenCandidates) {
			desAllLocalMapUnseen.push_back(it_cp->getDescriptor());
		}
		std::vector<DescriptorMatch> allMatches;
		std::vector<bool> isMatchedCloudPoints(localMapUnseenCandidates.size(), false);
		m_matcher->match(projected2DPtsCandidates, desAllLocalMapUnseen, newKeyframe, allMatches);
		// update new visibilities of new frame
		int nbMatchesLocalMap(0);
		for (const auto &it_match : allMatches) {
			int idx_2d = it_match.getIndexInDescriptorB();
			int idx_3d = it_match.getIndexInDescriptorA();
			if ((!isMatchedCloudPoints[idx_3d]) && (keyframeVisibilities.find(idx_2d) == keyframeVisibilities.end())) {
				isMatchedCloudPoints[idx_3d] = true;
				newKeyframe->addVisibility(idx_2d, localMapUnseenCandidates[idx_3d]->getId());
				localMapUnseenCandidates[idx_3d]->updateConfidence(true);
				nbMatchesLocalMap++;
			}
		}
		LOG_DEBUG("Nb of matched local map: {}", nbMatchesLocalMap);
		for (int i = 0; i < isMatchedCloudPoints.size(); ++i)
			if (!isMatchedCloudPoints[i])
				notMatchedCloudPoints.push_back(localMapUnseenCandidates[i]);
		LOG_DEBUG("Nb of not matched local map: {}", notMatchedCloudPoints.size());
	}
}

void SolARMapUpdate::defineInvalidCloudPoints(SRef<datastructure::Keyframe> newKeyframe, std::vector<SRef<datastructure::CloudPoint>>& notMatchedCloudPoints)
{
	if (notMatchedCloudPoints.size() == 0)
		return;
	uint32_t imgWidth = newKeyframe->getView()->getWidth();
	uint32_t imgHeight = newKeyframe->getView()->getHeight();
	float updateWindows = RATIO_UPDATE_WINDOWS * imgWidth;
	float borderImage = RATIO_BORDER_IMAGE * imgWidth;
	std::vector<Point2Df> pts2dInliers;
	std::vector<Point3Df> pts3dInliers;
	const std::map<uint32_t, uint32_t>& keyframeVisibilities = newKeyframe->getVisibility();
	const std::vector<Keypoint>& keypoints = newKeyframe->getKeypoints();
	for (const auto& it : keyframeVisibilities) {
		SRef<CloudPoint> cloudPoint;
		if (m_pointCloud->getPoint(it.second, cloudPoint) == FrameworkReturnCode::_SUCCESS) {
			pts2dInliers.push_back(keypoints[it.first]);
			pts3dInliers.push_back(Point3Df(cloudPoint->getX(), cloudPoint->getY(), cloudPoint->getZ()));
		}
	}
	std::vector< Point2Df > projected2DPts;
	m_projector->project(notMatchedCloudPoints, projected2DPts, newKeyframe->getPose());
	for (int i = 0; i < notMatchedCloudPoints.size(); ++i) {
		if ((projected2DPts[i].getX() < borderImage) || (projected2DPts[i].getX() > imgWidth - borderImage) ||
			(projected2DPts[i].getY() < borderImage) || (projected2DPts[i].getY() > imgHeight - borderImage))
			continue;
		// handle occlusion
		std::vector<Point3Df> tracked3Dpts;
		for (int j = 0; j < pts2dInliers.size(); j++)
			if ((projected2DPts[i] - pts2dInliers[j]).norm() < updateWindows)
				tracked3Dpts.push_back(pts3dInliers[j]);
		if (tracked3Dpts.size() == 0) {
			notMatchedCloudPoints[i]->updateConfidence(false);
			if (notMatchedCloudPoints[i]->getConfidence() < m_thresConfidence) {
				notMatchedCloudPoints[i]->setInvalid();
			}
			continue;
		}
		auto itVis = notMatchedCloudPoints[i]->getVisibility().begin();
		uint32_t idKf = itVis->first;
		SRef<Keyframe> tmpKf;
		if (m_keyframeCollection->getKeyframe(idKf, tmpKf) != FrameworkReturnCode::_SUCCESS)
			continue;
		std::vector< Point2Df > tracked3DPtsProjected;
		m_projector->project(tracked3Dpts, tracked3DPtsProjected, tmpKf->getPose());
		const Keypoint& tmpKp = tmpKf->getKeypoint(itVis->second);
		for (const auto &it : tracked3DPtsProjected)
			if ((it - tmpKp).norm() < updateWindows) {
				notMatchedCloudPoints[i]->updateConfidence(false);
				if (notMatchedCloudPoints[i]->getConfidence() < m_thresConfidence) {
					notMatchedCloudPoints[i]->setInvalid();
				}
				break;
			}
	}
}

}
}
}

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

#include "SolARSLAMTracking.h"
#include "core/Log.h"


namespace xpcf = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::TOOLS::SolARSLAMTracking);
#define RATIO_BORDER_IMAGE 0.08
#define RATIO_UPDATE_WINDOWS 0.03

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {


SolARSLAMTracking::SolARSLAMTracking() :ConfigurableBase(xpcf::toUUID<SolARSLAMTracking>())
{
	addInterface<api::slam::ITracking>(this);
	declareInjectable<api::storage::IMapManager>(m_mapManager);
	declareInjectable<api::storage::IKeyframesManager>(m_keyframesManager);
	declareInjectable<api::storage::IPointCloudManager>(m_pointCloudManager);
	declareInjectable<api::features::IDescriptorMatcher>(m_matcher);
	declareInjectable<api::features::IDescriptorMatcherRegion>(m_matcherRegion);
	declareInjectable<api::features::IMatchesFilter>(m_matchesFilter);
	declareInjectable<api::solver::pose::I2D3DCorrespondencesFinder>(m_corr2D3DFinder);
	declareInjectable<api::solver::pose::I3DTransformFinderFrom2D3D>(m_pnp);
	declareInjectable<api::solver::pose::I3DTransformSACFinderFrom2D3D>(m_pnpRansac);
	declareInjectable<api::geom::IProject>(m_projector);
	declareInjectable<api::reloc::IKeyframeRetriever>(m_keyframeRetriever);
	declareInjectable<api::display::I2DOverlay>(m_overlay2DGreen, "Green");
	declareInjectable<api::display::I2DOverlay>(m_overlay2DRed, "Red");
	declareProperty("minWeightNeighbor", m_minWeightNeighbor);
	declareProperty("thresAngleViewDirection", m_thresAngleViewDirection);
	declareProperty("displayTrackedPoints", m_displayTrackedPoints);
    declareProperty("minTrackedPoints", m_minTrackedPoints);
    declareProperty("nbVisibilityAtLeast", m_nbVisibilityAtLeast);
    declareProperty("nbPassedFrameAtLeast", m_nbPassedFrameAtLeast);
    declareProperty("ratioCPRefKeyframe", m_ratioCPRefKeyframe);
	LOG_DEBUG("SolARSLAMTracking constructor");
}

xpcf::XPCFErrorCode SolARSLAMTracking::onConfigured()
{
	LOG_DEBUG("SolARSLAMTracking onConfigured");
	m_reprojErrorThreshold = m_mapManager->bindTo<xpcf::IConfigurable>()->getProperty("reprojErrorThreshold")->getFloatingValue();
	m_thresConfidence = m_mapManager->bindTo<xpcf::IConfigurable>()->getProperty("thresConfidence")->getFloatingValue();
	m_minNbInliers = m_pnpRansac->bindTo<xpcf::IConfigurable>()->getProperty("minNbInliers")->getIntegerValue();
	return xpcf::XPCFErrorCode::_SUCCESS;
}

void SolARSLAMTracking::setCameraParameters(const CamCalibration & intrinsicParams, const CamDistortion & distortionParams) {
	m_camMatrix = intrinsicParams;
	m_camDistortion = distortionParams;
	m_pnpRansac->setCameraParameters(m_camMatrix, m_camDistortion);
	m_pnp->setCameraParameters(m_camMatrix, m_camDistortion);
	m_projector->setCameraParameters(m_camMatrix, m_camDistortion);	
}

void SolARSLAMTracking::setNewKeyframe(const SRef<SolAR::datastructure::Keyframe> newKeyframe)
{
	std::unique_lock<std::mutex> lock(m_newKeyframeMutex);
	LOG_DEBUG("Set new keyframe id: {}", newKeyframe->getId());
	updateReferenceKeyframe(newKeyframe);	
	m_lastKeyframeId = newKeyframe->getId();
	m_nbPassedFrames = 0;
	setNeedNewKeyframe(false);
}

float cosineViewDirectionAngle(const SRef<Frame>& frame, const SRef<CloudPoint>& cp)
{
	const Transform3Df &pose = frame->getPose();
	Vector3f frameViewDir(pose(0, 3) - cp->getX(), pose(1, 3) - cp->getY(), pose(2, 3) - cp->getZ());
	const Vector3f& cpViewDir = cp->getViewDirection();
	return cpViewDir.dot(frameViewDir.normalized());
}

FrameworkReturnCode SolARSLAMTracking::process(const SRef<Frame> frame, SRef<Image> &displayImage)
{
	std::unique_lock<std::mutex> lock(m_newKeyframeMutex);
	// init image to display
	displayImage = frame->getView()->copy();
	LOG_DEBUG("Number keypoints: {}", frame->getKeypoints().size());
	if (frame->getKeypoints().size() == 0)
		return FrameworkReturnCode::_ERROR_;
	std::vector<DescriptorMatch> matches;
	Transform3Df framePose = frame->getPose();
    // lost tracking try to relocalize
	if (m_isLostTrack) {
		LOG_DEBUG("Tracking lost and try to relocalize");		
		// reloc
		std::vector < uint32_t> retKeyframesId;
		if (m_keyframeRetriever->retrieve(frame, retKeyframesId) == FrameworkReturnCode::_SUCCESS) {
			LOG_DEBUG("Successful relocalization. Update reference keyframe id: {}", retKeyframesId[0]);
			SRef<Keyframe> bestRetKeyframe;
			if (m_keyframesManager->getKeyframe(retKeyframesId[0], bestRetKeyframe) == FrameworkReturnCode::_SUCCESS) {
				updateReferenceKeyframe(bestRetKeyframe);
                m_nbPassedFrames = m_nbPassedFrameAtLeast - 1;
			}
			else
				return FrameworkReturnCode::_ERROR_;
		}
		else {
			LOG_DEBUG("Relocalization Failed");
			return FrameworkReturnCode::_ERROR_;
		}
	}	

	// set reference keyframe for new frame
	frame->setReferenceKeyframe(m_referenceKeyframe);	
    m_isLostTrack = true;
	// matching feature			
	m_matcher->match(m_referenceKeyframe->getDescriptors(), frame->getDescriptors(), matches);
	if (matches.size() < m_minNbInliers)
		return FrameworkReturnCode::_ERROR_;
    m_matchesFilter->filter(matches, matches, m_referenceKeyframe->getUndistortedKeypoints(), frame->getUndistortedKeypoints());
	float maxMatchDistance = -FLT_MAX;
	for (const auto &it : matches)
		maxMatchDistance = std::max(maxMatchDistance, it.getMatchingScore());

	// find 2D-3D point correspondences
	std::vector<Point2Df> pt2d;
	std::vector<Point3Df> pt3d;
	std::vector<CloudPoint> foundPoints;
	std::vector<DescriptorMatch> foundMatches;
	std::vector<DescriptorMatch> remainingMatches;
	std::vector < std::pair<uint32_t, SRef<CloudPoint>>> corres2D3D;
	m_corr2D3DFinder->find(m_referenceKeyframe, frame, matches, pt3d, pt2d, corres2D3D, foundMatches, remainingMatches);
	LOG_DEBUG("Nb of 2D-3D correspondences: {}", pt2d.size());	
	if (pt2d.size() == 0)
		return FrameworkReturnCode::_ERROR_;

	// find initial pose
	bool bFindPose = framePose.isApprox(Transform3Df::Identity());
	if (bFindPose) {
		std::vector<uint32_t> inliers;
		if (m_pnpRansac->estimate(pt2d, pt3d, inliers, framePose, m_lastPose) != FrameworkReturnCode::_SUCCESS)
			return FrameworkReturnCode::_ERROR_;
		LOG_DEBUG("Estimated pose: \n {}", framePose.matrix());		
		frame->setPose(framePose);	// Set the pose of the new frame
	}

	// Update map visibility of the current frame
	// And find more inliers of 2D-3D points for improving pose
	std::map<uint32_t, uint32_t> newMapVisibility;
	std::vector<Point2Df> pts2dInliers;
	std::vector<Point3Df> pts3dInliers;
	const std::vector<Keypoint> &keypoints = frame->getKeypoints();
	std::set<uint32_t> idxCPSeen;
	// Define inlier/outlier of 2D-3D correspondences found by the reference keyframe
	{
		std::vector< Point2Df > pt3DProj;		
		m_projector->project(pt3d, pt3DProj, framePose);
		for (int i = 0; i < pt3DProj.size(); ++i) {
			if ((pt2d[i] - pt3DProj[i]).norm() < m_reprojErrorThreshold) {
				idxCPSeen.insert(corres2D3D[i].second->getId());
				corres2D3D[i].second->updateConfidence(true);
				newMapVisibility[corres2D3D[i].first] = corres2D3D[i].second->getId();
				pts2dInliers.push_back(pt2d[i]);
				pts3dInliers.push_back(pt3d[i]);
			}
		}
		LOG_DEBUG("Inliers / Nb of correspondences: {} / {}", pts2dInliers.size(), pt3d.size());
	}

	std::vector<SRef<CloudPoint>> localMapUnseenCandidates;
	std::vector< Point2Df > projected2DPtsCandidates;
	std::vector<bool> checkLocalMapInOut;
	uint32_t imgWidth = frame->getView()->getWidth();
	uint32_t imgHeight = frame->getView()->getHeight();
	float updateWindows = RATIO_UPDATE_WINDOWS * imgWidth;
	float borderImage = RATIO_BORDER_IMAGE * imgWidth;
	{
		// find other visiblities from local map
		std::vector<SRef<CloudPoint>> localMapUnseen;
		for (auto &it_cp : m_localMap)
			if (it_cp->isValid() && (idxCPSeen.find(it_cp->getId()) == idxCPSeen.end()) && (cosineViewDirectionAngle(frame, it_cp) > m_thresAngleViewDirection))
				localMapUnseen.push_back(it_cp);

		//  projection points and filter point out of frame
		if (localMapUnseen.size() > 0) {
			std::vector< Point2Df > projected2DPts;
			m_projector->project(localMapUnseen, projected2DPts, frame->getPose());			
			for (int idx = 0; idx < projected2DPts.size(); idx++)
				if ((projected2DPts[idx].getX() > 0) && (projected2DPts[idx].getX() < imgWidth) && (projected2DPts[idx].getY() > 0) && (projected2DPts[idx].getY() < imgHeight)) {
					projected2DPtsCandidates.push_back(projected2DPts[idx]);
					localMapUnseenCandidates.push_back(localMapUnseen[idx]);
				}
		}
		LOG_DEBUG("Nb of filtered local map : {}", localMapUnseenCandidates.size());				
		if (localMapUnseenCandidates.size() > 0) {
			checkLocalMapInOut.resize(localMapUnseenCandidates.size(), false);
			// find more inlier matches
			std::vector<SRef<DescriptorBuffer>> desAllLocalMapUnseen;
			for (auto &it_cp : localMapUnseenCandidates) {
				desAllLocalMapUnseen.push_back(it_cp->getDescriptor());
			}
			std::vector<DescriptorMatch> allMatches;
			m_matcherRegion->match(projected2DPtsCandidates, desAllLocalMapUnseen, frame, allMatches, 0, maxMatchDistance);
			// find visibility of new frame
			int nbMatchesLocalMap(0);
			for (auto &it_match : allMatches) {
				int idx_2d = it_match.getIndexInDescriptorB();
				int idx_3d = it_match.getIndexInDescriptorA();
				checkLocalMapInOut[idx_3d] = true;
				if (newMapVisibility.find(idx_2d) == newMapVisibility.end()) {
					pts2dInliers.push_back(Point2Df(keypoints[idx_2d].getX(), keypoints[idx_2d].getY()));
					pts3dInliers.push_back(Point3Df(localMapUnseenCandidates[idx_3d]->getX(), localMapUnseenCandidates[idx_3d]->getY(), localMapUnseenCandidates[idx_3d]->getZ()));
					newMapVisibility[idx_2d] = localMapUnseenCandidates[idx_3d]->getId();
					localMapUnseenCandidates[idx_3d]->updateConfidence(true);
					nbMatchesLocalMap++;
				}
			}
			LOG_DEBUG("Nb of matched local map: {}", nbMatchesLocalMap);
		}
	}

	if (newMapVisibility.size() < m_minNbInliers)
		return FrameworkReturnCode::_ERROR_;

	// update map visibility of current frame
	frame->addVisibilities(newMapVisibility);
	LOG_DEBUG("Nb of map visibilities of current frame: {}", newMapVisibility.size());

	// pnp optimization
	if (bFindPose) {
		Transform3Df refinedPose;
		m_pnp->estimate(pts2dInliers, pts3dInliers, refinedPose, frame->getPose());
		frame->setPose(refinedPose);
	}
	LOG_DEBUG("Refined pose: \n {}", frame->getPose().matrix());
	m_lastPose = frame->getPose();	

    // tracking is good
    m_isLostTrack = false;
    m_nbPassedFrames++;
		
	// define invalid cloud points
	std::vector<Point2Df> localPtsInvalid;
	{		
		for (int i = 0; i < localMapUnseenCandidates.size(); ++i) {
			if (checkLocalMapInOut[i] || (projected2DPtsCandidates[i].getX() < borderImage) || (projected2DPtsCandidates[i].getX() > imgWidth - borderImage) ||
				(projected2DPtsCandidates[i].getY() < borderImage) || (projected2DPtsCandidates[i].getY() > imgHeight - borderImage))
				continue;
			// handle occlusion
			std::vector<Point3Df> tracked3Dpts;
			for (int j = 0; j < pts2dInliers.size(); j++)
				if ((projected2DPtsCandidates[i] - pts2dInliers[j]).norm() < updateWindows)
					tracked3Dpts.push_back(pts3dInliers[j]);
			if (tracked3Dpts.size() == 0)
				continue;
			auto itVis = localMapUnseenCandidates[i]->getVisibility().begin();
			uint32_t idKf = itVis->first;
			SRef<Keyframe> tmpKf;
			if (m_keyframesManager->getKeyframe(idKf, tmpKf) != FrameworkReturnCode::_SUCCESS)
				continue;
			std::vector< Point2Df > tracked3DPtsProjected;
			m_projector->project(tracked3Dpts, tracked3DPtsProjected, tmpKf->getPose());
			const Keypoint& tmpKp = tmpKf->getKeypoint(itVis->second);
			for (const auto &it : tracked3DPtsProjected)
				if ((it - tmpKp).norm() < updateWindows) {
					localPtsInvalid.push_back(projected2DPtsCandidates[i]);
					localMapUnseenCandidates[i]->updateConfidence(false);
					if (localMapUnseenCandidates[i]->getConfidence() < m_thresConfidence)
						localMapUnseenCandidates[i]->setInvalid();
					break;
				}
		}
	}
	LOG_DEBUG("Number of invalid local points: {}", localPtsInvalid.size());
	// display tracked points
	if (m_displayTrackedPoints) {
		m_overlay2DRed->drawCircles(localPtsInvalid, displayImage);
		m_overlay2DGreen->drawCircles(pts2dInliers, displayImage);
	}

	// compute number of shared cloud points between current frame and each keyframe
	std::map<uint32_t, int> kfCounterVis;
	for (const auto &it : newMapVisibility) {
		SRef<CloudPoint> cloudPoint;
		if (m_pointCloudManager->getPoint(it.second, cloudPoint) == FrameworkReturnCode::_SUCCESS) {
			const std::map<uint32_t, uint32_t> &cpKfVisibility = cloudPoint->getVisibility();
			for (auto const &it_kf : cpKfVisibility)
				kfCounterVis[it_kf.first]++;
		}
	}
	// find the keyframe that shares the most points
	uint32_t idBestKf = m_referenceKeyframe->getId();
	int nbMaxPts = kfCounterVis[idBestKf];
	for (const auto &it : kfCounterVis)
		if (it.second > nbMaxPts) {
			nbMaxPts = it.second;
			idBestKf = it.first;
		}
	// update the reference keyframe
	SRef<Keyframe> newRefKf;
	if ((idBestKf <= m_lastKeyframeId) && (idBestKf != m_referenceKeyframe->getId()) &&
		(m_keyframesManager->getKeyframe(idBestKf, newRefKf) == FrameworkReturnCode::_SUCCESS)) {
		LOG_DEBUG("Update new reference keyframe from {} to {}", m_referenceKeyframe->getId(), idBestKf);
		frame->setReferenceKeyframe(newRefKf);
		updateReferenceKeyframe(newRefKf);		
    }

    // Check need new keyframe
    if (!checkNeedNewKeyframe()) {
        LOG_DEBUG("Nb of passed frames: {}, ratio: {}", m_nbPassedFrames, (float)frame->getVisibility().size() / m_referenceKeyframe->getVisibility().size());
        // check conditions
        if ((m_nbPassedFrames > m_nbPassedFrameAtLeast) && (frame->getVisibility().size() > m_nbVisibilityAtLeast) &&
            ((frame->getVisibility().size() < m_ratioCPRefKeyframe * m_referenceKeyframe->getVisibility().size()) || (frame->getVisibility().size() < m_minTrackedPoints)))
        {
            LOG_DEBUG("Need new keyframe");
            setNeedNewKeyframe(true);
        }
    }

	return FrameworkReturnCode::_SUCCESS;
}

bool SolARSLAMTracking::checkNeedNewKeyframe()
{
    std::unique_lock<std::mutex> lock(m_needNewKeyframe);
    return m_isNeedNewKeyframe;
}

void SolARSLAMTracking::setNeedNewKeyframe(bool flag)
{
    std::unique_lock<std::mutex> lock(m_needNewKeyframe);
    m_isNeedNewKeyframe = flag;
}

void SolARSLAMTracking::updateReferenceKeyframe(const SRef<Keyframe> refKeyframe)
{
	// update new reference keyframe
	m_referenceKeyframe = refKeyframe;
	m_localMap.clear();
	// get local point cloud
	m_mapManager->getLocalPointCloud(m_referenceKeyframe, m_minWeightNeighbor, m_localMap);
	m_lastPose = m_referenceKeyframe->getPose();	
}

}
}
}

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

#include "SolARLoopClosureDetector.h"
#include "core/Log.h"


namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::TOOLS::SolARLoopClosureDetector);


namespace SolAR {
using namespace SolAR::datastructure;
using namespace SolAR::api;
using namespace SolAR::api::storage;
namespace MODULES {
namespace TOOLS {

SolARLoopClosureDetector::SolARLoopClosureDetector():ConfigurableBase(xpcf::toUUID<SolARLoopClosureDetector>())
{
    addInterface<SolAR::api::loop::ILoopClosureDetector>(this);
	declareInjectable<IKeyframesManager>(m_keyframesManager);
    declareInjectable<ICovisibilityGraphManager>(m_covisibilityGraphManager);
	declareInjectable<reloc::IKeyframeRetriever>(m_keyframeRetriever);
	declareInjectable<solver::pose::I3DTransformSACFinderFrom3D3D>(m_estimator3D);
	declareInjectable<features::IDescriptorMatcher>(m_matcher);
	declareInjectable<features::IMatchesFilter>(m_matchesFilter);
	declareInjectable<solver::pose::I3D3DCorrespondencesFinder>(m_corr3D3DFinder);
	declareInjectable<geom::I3DTransform>(m_transform3D);
	declareProperty("minNbInliers", m_NbMinInliers);
	LOG_DEBUG("SolARLoopClosureDetector constructor");
}

FrameworkReturnCode SolARLoopClosureDetector::detect(const SRef<Keyframe> queryKeyframe,
                                                     SRef<Keyframe> & detectedLoopKeyframe,
                                                     Transform3Df & sim3Transform,
                                                     std::vector<std::pair<uint32_t, uint32_t>>& duplicatedPointsIndices) const
{
	uint32_t queryKeyframeId = queryKeyframe->getId();
	std::vector<uint32_t> retKeyframesIndex;
	std::vector<uint32_t> candidatesId;
	// get candidate keyframes using BoW and covisibility graph
	m_keyframeRetriever->retrieve(SRef<Frame>(queryKeyframe), retKeyframesIndex);
	for (auto &it : retKeyframesIndex) {
		std::vector<uint32_t> paths;
        m_covisibilityGraphManager->getShortestPath(queryKeyframeId, it, paths);
		if (paths.size() > 3) {
			candidatesId.push_back(it);
			if (candidatesId.size() >= 3)
				break;
		}
	}
	std::vector<SRef<Keyframe>> candidateKeyframes;
	std::vector<Transform3Df> candidateKeyframePoses;
	for (auto &it : candidatesId) {
		SRef<Keyframe> keyframe;
		if (m_keyframesManager->getKeyframe(it, keyframe) != FrameworkReturnCode::_SUCCESS)
			continue;
		candidateKeyframes.push_back(keyframe);
		candidateKeyframePoses.push_back(keyframe->getPose());
	}
	
	// find best candidate loop detection
	Transform3Df bestTransform;
	SRef<Keyframe> bestDetectedLoopKeyframe;
	std::vector<SRef<CloudPoint>> bestFirstCloudPoints, bestSecondCloudPoints;
	std::vector<int> bestInliers;
	for (const auto &it : candidateKeyframes) {
		std::vector<DescriptorMatch> matches, foundMatches, remainingMatches;
		std::vector<SRef<CloudPoint>> firstCloudPoints, secondCloudPoints;
		m_matcher->match(queryKeyframe->getDescriptors(), it->getDescriptors(), matches);
		m_matchesFilter->filter(matches, matches, queryKeyframe->getUndistortedKeypoints(), it->getUndistortedKeypoints());
		m_corr3D3DFinder->find(queryKeyframe, it, matches, firstCloudPoints, secondCloudPoints, foundMatches, remainingMatches);
		std::vector<Point3Df> pts1, pts2;
		pts1.resize(firstCloudPoints.size());
		pts2.resize(firstCloudPoints.size());
		for (int i = 0; i < firstCloudPoints.size(); ++i) {
			pts1[i] = Point3Df(firstCloudPoints[i]->getX(), firstCloudPoints[i]->getY(), firstCloudPoints[i]->getZ());
			pts2[i] = Point3Df(secondCloudPoints[i]->getX(), secondCloudPoints[i]->getY(), secondCloudPoints[i]->getZ());
		}
		Transform3Df pose;
		std::vector<int> inliers;
		if (m_estimator3D->estimate(queryKeyframe, it, foundMatches, pts1, pts2, pose, inliers) == FrameworkReturnCode::_SUCCESS) {
			if (inliers.size() > bestInliers.size()) {
				bestTransform = pose;
				bestDetectedLoopKeyframe = it;
				bestFirstCloudPoints.swap(firstCloudPoints);
				bestSecondCloudPoints.swap(secondCloudPoints);
				bestInliers.swap(inliers);
			}
		}
	}
	if (bestInliers.size() < m_NbMinInliers)
		return FrameworkReturnCode::_ERROR_;

	detectedLoopKeyframe = bestDetectedLoopKeyframe;
	sim3Transform = bestTransform;

	for (const auto &it : bestInliers)
		duplicatedPointsIndices.push_back(std::make_pair(bestFirstCloudPoints[it]->getId(), bestSecondCloudPoints[it]->getId()));

	return FrameworkReturnCode::_SUCCESS;
}

}
}
}

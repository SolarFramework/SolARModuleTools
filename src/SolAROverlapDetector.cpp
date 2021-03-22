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

#include "SolAROverlapDetector.h"
#include "core/Log.h"


namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::TOOLS::SolAROverlapDetector);


namespace SolAR {
using namespace datastructure;
using namespace api;
using namespace api::storage;
using namespace api::reloc;
namespace MODULES {
namespace TOOLS {

SolAROverlapDetector::SolAROverlapDetector():ConfigurableBase(xpcf::toUUID<SolAROverlapDetector>())
{
    addInterface<api::loop::IOverlapDetector>(this);
	declareInjectable<solver::pose::I3DTransformSACFinderFrom3D3D>(m_estimator3D);
	declareInjectable<features::IDescriptorMatcher>(m_matcher);
	declareInjectable<features::IMatchesFilter>(m_matchesFilter);
	declareInjectable<solver::pose::I3D3DCorrespondencesFinder>(m_corr3D3DFinder);
	declareInjectable<geom::I3DTransform>(m_transform3D);
	declareProperty("minNbInliers", m_NbMinInliers);
}

void SolAROverlapDetector::setCameraParameters(const CamCalibration & intrinsicParams, const CamDistortion & distortionParams) {
	m_estimator3D->setCameraParameters(intrinsicParams, distortionParams);
}

FrameworkReturnCode SolAROverlapDetector::detect(const SRef<api::solver::map::IMapper> globalMap,
                                                 const SRef<api::solver::map::IMapper> floatingMap,
                                                 Transform3Df & sim3Transform,
                                                 std::vector<std::pair<uint32_t, uint32_t>>& cpOverlapIndices) const
{
	SRef<IPointCloudManager> floatingPointCloudManager, globalPointCloudManager;
	SRef<IKeyframesManager> floatingKeyframesManager, globalKeyframesManager;
	SRef<IKeyframeRetriever> globalKeyframeRetriever;
	std::vector<SRef<Keyframe>> allFloatingKeyframes;
	std::vector<SRef<CloudPoint>> floatingPointCloud, globalPointCloud;

	// get floating mapper information
	floatingMap->getKeyframesManager(floatingKeyframesManager);
	floatingMap->getPointCloudManager(floatingPointCloudManager);
	floatingPointCloudManager->getAllPoints(floatingPointCloud);
	floatingKeyframesManager->getAllKeyframes(allFloatingKeyframes);

	// get global mapper information
	globalMap->getPointCloudManager(globalPointCloudManager);
	globalMap->getKeyframeRetriever(globalKeyframeRetriever);
	globalMap->getKeyframesManager(globalKeyframesManager);
	globalPointCloudManager->getAllPoints(globalPointCloud);

	// find 3D-3D correspondences
	std::set<uint32_t> foundIdFloatCPs, foundIdGlobalCPs;
	std::vector<Point3Df> ptsFloating, ptsGlobal;
	std::vector<std::pair<uint32_t, uint32_t>> duplicatedPointsIndices;
	for (const auto & queryKeyframe : allFloatingKeyframes) {
		std::vector<uint32_t> candidatesId;
		// get candidate keyframes using BoW and covisibility graph
        globalKeyframeRetriever->retrieve(SRef<Frame>(queryKeyframe), candidatesId);
		if (candidatesId.size() == 0)
			continue;
		std::vector<SRef<Keyframe>> candidateKeyframes;
		for (auto &it : candidatesId) {
			SRef<Keyframe> keyframe;
			if (globalKeyframesManager->getKeyframe(it, keyframe) == FrameworkReturnCode::_SUCCESS)
				candidateKeyframes.push_back(keyframe);
			if (candidateKeyframes.size() > 2)
				break;
		}
		// find best candidate loop detection
		Transform3Df bestTransform;
		SRef<Keyframe> bestDetectedLoopKeyframe;
		std::vector<SRef<CloudPoint>> bestFirstCloudPoints, bestSecondCloudPoints;
		std::vector<int> bestInliers;
		for (const auto &it : candidateKeyframes) {
			std::vector<DescriptorMatch> matches, foundMatches;
			std::vector<SRef<CloudPoint>> floatingCloudPoints, globalCloudPoints;
			std::vector<uint32_t>floatingCloudPointsIndices, globalCloudPointsIndices;
			m_matcher->match(queryKeyframe->getDescriptors(), it->getDescriptors(), matches);
			m_matchesFilter->filter(matches, matches, queryKeyframe->getKeypoints(), it->getKeypoints());
			m_corr3D3DFinder->find(queryKeyframe, it, matches, floatingCloudPointsIndices, globalCloudPointsIndices, foundMatches);

			floatingPointCloudManager->getPoints(floatingCloudPointsIndices, floatingCloudPoints);
			globalPointCloudManager->getPoints(globalCloudPointsIndices, globalCloudPoints);
			if (floatingCloudPoints.size() != globalCloudPoints.size())
				continue;
			std::vector<Point3Df> pts1, pts2;
			pts1.resize(floatingCloudPoints.size());
			pts2.resize(floatingCloudPoints.size());
			for (int i = 0; i < floatingCloudPoints.size(); ++i) {
				pts1[i] = Point3Df(floatingCloudPoints[i]->getX(), floatingCloudPoints[i]->getY(), floatingCloudPoints[i]->getZ());
				pts2[i] = Point3Df(globalCloudPoints[i]->getX(), globalCloudPoints[i]->getY(), globalCloudPoints[i]->getZ());
			}
			Transform3Df pose;
			std::vector<int> inliers;

			if (m_estimator3D->estimate(queryKeyframe, it, foundMatches, pts1, pts2, pose, inliers) == FrameworkReturnCode::_SUCCESS) {
				if (inliers.size() > bestInliers.size()) {
					bestTransform = pose;
					bestDetectedLoopKeyframe = it;
					bestFirstCloudPoints.swap(floatingCloudPoints);
					bestSecondCloudPoints.swap(globalCloudPoints);
					bestInliers.swap(inliers);
				}
			}

		}
		// get 3D-3D correspondences
		if (bestInliers.size() >= m_NbMinInliers) {
			for (const auto &it : bestInliers) {
				uint32_t idFloatCP = bestFirstCloudPoints[it]->getId();
				uint32_t idGlobalCP = bestSecondCloudPoints[it]->getId();
				if ((foundIdFloatCPs.find(idFloatCP) == foundIdFloatCPs.end()) && (foundIdGlobalCPs.find(idGlobalCP) == foundIdGlobalCPs.end())) {
					foundIdFloatCPs.insert(idFloatCP);
					foundIdGlobalCPs.insert(idGlobalCP);
					duplicatedPointsIndices.push_back(std::make_pair(idFloatCP, idGlobalCP));
					ptsFloating.push_back(std::move(Point3Df(bestFirstCloudPoints[it]->getX(), bestFirstCloudPoints[it]->getY(), bestFirstCloudPoints[it]->getZ())));
					ptsGlobal.push_back(std::move(Point3Df(bestSecondCloudPoints[it]->getX(), bestSecondCloudPoints[it]->getY(), bestSecondCloudPoints[it]->getZ())));
				}				
			}				
		}
	}

	std::vector<int> inliers;
	if (m_estimator3D->estimate(ptsFloating, ptsGlobal, sim3Transform, inliers) == FrameworkReturnCode::_SUCCESS) {
		if (inliers.size() > m_NbMinInliers) {
			for (const auto &it : inliers)
				cpOverlapIndices.push_back(duplicatedPointsIndices[it]);
			return FrameworkReturnCode::_SUCCESS;
		}
	}
	return FrameworkReturnCode::_ERROR_;
}

FrameworkReturnCode SolAROverlapDetector::detect(const SRef<api::solver::map::IMapper> globalMap,
                                                const SRef<api::solver::map::IMapper> floatingMap,
												std::vector<Transform3Df> &sim3Transform,
												std::vector<std::pair<uint32_t, uint32_t>>&overlapIndices,
                                                std::vector<double>&scores) const
{	
	SRef<IPointCloudManager> floatingPointCloudManager, globalPointCloudManager;
	SRef<IKeyframesManager> floatingKeyframesManager, globalKeyframesManager;
	SRef<IKeyframeRetriever> globalKeyframeRetriever;
	std::vector<SRef<Keyframe>> allFloatingKeyframes;
	std::vector<SRef<CloudPoint>> floatingPointCloud, globalPointCloud;

	// get floating mapper information
	floatingMap->getKeyframesManager(floatingKeyframesManager);
	floatingMap->getPointCloudManager(floatingPointCloudManager);
	floatingPointCloudManager->getAllPoints(floatingPointCloud);
	floatingKeyframesManager->getAllKeyframes(allFloatingKeyframes);	

	// get global mapper information
	globalMap->getPointCloudManager(globalPointCloudManager);
	globalMap->getKeyframeRetriever(globalKeyframeRetriever);
	globalMap->getKeyframesManager(globalKeyframesManager);
	globalPointCloudManager->getAllPoints(globalPointCloud);

	for (const auto & queryKeyframe : allFloatingKeyframes) {
		std::vector<uint32_t> candidatesId;
		// get candidate keyframes using BoW and covisibility graph
		globalKeyframeRetriever->retrieve(SRef<Frame>(queryKeyframe), candidatesId);
		if (candidatesId.size() == 0)
			continue;
		std::vector<SRef<Keyframe>> candidateKeyframes;
		for (auto &it : candidatesId) {
			SRef<Keyframe> keyframe;
			if (globalKeyframesManager->getKeyframe(it, keyframe) == FrameworkReturnCode::_SUCCESS)
				candidateKeyframes.push_back(keyframe);
			if (candidateKeyframes.size() > 2)
				break;
		}
		// find best candidate loop detection
		Transform3Df bestTransform;
		SRef<Keyframe> bestDetectedLoopKeyframe;
		std::vector<SRef<CloudPoint>> bestFirstCloudPoints, bestSecondCloudPoints;
		std::vector<int> bestInliers;
		for (const auto &it : candidateKeyframes) {
			std::vector<DescriptorMatch> matches, foundMatches;
			std::vector<SRef<CloudPoint>> floatingCloudPoints, globalCloudPoints;
			std::vector<uint32_t>floatingCloudPointsIndices, globalCloudPointsIndices;
			m_matcher->match(queryKeyframe->getDescriptors(), it->getDescriptors(), matches);
			m_matchesFilter->filter(matches, matches, queryKeyframe->getKeypoints(), it->getKeypoints());
			m_corr3D3DFinder->find(queryKeyframe, it, matches, floatingCloudPointsIndices, globalCloudPointsIndices, foundMatches);

			floatingPointCloudManager->getPoints(floatingCloudPointsIndices, floatingCloudPoints);
			globalPointCloudManager->getPoints(globalCloudPointsIndices, globalCloudPoints);

			std::vector<Point3Df> pts1, pts2;
			pts1.resize(floatingCloudPoints.size());
			pts2.resize(floatingCloudPoints.size());
			for (int i = 0; i < floatingCloudPoints.size(); ++i) {
				pts1[i] = Point3Df(floatingCloudPoints[i]->getX(), floatingCloudPoints[i]->getY(), floatingCloudPoints[i]->getZ());
				pts2[i] = Point3Df(globalCloudPoints[i]->getX(), globalCloudPoints[i]->getY(), globalCloudPoints[i]->getZ());
			}
			Transform3Df pose;
			std::vector<int> inliers;

			if (m_estimator3D->estimate(pts1, pts2, pose, inliers) == FrameworkReturnCode::_SUCCESS) {
				if (inliers.size() > bestInliers.size()) {
					bestTransform = pose;
					bestDetectedLoopKeyframe = it;
					bestFirstCloudPoints.swap(floatingCloudPoints);
					bestSecondCloudPoints.swap(globalCloudPoints);
					bestInliers.swap(inliers);
				}
			}

		}
		if (bestInliers.size() >= m_NbMinInliers) {
			sim3Transform.push_back(bestTransform);
			uint32_t idxGlobalKeyframe = bestDetectedLoopKeyframe->getId();
			uint32_t idxFloatingKeyframe = queryKeyframe->getId();
			overlapIndices.push_back(std::make_pair(idxFloatingKeyframe, idxGlobalKeyframe));
			scores.push_back(double(bestInliers.size()));
			LOG_DEBUG("#OVERLAP detection between: floatingKeyframe {} and globalKeyframe {} with {} inliers", idxFloatingKeyframe, idxGlobalKeyframe,bestInliers.size());

		}
	}
	if (sim3Transform.size() > 0)
		return FrameworkReturnCode::_SUCCESS;
	else
		return FrameworkReturnCode::_ERROR_;
}

}
}
}

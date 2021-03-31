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

#include "SolARSLAMBootstrapper.h"
#include "core/Log.h"


namespace xpcf = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::TOOLS::SolARSLAMBootstrapper);


namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {


SolARSLAMBootstrapper::SolARSLAMBootstrapper() :ConfigurableBase(xpcf::toUUID<SolARSLAMBootstrapper>())
{
    addInterface<SolAR::api::slam::IBootstrapper>(this);
    declareInjectable<SolAR::api::solver::map::IMapper>(m_mapper);
    declareInjectable<SolAR::api::features::IKeypointDetector>(m_keypointsDetector);
    declareInjectable<SolAR::api::features::IDescriptorsExtractor>(m_descriptorExtractor);
    declareInjectable<SolAR::api::features::IDescriptorMatcher>(m_matcher);
    declareInjectable<SolAR::api::features::IMatchesFilter>(m_matchesFilter);
    declareInjectable<SolAR::api::solver::map::ITriangulator>(m_triangulator);
    declareInjectable<SolAR::api::solver::map::IMapFilter>(m_mapFilter);
    declareInjectable<SolAR::api::solver::map::IKeyframeSelector>(m_keyframeSelector);
    declareInjectable<SolAR::api::solver::pose::I3DTransformFinderFrom2D2D>(m_poseFinderFrom2D2D);
    declareInjectable<SolAR::api::geom::IUndistortPoints>(m_undistortPoints);
    declareInjectable<SolAR::api::display::IMatchesOverlay>(m_matchesOverlay);
	declareProperty("hasPose", m_hasPose);
	declareProperty("nbMinInitPointCloud", m_nbMinInitPointCloud);
	declareProperty("angleThres", m_angleThres);	
}

xpcf::XPCFErrorCode SolARSLAMBootstrapper::onConfigured()
{
	LOG_DEBUG("SolARSLAMBootstrapper onConfigured");
	m_ratioDistanceIsKeyframe = m_keyframeSelector->bindTo<xpcf::IConfigurable>()->getProperty("minMeanDistanceIsKeyframe")->getFloatingValue();
	return xpcf::XPCFErrorCode::_SUCCESS;
}

void SolARSLAMBootstrapper::setCameraParameters(const CamCalibration & intrinsicParams, const CamDistortion & distortionParams) {
	m_camMatrix = intrinsicParams;
	m_camDistortion = distortionParams;
	m_triangulator->setCameraParameters(m_camMatrix, m_camDistortion);
	m_poseFinderFrom2D2D->setCameraParameters(m_camMatrix, m_camDistortion);
    m_undistortPoints->setCameraParameters(m_camMatrix, m_camDistortion);
}

inline float angleCamDistance(const Transform3Df & pose1, const Transform3Df & pose2) {
	return std::acos(pose1(0, 2) * pose2(0, 2) + pose1(1, 2) * pose2(1, 2) + pose1(2, 2) * pose2(2, 2));
}

FrameworkReturnCode SolARSLAMBootstrapper::process(const SRef<Image> image, SRef<Image> &view, const Transform3Df &pose)
{
	Transform3Df						poseFrame;
	std::vector<Keypoint>				keypoints, undistortedKeypoints;
	SRef<DescriptorBuffer>				descriptors;
	std::vector<DescriptorMatch>		matches;
	SRef<Frame>							frame2;	
	std::vector<SRef<CloudPoint>>		cloud, filteredCloud;	
	view = image->copy();
	if (m_hasPose) {
		if (pose.isApprox(Transform3Df::Identity()))
			return FrameworkReturnCode::_ERROR_;
		else
			poseFrame = pose;
	}
	else
		poseFrame = Transform3Df::Identity();			

	// keypoint detection
	m_keypointsDetector->detect(image, keypoints);
	// undistort keypoints
	m_undistortPoints->undistort(keypoints, undistortedKeypoints);
	// feature extraction
	m_descriptorExtractor->extract(image, keypoints, descriptors);
	if (!m_initKeyframe1) {
		// init first keyframe
		m_initKeyframe1 = true;		
		m_keyframe1 = xpcf::utils::make_shared<Keyframe>(keypoints, undistortedKeypoints, descriptors, image, poseFrame);
	}
	else {
		frame2 = xpcf::utils::make_shared<Frame>(keypoints, undistortedKeypoints, descriptors, image, m_keyframe1, poseFrame);		
		// matching
		m_matcher->match(m_keyframe1->getDescriptors(), descriptors, matches);
		//m_matcher->matchInRegion(m_keyframe1, frame2, matches, image->getWidth() * (m_ratioDistanceIsKeyframe + 0.01));
		m_matchesFilter->filter(matches, matches, m_keyframe1->getKeypoints(), frame2->getKeypoints());
		if (matches.size() > 0) {
			m_matchesOverlay->draw(image, view, m_keyframe1->getKeypoints(), frame2->getKeypoints(), matches);
		}
		if (matches.size() < m_nbMinInitPointCloud) {
			m_keyframe1 = xpcf::utils::make_shared<Keyframe>(frame2);
		}
		else if (m_keyframeSelector->select(frame2, matches)) {
			// Find pose of the second keyframe if not has pose
			if (!m_hasPose) {
				m_poseFinderFrom2D2D->estimate(m_keyframe1->getKeypoints(), frame2->getKeypoints(), m_keyframe1->getPose(), poseFrame, matches);				
				frame2->setPose(poseFrame);
			}
			if (angleCamDistance(m_keyframe1->getPose(), frame2->getPose()) > m_angleThres)
				return FrameworkReturnCode::_ERROR_;
			// Triangulate
			m_triangulator->triangulate(m_keyframe1->getKeypoints(), frame2->getKeypoints(), m_keyframe1->getDescriptors(), frame2->getDescriptors(), matches,
				std::make_pair(0, 1), m_keyframe1->getPose(), frame2->getPose(), cloud);
			// Filter cloud points
			m_mapFilter->filter(m_keyframe1->getPose(), frame2->getPose(), cloud, filteredCloud);
			if (filteredCloud.size() > m_nbMinInitPointCloud) {
                SRef<SolAR::api::storage::IKeyframesManager>keyframesManager;
                SRef<SolAR::api::reloc::IKeyframeRetriever> keyframeRetriever;
				m_mapper->getKeyframeRetriever(keyframeRetriever);
				m_mapper->getKeyframesManager(keyframesManager);
				// add keyframes to keyframes manager
				keyframesManager->addKeyframe(m_keyframe1);
				m_keyframe2 = xpcf::utils::make_shared<Keyframe>(frame2);
				keyframesManager->addKeyframe(m_keyframe2);
				// add intial point cloud to point cloud manager and update visibility map and update covisibility graph
				for (auto const &it : filteredCloud)
					m_mapper->addCloudPoint(it);
				// add keyframes to retriever
				keyframeRetriever->addKeyframe(m_keyframe1);
				keyframeRetriever->addKeyframe(m_keyframe2);
				return FrameworkReturnCode::_SUCCESS;
			}
			else {
				m_keyframe1 = xpcf::utils::make_shared<Keyframe>(frame2);
				if (!m_hasPose)
					m_keyframe1->setPose(Transform3Df::Identity());
			}
		}
	}
	return FrameworkReturnCode::_ERROR_;
}

}
}
}

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
	addInterface<api::slam::IBootstrapper>(this);
	declareInjectable<api::storage::IMapManager>(m_mapManager);
	declareInjectable<api::features::IDescriptorMatcher>(m_matcher);
	declareInjectable<api::features::IMatchesFilter>(m_matchesFilter);
	declareInjectable<api::solver::map::ITriangulator>(m_triangulator);
	declareInjectable<api::solver::map::IMapFilter>(m_mapFilter);
	declareInjectable<api::solver::map::IKeyframeSelector>(m_keyframeSelector);
	declareInjectable<api::solver::pose::I3DTransformFinderFrom2D2D>(m_poseFinderFrom2D2D);
	declareInjectable<api::display::IMatchesOverlay>(m_matchesOverlay);
	declareProperty("hasPose", m_hasPose);
	declareProperty("nbMinInitPointCloud", m_nbMinInitPointCloud);
	declareProperty("angleThres", m_angleThres);
	LOG_DEBUG("SolARSLAMBootstrapper constructor");	
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
}

inline float angleCamDistance(const Transform3Df & pose1, const Transform3Df & pose2) {
	return std::acos(pose1(0, 2) * pose2(0, 2) + pose1(1, 2) * pose2(1, 2) + pose1(2, 2) * pose2(2, 2));
}

FrameworkReturnCode SolARSLAMBootstrapper::process(const SRef<Frame>& frame, SRef<Image> &view)
{
    view = frame->getView()->copy();
	if (m_hasPose) {
		if (frame->getPose().isApprox(Transform3Df::Identity()))
			return FrameworkReturnCode::_ERROR_;
	}
	else
		frame->setPose(Transform3Df::Identity());
	
	if (!m_initKeyframe1) {
		// init first keyframe
		m_initKeyframe1 = true;		
        m_keyframe1 = xpcf::utils::make_shared<Keyframe>(frame);
	}
	else {
		// set reference keyframe
		frame->setReferenceKeyframe(m_keyframe1);
		// matching
		std::vector<DescriptorMatch> matches;
		m_matcher->match(m_keyframe1->getDescriptors(), frame->getDescriptors(), matches);
		m_matchesFilter->filter(matches, matches, m_keyframe1->getKeypoints(), frame->getKeypoints());
		if (matches.size() > 0) {
            m_matchesOverlay->draw(frame->getView(), view, m_keyframe1->getKeypoints(), frame->getKeypoints(), matches);
		}
		if (matches.size() < m_nbMinInitPointCloud) {
			m_keyframe1 = xpcf::utils::make_shared<Keyframe>(frame);
		}
		else if (m_keyframeSelector->select(frame, matches)) {
			// Find pose of the second keyframe if not has pose
			if (!m_hasPose) {
				Transform3Df pose;
				m_poseFinderFrom2D2D->estimate(m_keyframe1->getKeypoints(), frame->getKeypoints(), m_keyframe1->getPose(), pose, matches);				
				frame->setPose(pose);
			}
			if (angleCamDistance(m_keyframe1->getPose(), frame->getPose()) > m_angleThres)
				return FrameworkReturnCode::_ERROR_;
			// Triangulate
			std::vector<SRef<CloudPoint>> cloud, filteredCloud;
			m_triangulator->triangulate(m_keyframe1->getKeypoints(), frame->getKeypoints(), m_keyframe1->getDescriptors(), frame->getDescriptors(), matches,
				std::make_pair(0, 1), m_keyframe1->getPose(), frame->getPose(), cloud);
			// Filter cloud points
			m_mapFilter->filter(m_keyframe1->getPose(), frame->getPose(), cloud, filteredCloud);
			if (filteredCloud.size() > m_nbMinInitPointCloud) {
				// add keyframes to map manager
				m_mapManager->addKeyframe(m_keyframe1);
				m_keyframe2 = xpcf::utils::make_shared<Keyframe>(frame);
				m_mapManager->addKeyframe(m_keyframe2);
				// add intial point cloud to point cloud manager and update visibility map and update covisibility graph
				for (auto const &it : filteredCloud)
					m_mapManager->addCloudPoint(it);
				return FrameworkReturnCode::_SUCCESS;
			}
			else {
				m_keyframe1 = xpcf::utils::make_shared<Keyframe>(frame);
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

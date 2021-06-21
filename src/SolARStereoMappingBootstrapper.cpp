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

#include "SolARStereoMappingBootstrapper.h"
#include "core/Log.h"
#include <thread>

namespace xpcf = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::TOOLS::SolARStereoMappingBootstrapper);

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {

SolARStereoMappingBootstrapper::SolARStereoMappingBootstrapper() :ConfigurableBase(xpcf::toUUID<SolARStereoMappingBootstrapper>())
{
	addInterface<api::stereo::IStereoMappingBootstrapper>(this);
	declareInjectable<api::storage::IMapManager>(m_mapManager);
	declareInjectable<api::stereo::IStereoDepthEstimation>(m_stereoDepthEstimator);
	declareInjectable<api::display::I2DOverlay>(m_overlay2DGreen, "Green");
	declareInjectable<api::display::I2DOverlay>(m_overlay2DRed, "Red");
	declareProperty("nbMinInitPointCloud", m_nbMinInitPointCloud);
	LOG_DEBUG("SolARStereoMappingBootstrapper constructor");
}

SolARStereoMappingBootstrapper::~SolARStereoMappingBootstrapper()
{
	LOG_DEBUG("SolARStereoMappingBootstrapper destructor");
}

void SolARStereoMappingBootstrapper::setCameraParameters(const SolAR::datastructure::CamCalibration & intrinsicParams)
{
	m_intrinsicParams = intrinsicParams;
}

FrameworkReturnCode SolARStereoMappingBootstrapper::process(const SRef<SolAR::datastructure::Frame>& frame, SRef<SolAR::datastructure::Image>& view)
{
	view = frame->getView()->copy();
	std::vector<SRef<CloudPoint>> cloudPoints;
	m_stereoDepthEstimator->reprojectToCloudPoints(frame, m_intrinsicParams, cloudPoints);
	LOG_DEBUG("Number of estimated cloud points: {}", cloudPoints.size());
	// draw to display
	const std::vector<Keypoint>& undistortedKeypoints = frame->getUndistortedKeypoints();
	const std::vector<Keypoint>& keypoints = frame->getKeypoints();
	for (int i = 0; i < undistortedKeypoints.size(); ++i)
		if (undistortedKeypoints[i].getDepth() > 0)
			m_overlay2DGreen->drawCircle(keypoints[i], view);
		else
			m_overlay2DRed->drawCircle(keypoints[i], view);
	// check bootstrap is ok
	if (cloudPoints.size() > m_nbMinInitPointCloud) {
		// add keyframes to map manager
		SRef<Keyframe> keyframe1 = xpcf::utils::make_shared<Keyframe>(frame);
		m_mapManager->addKeyframe(keyframe1);
		// add intial point cloud to point cloud manager and update visibility map and update covisibility graph
		for (auto const &it : cloudPoints) {
			m_mapManager->addCloudPoint(it);
		}
		return FrameworkReturnCode::_SUCCESS;
	}
	else
		return FrameworkReturnCode::_ERROR_;
}


}
}
}

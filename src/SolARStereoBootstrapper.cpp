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

#include "SolARStereoBootstrapper.h"
#include "core/Log.h"
#include <thread>

namespace xpcf = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::TOOLS::SolARStereoBootstrapper);

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {

SolARStereoBootstrapper::SolARStereoBootstrapper() :ConfigurableBase(xpcf::toUUID<SolARStereoBootstrapper>())
{
    addInterface<api::slam::IBootstrapperStereo>(this);
	declareInjectable<api::storage::IMapManager>(m_mapManager);
    declareInjectable<api::geom::IReprojectionStereo>(m_stereoReprojector);
	declareInjectable<api::display::I2DOverlay>(m_overlay2DGreen, "Green");
	declareInjectable<api::display::I2DOverlay>(m_overlay2DRed, "Red");
	declareProperty("nbMinInitPointCloud", m_nbMinInitPointCloud);
    LOG_DEBUG("SolARStereoBootstrapper constructor");
}

SolARStereoBootstrapper::~SolARStereoBootstrapper()
{
    LOG_DEBUG("SolARStereoBootstrapper destructor");
}

void SolARStereoBootstrapper::setCameraParameters(const SolAR::datastructure::CamCalibration & intrinsicParams)
{
	m_intrinsicParams = intrinsicParams;
}

FrameworkReturnCode SolARStereoBootstrapper::process(const SRef<SolAR::datastructure::Frame>& frame, SRef<SolAR::datastructure::Image>& view)
{
	view = frame->getView()->copy();
	std::vector<SRef<CloudPoint>> cloudPoints;
    m_stereoReprojector->reprojectToCloudPoints(frame, m_intrinsicParams, cloudPoints);
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

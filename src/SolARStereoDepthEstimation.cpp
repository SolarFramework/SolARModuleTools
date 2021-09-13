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

#include "SolARStereoDepthEstimation.h"
#include "core/Log.h"


namespace xpcf = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::TOOLS::SolARStereoDepthEstimation);

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {

SolARStereoDepthEstimation::SolARStereoDepthEstimation() :ConfigurableBase(xpcf::toUUID<SolARStereoDepthEstimation>())
{
	declareInterface<api::geom::IDepthEstimation>(this);
	declareProperty("ratioNear", m_ratioNear);
	declareProperty("ratioFar", m_ratioFar);
	LOG_DEBUG("SolARStereoDepthEstimation constructor");
}

SolARStereoDepthEstimation::~SolARStereoDepthEstimation()
{
	LOG_DEBUG("SolARStereoDepthEstimation destructor");
}

FrameworkReturnCode SolARStereoDepthEstimation::estimate(std::vector<SolAR::datastructure::Keypoint>& rectKeypoints1,
                                                         std::vector<SolAR::datastructure::Keypoint>& rectKeypoints2,
                                                         const std::vector<SolAR::datastructure::DescriptorMatch>& matches,
                                                         float focal,
                                                         float baseline,
                                                         SolAR::datastructure::StereoType type)
{
	// disparity min max
	float dMin = focal * m_ratioFar;
	float dMax = focal * m_ratioNear;
	// compute fb
	float fb = focal * baseline;
	// triangulation by disparity
	for (const auto& match : matches) {
        Keypoint& kp1 = rectKeypoints1[match.getIndexInDescriptorA()];
        Keypoint& kp2 = rectKeypoints2[match.getIndexInDescriptorB()];
		int disparity;
		if (type == StereoType::Vertical)
			disparity = kp2.getY() - kp1.getY();
		else
			disparity = kp1.getX() - kp2.getX();
		if ((disparity > dMin) && (disparity < dMax)) {
			kp1.setDepth(fb / disparity);
			kp2.setDepth(fb / disparity);
		}
	}
	return FrameworkReturnCode::_SUCCESS;
}

}
}
}

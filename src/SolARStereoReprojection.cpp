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

#include "SolARStereoReprojection.h"
#include "core/Log.h"


namespace xpcf = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::TOOLS::SolARStereoReprojection);

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {

SolARStereoReprojection::SolARStereoReprojection() : base::geom::AReprojectionStereo(xpcf::toMap<SolARStereoReprojection>())
{
    LOG_DEBUG("SolARStereoReprojection constructor");
}

SolARStereoReprojection::~SolARStereoReprojection()
{
    LOG_DEBUG("SolARStereoReprojection destructor");
}

FrameworkReturnCode SolARStereoReprojection::reprojectToUnrectification(const std::vector<SolAR::datastructure::Keypoint>& rectifiedKeypoints,
                                                                        const SolAR::datastructure::RectificationParameters & rectParams,
                                                                        std::vector<SolAR::datastructure::Keypoint>& unrectifiedKeypoints)
{
	Maths::Matrix3f rot = rectParams.rotation;
	Maths::Matrix<float, 3, 4> proj = rectParams.projection;
	for (int i = 0; i < rectifiedKeypoints.size(); ++i)
		if (rectifiedKeypoints[i].getDepth() > 0) {
			float u = rectifiedKeypoints[i].getX();
			float v = rectifiedKeypoints[i].getY();
			float Z = rectifiedKeypoints[i].getDepth();
			float X = (u - proj(0, 2)) * Z / proj(0, 0);
			float Y = (v - proj(1, 2)) * Z / proj(1, 1);
			unrectifiedKeypoints[i].setDepth(rot(0, 2) * X + rot(1, 2) * Y + rot(2, 2) * Z);
		}
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARStereoReprojection::reprojectToCloudPoints(const std::vector<SolAR::datastructure::Keypoint>& undistortedKeypoints,
                                                                    const SRef<SolAR::datastructure::DescriptorBuffer> descriptors,
                                                                    const SolAR::datastructure::Transform3Df& pose,
                                                                    const SolAR::datastructure::CamCalibration& intrinsicParams,
                                                                    std::vector<SRef<SolAR::datastructure::CloudPoint>>& cloudPoints)
{
	for (int i = 0; i < undistortedKeypoints.size(); ++i)
		if (undistortedKeypoints[i].getDepth() > 0) {
			std::map<uint32_t, uint32_t> visibility;
			visibility[0] = i;
			float Z = undistortedKeypoints[i].getDepth();
			float X = (undistortedKeypoints[i].getX() - intrinsicParams(0, 2)) * Z / intrinsicParams(0, 0);
			float Y = (undistortedKeypoints[i].getY() - intrinsicParams(1, 2)) * Z / intrinsicParams(1, 1);
			Vector3f pts3D(X, Y, Z);
			Vector3f pts3DTrans = pose * pts3D;
			// calculate view direction
			Vector3f viewDirection(pose(0, 3) - pts3DTrans[0], pose(1, 3) - pts3DTrans[1], pose(2, 3) - pts3DTrans[2]);
			viewDirection = viewDirection / viewDirection.norm();
			// get descriptor 
			SRef<DescriptorBuffer> descriptor = xpcf::utils::make_shared<DescriptorBuffer>(descriptors->getDescriptor(i));
			SRef<CloudPoint> cp = xpcf::utils::make_shared<CloudPoint>(pts3DTrans[0], pts3DTrans[1], pts3DTrans[2],
				undistortedKeypoints[i].getR(), undistortedKeypoints[i].getG(), undistortedKeypoints[i].getB(), viewDirection[0], viewDirection[1], viewDirection[2],
				0.f, visibility, descriptor);
			cloudPoints.push_back(cp);
		}
	return FrameworkReturnCode::_SUCCESS;
}

}
}
}

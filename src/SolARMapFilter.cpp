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

#include "SolARMapFilter.h"
#include "core/Log.h"
#include "datastructure/Keyframe.h"
#include "datastructure/MathDefinitions.h"

namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::TOOLS::SolARMapFilter);

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {

SolARMapFilter::SolARMapFilter():ConfigurableBase(xpcf::toUUID<SolARMapFilter>())
{
    declareInterface<IMapFilter>(this);
    declareProperty("reprojErrorThreshold", m_reprojErrorThreshold);
    declareProperty("cheiralityCheck", m_cheiralityCheck);
    declareProperty("minTriangulationAngle", m_minTriangulationAngle);
    declareProperty("maxTriangulationAngle", m_maxTriangulationAngle);
	LOG_DEBUG("SolARMapFilter constructor");
}

float calculateTriangulationAngle(const Vector3f& center1,
                                const Vector3f& center2,
                                const Vector3f& point3D) {
	float baselineLengthSquared = (center1 - center2).squaredNorm();
	float ray1LengthSquared = (point3D - center1).squaredNorm();
	float ray2lengthSquared = (point3D - center2).squaredNorm();

	// Using "law of cosines" to compute the enclosing angle between rays.
	float denominator = 2.0 * std::sqrt(ray1LengthSquared * ray2lengthSquared);
	if (denominator == 0.0) {
		return 0.f;
	}
	const float nominator = ray1LengthSquared + ray2lengthSquared - baselineLengthSquared;
	const float angle = std::abs(std::acos(nominator / denominator));

	// Triangulation is unstable for acute angles (far away points) and
	// obtuse angles (close points), so always compute the minimum angle
	// between the two intersecting rays.	
    return (angle < SOLAR_PI - angle ? angle : SOLAR_PI - angle);
}

void  SolARMapFilter::filter(const Transform3Df & pose1, const Transform3Df & pose2, const std::vector<SRef<CloudPoint>>& input,  std::vector<SRef<CloudPoint>>& output)
{
	std::vector<int> index;
	this->filter(pose1, pose2, input, output, index);
}

void  SolARMapFilter::filter(const Transform3Df & pose1, const Transform3Df & pose2, const std::vector<SRef<CloudPoint>>& input, std::vector<SRef<CloudPoint>>& output, std::vector<int> &index)
{
	if (input.size() == 0)
	{
        LOG_DEBUG("mapFilter opencv has an empty vector as input");
        return;
	}

	output.clear();

	Transform3Df invPose1, invPose2;
	invPose1 = pose1.inverse();
	invPose2 = pose2.inverse();

	for (int i = 0; i < input.size(); i++)
	{
		// Check for cheirality (if the point is in front of the camera)

        // BUG patch To correct, Vector4f should but is not accepted with windows !
        Vector3f point(input[i]->getX(), input[i]->getY(), input[i]->getZ());
		Vector3f pointInCam1Ref, pointInCam2Ref;

		pointInCam1Ref = invPose1 * point;
		pointInCam2Ref = invPose2 * point;

		if ((!m_cheiralityCheck) || ((pointInCam1Ref(2) >= 0) && pointInCam2Ref(2) >= 0))
		{
			// if the reprojection error is less than the threshold
			if (input[i]->getReprojError() < m_reprojErrorThreshold) {
                // if the angle is greater than the threshold
                float angle = calculateTriangulationAngle(pose1.translation(), pose2.translation(), point);
				if ((angle > m_minTriangulationAngle) && (angle < m_maxTriangulationAngle)) {
					index.push_back(i);
					output.push_back(input[i]);
				}
			}
		}
	}
}

}
}
}

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

#ifndef SOLARPOSEHELPER_H
#define SOLARPOSEHELPER_H

#include "SolARToolsAPI.h"
#include "datastructure/Pose.h"

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {

class SOLAR_TOOLS_EXPORT_API SolARPoseHelper
{
public:
	SolARPoseHelper();
	~SolARPoseHelper();

public:
	/// @brief Transform a point's coordinates using the transform of the Pose
	static Point3Df transformPoint(const Pose& pose, const Point3Df& point);

	/// @brief Transform a point's coordinates using the transform of the Pose
	static Vector4f transformPoint(const Pose& pose, const Vector4f& point);

	/// @brief Transform a Pose into the reference frame of another Pose
	/// @param referencePose : The Pose to use as a reference frame
	/// @param pose : the Pose to transform into the frame of referencePose
	static Pose transformPose(const Pose& referencePose, const Pose& pose);

	/// @brief Recover Euler angles of the Pose's rotation
	/// @return A vector with rotation angles, in radians, with index 0 = rotation around the X-axis, 1 = rotation around Y-axis, 2 = rotation around the Z-axis.
	static Vector3f getEulerAngles(const Pose& pose);

	/// @brief Create a Pose from Euler angles.
	/// @param euler A vector with rotation angles, in radians, with index 0 = rotation around the X-axis, 1 = rotation around Y-axis, 2 = rotation around the Z-axis.
	/// @return A pure rotation Pose with no translation.
	static Pose createPose(const Vector3f& euler);

	/// @brief Create a Pose from Euler angles.
	/// @param translation Translation to apply to the Pose
	/// @param euler A vector with rotation angles, in radians, with index 0 = rotation around the X-axis, 1 = rotation around Y-axis, 2 = rotation around the Z-axis.
	/// @return A Pose with translation set and rotated with the Euler angles provided.
	static Pose createPose(const Vector3f& translation, const Vector3f& euler);

	/// \brief constructs a Pose from a rotation matrix of size 3x3 and a translation vector of size 3
	/// \param rotation_matrix3x3, rotation matrix of size 3x3
	/// \param translation vector, translation vector of size 3
	static Pose createPose(const float rotation_matrix3x3[], const float translation_vector[]);

	/// \brief constructs a Pose from a rotation matrix of size 3x3 and a translation vector of size 3
	/// \param r, rotation matrix of size 3x3
	/// \param t, translation vector of size 3
	static Pose createPose(const  RotationMatrixf &r, const  Vector3f & t);
};
}
}
}
#endif //SOLARPOSEHELPER_H

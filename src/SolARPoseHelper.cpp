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

#include "SolARPoseHelper.h"

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {

SolARPoseHelper::SolARPoseHelper()
{
}

SolARPoseHelper::~SolARPoseHelper()
{
}

Point3Df SolARPoseHelper::transformPoint(const Pose& pose, const Point3Df& point)
{
	Vector4f tmp = transformPoint(pose, Vector4f(point[0], point[1], point[2], 1));
	return Point3Df(tmp[0], tmp[1], tmp[2]);
}

Vector4f SolARPoseHelper::transformPoint(const Pose& pose, const Vector4f& point)
{
	return pose.getPoseTransform() * point;
}

Pose SolARPoseHelper::transformPose(const Pose& referencePose, const Pose& pose)
{
	Transform3Df referenceInverse = referencePose.getPoseTransform().inverse();
	Transform3Df transform = pose.getPoseTransform();
	return Pose(referenceInverse*transform);
}

Vector3f SolARPoseHelper::getEulerAngles(const Pose& pose)
{
	return pose.getPoseTransform().rotation().eulerAngles(0, 1, 2);
}

Pose SolARPoseHelper::createPose(const Vector3f& euler)
{
	Transform3Df transform;
	transform.setIdentity();

	transform.rotate(Eigen::AngleAxis<float>(euler[0], Vector3f::UnitX()));
	transform.rotate(Eigen::AngleAxis<float>(euler[1], Vector3f::UnitY()));
	transform.rotate(Eigen::AngleAxis<float>(euler[2], Vector3f::UnitZ()));

	return Pose(transform);
}

Pose SolARPoseHelper::createPose(const Vector3f& translation, const Vector3f& euler)
{
	Transform3Df transform;

	transform = Translation3Df(translation);
	transform.rotate(Eigen::AngleAxis<float>(euler[0], Vector3f::UnitX()));
	transform.rotate(Eigen::AngleAxis<float>(euler[1], Vector3f::UnitY()));
	transform.rotate(Eigen::AngleAxis<float>(euler[2], Vector3f::UnitZ()));

	return Pose(transform);
}

Pose SolARPoseHelper::createPose(const float rotation_matrix3x3[], const float translation_vector[])
{
	Transform3Df transform;
	transform.setIdentity();

	//rotation matrix
	transform(0, 0) = rotation_matrix3x3[0];  transform(0, 1) = rotation_matrix3x3[1];   transform(0, 2) = rotation_matrix3x3[2];
	transform(1, 0) = rotation_matrix3x3[3];  transform(1, 1) = rotation_matrix3x3[4];   transform(1, 2) = rotation_matrix3x3[5];
	transform(2, 0) = rotation_matrix3x3[6];  transform(2, 1) = rotation_matrix3x3[7];   transform(2, 2) = rotation_matrix3x3[8];

	//translation vector
	transform(0, 3) = translation_vector[0];
	transform(1, 3) = translation_vector[1];
	transform(2, 3) = translation_vector[2];

	return Pose(transform);
}

Pose SolARPoseHelper::createPose(const  RotationMatrixf &r, const  Vector3f &t)
{
	return createPose(r.data(), t.data());
}

}
}
}

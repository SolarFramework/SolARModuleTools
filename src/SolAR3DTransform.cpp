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

#include "SolAR3DTransform.h"
#include "xpcf/component/ComponentFactory.h"
namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::TOOLS::SolAR3DTransform);

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {

SolAR3DTransform::SolAR3DTransform():ComponentBase(xpcf::toUUID<SolAR3DTransform>())
{
    declareInterface<SolAR::api::geom::I3DTransform>(this);
	LOG_DEBUG("SolAR3DTransform constructor");
}


SolAR3DTransform::~SolAR3DTransform(){

}

FrameworkReturnCode SolAR3DTransform::transform(const std::vector<Point3Df> & inputPoints, const Transform3Df & transformation, std::vector<Point3Df> & outputPoints)
{
    Point3Df outputPoint3D;
    Vector3f outputVector3f;

    for (auto inputPoint3D : inputPoints){
        Vector3f inputVector3f(inputPoint3D.getX(),inputPoint3D.getY(), inputPoint3D.getZ());
        outputVector3f = transformation * inputVector3f;
        outputPoint3D.setX(outputVector3f[0]);
        outputPoint3D.setY(outputVector3f[1]);
        outputPoint3D.setZ(outputVector3f[2]);
        outputPoints.push_back(outputPoint3D);
    }
    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolAR3DTransform::transformInPlace(const datastructure::Transform3Df & transformation, SRef<datastructure::Map> map)
{
	// apply transformation to point cloud
	{
		SRef<PointCloud> pointCloud;
		map->getPointCloud(pointCloud);
		this->transformInPlace(transformation, pointCloud);
	}	
	// apply transformation to keyframes
	{
		SRef<KeyframeCollection> keyframeCollection;
		map->getKeyframeCollection(keyframeCollection);
		this->transformInPlace(transformation, keyframeCollection);
	}	
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolAR3DTransform::transformInPlace(const datastructure::Transform3Df & transformation,
	SRef<datastructure::PointCloud> pointCloud)
{
	Vector3f outputVector3f;
	std::vector<SRef<CloudPoint>> inputPoints;
	pointCloud->getAllPoints(inputPoints);

	for (auto i = 0u; i < inputPoints.size(); i++)
	{
		auto& point3D = inputPoints.at(i);
		Vector3f inputVector3f(point3D->getX(), point3D->getY(), point3D->getZ());
		outputVector3f = transformation * inputVector3f;
		point3D->setX(outputVector3f[0]);
		point3D->setY(outputVector3f[1]);
		point3D->setZ(outputVector3f[2]);
	}
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolAR3DTransform::transformInPlace(const datastructure::Transform3Df & transformation, SRef<datastructure::KeyframeCollection> keyframeCollection)
{
	std::vector<SRef<Keyframe>> keyframes;
	keyframeCollection->getAllKeyframes(keyframes);
	// apply transformation to keyframes
	for (auto &kf : keyframes) {
		kf->setPose(transformation * kf->getPose());
	}
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolAR3DTransform::transformInPlace(const Transform3Df & transformation, std::vector<SRef<CloudPoint>>& pointCloud)
{
    // apply transformation to point cloud
    for (auto &cp : pointCloud) {
        Vector3f inputVector3f(cp->getX(), cp->getY(), cp->getZ());
        Vector3f outputVector3f = transformation * inputVector3f;
        cp->setX(outputVector3f[0]);
        cp->setY(outputVector3f[1]);
        cp->setZ(outputVector3f[2]);
    }

    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolAR3DTransform::transformInPlace(const Transform3Df & transformation, std::vector<SRef<Keyframe>>& keyframes)
{
    // apply transformation to keyframes
    for (auto &kf : keyframes) {
        kf->setPose(transformation * kf->getPose());
    }

    return FrameworkReturnCode::_SUCCESS;
}

}
}
}

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

#ifndef SOLARMAPUPDATE_H
#define SOLARMAPUPDATE_H

#include "api/solver/map/IMapUpdate.h"
#include "api/storage/IMapManager.h"
#include "api/geom/IProject.h"
#include "api/features/IDescriptorMatcherRegion.h"
#include "xpcf/component/ConfigurableBase.h"
#include "SolARToolsAPI.h"

namespace SolAR {
namespace MODULES {
namespace TOOLS {
/**
* @class SolARMapUpdate
* @brief Update the global map after merging a local map into the global map.
* <TT>UUID: 3960331a-9190-48f4-aeba-e20bf6a24465</TT>
*
* @SolARComponentInjectablesBegin
* @SolARComponentInjectable{SolAR::api::storage::IMapManager}
* @SolARComponentInjectable{SolAR::api::geom::IProject}
* @SolARComponentInjectable{SolAR::api::features::IDescriptorMatcherRegion}
* @SolARComponentInjectablesEnd
*
* @SolARComponentPropertiesBegin
* @SolARComponentProperty{ m_thresAngleViewDirection,
*                          ,
*                          @SolARComponentPropertyDescNum{ float, [0..3.14f], 0.85f }}
* @SolARComponentPropertiesEnd
*
*/

class SOLAR_TOOLS_EXPORT_API SolARMapUpdate : public org::bcom::xpcf::ConfigurableBase,
	public api::solver::map::IMapUpdate {
public:
	SolARMapUpdate();
	~SolARMapUpdate() = default;

	/// @brief this method is used to set intrinsic parameters and distorsion of the camera
	/// @param[in] intrinsicParams Camera calibration matrix parameters.
	/// @param[in] distortionParams Camera distortion parameters.
	void setCameraParameters(const datastructure::CamCalibration & intrinsicParams, 
							 const datastructure::CamDistortion & distortionParams) override;

	/// @brief Update the global map.
	/// @param[in,out] globalMap the global map
	/// @param[in] newKeyframeIds the ids of new keyframes.
	/// @return FrameworkReturnCode::_SUCCESS_ if the fusion succeed, else FrameworkReturnCode::_ERROR.
	FrameworkReturnCode update(SRef<datastructure::Map> globalMap,
							   const std::vector<uint32_t>& newKeyframeIds) override;

	org::bcom::xpcf::XPCFErrorCode onConfigured() override final;
	void unloadComponent () override final;

private:
	void matchLocalMapPoints(const std::vector<SRef<datastructure::CloudPoint>>& localCloudPoints, 
							SRef<datastructure::Keyframe> newKeyframe, 
							std::vector<SRef<datastructure::CloudPoint>>& notMatchedCloudPoints);

	void defineInvalidCloudPoints(SRef<datastructure::Keyframe> newKeyframe, 
								  std::vector<SRef<datastructure::CloudPoint>>& notMatchedCloudPoints);

	float cosineViewDirectionAngle(const SRef<datastructure::Frame>& frame,
								   const SRef<datastructure::CloudPoint>& cloudPoint);

private:
	SRef<api::storage::IMapManager>					m_mapManager;
	SRef<api::geom::IProject>						m_projector;
	SRef<api::features::IDescriptorMatcherRegion>	m_matcher;
	SRef<datastructure::CovisibilityGraph>          m_covisibilityGraph;
	SRef<datastructure::PointCloud>                 m_pointCloud;
	SRef<datastructure::KeyframeCollection>         m_keyframeCollection;
	float											m_thresAngleViewDirection = 0.85f;
	float											m_thresConfidence = 0.1f;    
};

}
}
}

#endif // SOLARMAPUPDATE_H

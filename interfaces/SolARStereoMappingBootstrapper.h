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

#ifndef SOLARSTEREOMAPPINGBOOTSTRAPPER_H
#define SOLARSTEREOMAPPINGBOOTSTRAPPER_H
#include "api/stereo/IStereoMappingBootstrapper.h"
#include "api/stereo/IStereoDepthEstimation.h"
#include "api/storage/IMapManager.h"
#include "api/display/I2DOverlay.h"
#include "SolARToolsAPI.h"
#include "xpcf/component/ConfigurableBase.h"

namespace SolAR {
namespace MODULES {
namespace TOOLS {

/**
* @class SolARStereoMappingBootstrapper
* @brief <B>Perform mapping bootstrapper using stereo camera.</B>
* <TT>UUID: 02064ef7-e7b9-40e2-8793-6bd177f4bc79</TT>
*
* @SolARComponentInjectablesBegin
* @SolARComponentInjectable{SolAR::api::storage::IMapManager}
* @SolARComponentInjectable{SolAR::api::display::I2DOverlay}
* @SolARComponentInjectable{SolAR::stereo::IStereoDepthEstimation}
* @SolARComponentInjectablesEnd
*
*/

class SOLAR_TOOLS_EXPORT_API SolARStereoMappingBootstrapper : public org::bcom::xpcf::ConfigurableBase,
	public api::stereo::IStereoMappingBootstrapper
{
public:
	///@brief SolARStereoMappingBootstrapper constructor;
	SolARStereoMappingBootstrapper();

	///@brief SolARStereoMappingBootstrapper destructor;
	~SolARStereoMappingBootstrapper() override;

	/// @brief this method is used to set intrinsic parameters of the camera
	/// @param[in] intrinsicParams camera calibration matrix parameters.
	void setCameraParameters(const SolAR::datastructure::CamCalibration & intrinsicParams) override;

	/// @brief This method uses images to boostrap mapping
	/// @param[in] frame input image to process
	/// @param[out] view output image to visualize
	/// @return FrameworkReturnCode::_SUCCESS_ if initialization succeed, else FrameworkReturnCode::_ERROR.
	FrameworkReturnCode process(const SRef<SolAR::datastructure::Frame>& frame,
								SRef<SolAR::datastructure::Image> & view) override;
	
	void unloadComponent() override final;

private:
	int													m_nbMinInitPointCloud = 50;
	datastructure::CamCalibration						m_intrinsicParams;
	SRef<SolAR::api::stereo::IStereoDepthEstimation>	m_stereoDepthEstimator;
	SRef<SolAR::api::storage::IMapManager>				m_mapManager;
	SRef<SolAR::api::display::I2DOverlay>               m_overlay2DGreen, m_overlay2DRed;
};

}
}
}

#endif // SOLARSTEREOMAPPINGBOOTSTRAPPER_H

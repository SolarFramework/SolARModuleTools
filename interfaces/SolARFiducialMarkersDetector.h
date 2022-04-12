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

#ifndef SOLARFIDUCIALMARKERSDETECTOR_H
#define SOLARFIDUCIALMARKERSDETECTOR_H
#include "api/features/I2DTrackablesDetector.h"
#include "api/image/IImageFilter.h"
#include "api/image/IImageConvertor.h"
#include "api/features/IContoursExtractor.h"
#include "api/features/IContoursFilter.h"
#include "api/image/IPerspectiveController.h"
#include "api/features/IDescriptorsExtractorSBPattern.h"
#include "api/features/ISBPatternReIndexer.h"
#include "api/features/ICornerRefinement.h"
#include "datastructure/FiducialMarker.h"
#include "SolARToolsAPI.h"
#include "xpcf/component/ConfigurableBase.h"

namespace SolAR {
namespace MODULES {
namespace TOOLS {

/**
* @class SolAFiducialMarkersDetector
* @brief <B>Detect a set of given 2D fiducial markers in an image.</B>
* <TT>UUID: 31fa26d6-9744-4522-85fa-fc2baafbe397</TT>
*
* @SolARComponentInjectablesBegin
* @SolARComponentInjectable{SolAR::api::image::IImageFilter, optional}
* @SolARComponentInjectable{SolAR::api::image::IImageConvertor}
* @SolARComponentInjectable{SolAR::api::features::IContoursExtractor}
* @SolARComponentInjectable{SolAR::api::features::IContoursFilter}
* @SolARComponentInjectable{SolAR::api::image::IPerspectiveController}
* @SolARComponentInjectable{SolAR::api::features::IDescriptorsExtractorSBPattern}
* @SolARComponentInjectable{SolAR::api::features::IDescriptorMatcher}
* @SolARComponentInjectable{SolAR::api::features::ISBPatternReIndexer}
* @SolARComponentInjectable{SolAR::api::features::ICornerRefinement}
* @SolARComponentInjectablesEnd
*
* @SolARComponentPropertiesBegin
* @SolARComponentProperty{ nbThreshold,
*                         ,
*                         @SolARComponentPropertyDescNum{ int, [0..MAX INT], 3 }}
* @SolARComponentProperty{ minThreshold,
*                          ,
*                          @SolARComponentPropertyDescNum{ int, [-1..MAX INT], -1 }}
* @SolARComponentProperty{ maxThreshold,
*                          ,
*                          @SolARComponentPropertyDescNum{ int, [0..MAX INT], 220 }}
* @SolARComponentPropertiesEnd
*
*/

class SOLAR_TOOLS_EXPORT_API SolAFiducialMarkersDetector : public org::bcom::xpcf::ConfigurableBase,
    public SolAR::api::features::I2DTrackablesDetector
{
public:
    ///@brief SolAFiducialMarkersDetector constructor;
    SolAFiducialMarkersDetector();
    ///@brief SolAFiducialMarkersDetector destructor;
    ~SolAFiducialMarkersDetector() = default;

    /// @brief this method is used to set the set of 2D trackables.
    /// @param[in] trackables the set of 2D trackables.
    FrameworkReturnCode setTrackables(const std::vector<SRef<SolAR::datastructure::Trackable>> trackables) override;

    /// @brief Detect a set of trackables.
    /// @param[in] image input image.
    /// @param[out] corners a set of detected corners corresponding to the trackables (each trackable has a set of 4 corners).
    /// @return FrameworkReturnCode::_SUCCESS if the detection succeed, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode detect(const SRef<SolAR::datastructure::Image> image,
                               std::vector<std::vector<SolAR::datastructure::Point2Df>> & corners) override;

	void unloadComponent() override final;

private:
    SRef<SolAR::api::image::IImageFilter>						m_imageFilterBinary;
    SRef<SolAR::api::image::IImageConvertor>					m_imageConvertor;
    SRef<api::features::IContoursExtractor>                     m_contoursExtractor;
    SRef<SolAR::api::features::IContoursFilter>                 m_contoursFilter;
    SRef<SolAR::api::image::IPerspectiveController>             m_perspectiveController;
    SRef<SolAR::api::features::IDescriptorsExtractorSBPattern>	m_patternDescriptorExtractor;
    SRef<SolAR::api::features::IDescriptorMatcher>				m_patternMatcher;
    SRef<SolAR::api::features::ISBPatternReIndexer>             m_patternReIndexer;
    SRef<SolAR::api::features::ICornerRefinement>				m_cornerRefinement;
    std::map<int, std::vector<std::pair<int, SRef<SolAR::datastructure::DescriptorBuffer>>>> m_markerPatternDescriptors; // first key is the size of pattern, second key is a pair of id and pattern descriptor
	int															m_nbMarkers;
    int                                                         m_nbThreshold = 3;
    int                                                         m_minThreshold = -1;
    int                                                         m_maxThreshold = 220;
};

}
}
}

#endif // SOLARFIDUCIALMARKERSDETECTOR_H

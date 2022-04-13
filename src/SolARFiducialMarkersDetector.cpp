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

#include "SolARFiducialMarkersDetector.h"
#include "core/Log.h"


namespace xpcf = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::TOOLS::SolARFiducialMarkersDetector);


namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {


SolARFiducialMarkersDetector::SolARFiducialMarkersDetector():ConfigurableBase(xpcf::toUUID<SolARFiducialMarkersDetector>())
{
    addInterface<SolAR::api::features::I2DTrackablesDetector>(this);
    declareInjectable<SolAR::api::image::IImageFilter>(m_imageFilterBinary);
    declareInjectable<SolAR::api::image::IImageConvertor>(m_imageConvertor);
    declareInjectable<SolAR::api::features::IContoursExtractor>(m_contoursExtractor);
    declareInjectable<SolAR::api::features::IContoursFilter>(m_contoursFilter);
    declareInjectable<SolAR::api::image::IPerspectiveController>(m_perspectiveController);
    declareInjectable<SolAR::api::features::IDescriptorsExtractorSBPattern>(m_patternDescriptorExtractor);
    declareInjectable<SolAR::api::features::IDescriptorMatcher>(m_patternMatcher);
    declareInjectable<SolAR::api::features::ISBPatternReIndexer>(m_patternReIndexer);
    declareInjectable<SolAR::api::features::ICornerRefinement>(m_cornerRefinement);
	declareProperty("nbThreshold", m_nbThreshold);
	declareProperty("minThreshold", m_minThreshold);
	declareProperty("maxThreshold", m_maxThreshold);
    LOG_DEBUG("SolAFiducialMarkersDetector constructor");
}

FrameworkReturnCode SolARFiducialMarkersDetector::setTrackables(const std::vector<SRef<SolAR::datastructure::Trackable>> trackables)
{
    // components initialisation for marker detection
	m_nbMarkers = trackables.size();
    if (m_nbMarkers == 0)
        return FrameworkReturnCode::_ERROR_;

    for (int i = 0; i < m_nbMarkers; ++i)
        if (trackables[i]->getType() == TrackableType::FIDUCIAL_MARKER) {
			SRef<FiducialMarker> fiducialMarker = xpcf::utils::dynamic_pointer_cast<FiducialMarker>(trackables[i]);
			// extract pattern code
			SquaredBinaryPattern pattern = fiducialMarker->getPattern();
			int patternSize = pattern.getSize();
			SRef<DescriptorBuffer> patternDesc;
			m_patternDescriptorExtractor->extract(pattern, patternDesc);
            m_markerPatternDescriptors[patternSize].push_back(std::make_pair(i, patternDesc));
            LOG_DEBUG("Marker pattern {}:\n {}", i, pattern.getPatternMatrix());
		}		
        else {
            LOG_ERROR("The SolARMultiFiducialMarkersPoseEstimator should only use a trackable of type FIDUCIAL_MARKER")
            return FrameworkReturnCode::_ERROR_;
        }
    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARFiducialMarkersDetector::detect(const SRef<SolAR::datastructure::Image> image,
                                                         std::vector<std::vector<SolAR::datastructure::Point2Df>> & corners)
{		
    corners.resize(m_nbMarkers);
	SRef<Image> greyImage;
	// Convert Image from RGB to grey
	if (image->getNbChannels() != 1)
		m_imageConvertor->convert(image, greyImage, Image::ImageLayout::LAYOUT_GREY);
	else
		greyImage = image->copy();

	int nbFoundMarkers(0);
	for (int num_threshold = 0; num_threshold < m_nbThreshold; num_threshold++)
	{
		SRef<Image>                     binaryImage;
		std::vector<Contour2Df>			contours;
		std::vector<Contour2Df>			filtered_contours;
		std::vector<SRef<Image>>        patches;

		// Compute the current Threshold valu for image binarization
		int threshold = m_minThreshold + (m_maxThreshold - m_minThreshold)*((float)num_threshold / (float)(m_nbThreshold - 1));
		// Convert Image from grey to black and white
		m_imageFilterBinary->bindTo<xpcf::IConfigurable>()->getProperty("min")->setIntegerValue(threshold);
		m_imageFilterBinary->bindTo<xpcf::IConfigurable>()->getProperty("max")->setIntegerValue(255);
		// Convert Image from grey to black and white
		m_imageFilterBinary->filter(greyImage, binaryImage);
		// Extract contours from binary image
		m_contoursExtractor->extract(binaryImage, contours);
		// Filter 4 edges contours to find those candidate for marker contours
		m_contoursFilter->filter(contours, filtered_contours);
		// Create one warpped and cropped image by contour
		m_perspectiveController->correct(binaryImage, filtered_contours, patches);

		for (const auto & markers : m_markerPatternDescriptors) {
			int patternSize = markers.first;
			m_patternDescriptorExtractor->bindTo<xpcf::IConfigurable>()->getProperty("patternSize")->setIntegerValue(patternSize);
			m_patternReIndexer->bindTo<xpcf::IConfigurable>()->getProperty("sbPatternSize")->setIntegerValue(patternSize);
            auto markersIdDescs = markers.second;
            int nbMarkers = markersIdDescs.size();
			SRef<DescriptorBuffer>  recognizedPatternsDescriptors;
			std::vector<Contour2Df>	recognizedContours;
			if (m_patternDescriptorExtractor->extract(patches, filtered_contours, recognizedPatternsDescriptors, recognizedContours) != FrameworkReturnCode::_ERROR_)
			{
				for (int num_marker = 0; num_marker < nbMarkers; num_marker++) {
                    int id = markersIdDescs[num_marker].first;
                    auto markersDescs = markersIdDescs[num_marker].second;
					std::vector<DescriptorMatch>    patternMatches;
					std::vector<Point2Df>			pattern2DPoints;
					std::vector<Point2Df>			img2DPoints;
					// From extracted squared binary pattern, match the one corresponding to the squared binary marker
                    if (m_patternMatcher->match(markersDescs, recognizedPatternsDescriptors, patternMatches) == FrameworkReturnCode::_SUCCESS)
					{
						// Reindex the pattern to create two vector of points, the first one corresponding to marker corner, the second one corresponding to the poitsn of the contour
						m_patternReIndexer->reindex(recognizedContours, patternMatches, pattern2DPoints, img2DPoints);
                        // refine corners
                        m_cornerRefinement->refine(greyImage, img2DPoints);
                        // Get 2D corners of markers
                        corners[id] = img2DPoints;
						nbFoundMarkers++;
					}
				}
			}
		}
		if (nbFoundMarkers > 0)
			break;
	}
	LOG_DEBUG("Number of detected markers: {}", nbFoundMarkers);
	if (nbFoundMarkers == 0)
		return FrameworkReturnCode::_ERROR_;
    else
        return FrameworkReturnCode::_SUCCESS;
}


}
}
}

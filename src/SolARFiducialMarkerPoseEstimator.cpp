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

#include "SolARFiducialMarkerPoseEstimator.h"
#include "core/Log.h"


namespace xpcf = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::TOOLS::SolARFiducialMarkerPoseEstimator);


namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {


SolARFiducialMarkerPoseEstimator::SolARFiducialMarkerPoseEstimator():ConfigurableBase(xpcf::toUUID<SolARFiducialMarkerPoseEstimator>())
{
    addInterface<SolAR::api::solver::pose::ITrackablePose>(this);
    declareInjectable<SolAR::api::image::IImageFilter>(m_imageFilterBinary);
    declareInjectable<SolAR::api::image::IImageConvertor>(m_imageConvertor);
    declareInjectable<SolAR::api::features::IContoursExtractor>(m_contoursExtractor);
    declareInjectable<SolAR::api::features::IContoursFilter>(m_contoursFilter);
    declareInjectable<SolAR::api::image::IPerspectiveController>(m_perspectiveController);
    declareInjectable<SolAR::api::features::IDescriptorsExtractorSBPattern>(m_patternDescriptorExtractor);
    declareInjectable<SolAR::api::features::IDescriptorMatcher>(m_patternMatcher);
    declareInjectable<SolAR::api::features::ISBPatternReIndexer>(m_patternReIndexer);
    declareInjectable<SolAR::api::geom::IImage2WorldMapper>(m_img2worldMapper);
    declareInjectable<SolAR::api::solver::pose::I3DTransformFinderFrom2D3D>(m_pnp);
    declareInjectable<SolAR::api::features::ICornerRefinement>(m_cornerRefinement);
    declareInjectable<SolAR::api::geom::IProject>(m_projector);
	declareProperty("nbThreshold", m_nbThreshold);
	declareProperty("minThreshold", m_minThreshold);
	declareProperty("maxThreshold", m_maxThreshold);
	declareProperty("maxReprojError", m_maxReprojError);
	LOG_DEBUG("SolARFiducialMarkerPoseEstimator constructor");
}

FrameworkReturnCode SolARFiducialMarkerPoseEstimator::setTrackable(const SRef<SolAR::datastructure::Trackable> trackable)
{
    // components initialisation for marker detection
    if (trackable->getType() == TrackableType::FIDUCIAL_MARKER)
    {
        m_fiducialMarker = xpcf::utils::dynamic_pointer_cast<FiducialMarker>(trackable);

        SquaredBinaryPattern pattern = m_fiducialMarker->getPattern();
        m_patternDescriptorExtractor->extract(pattern, m_markerPatternDescriptor);
        LOG_DEBUG("Marker pattern:\n {}", pattern.getPatternMatrix());
        int patternSize = pattern.getSize();
        m_patternDescriptorExtractor->bindTo<xpcf::IConfigurable>()->getProperty("patternSize")->setIntegerValue(patternSize);
        m_patternReIndexer->bindTo<xpcf::IConfigurable>()->getProperty("sbPatternSize")->setIntegerValue(patternSize);
        m_img2worldMapper->bindTo<xpcf::IConfigurable>()->getProperty("digitalWidth")->setIntegerValue(patternSize);
        m_img2worldMapper->bindTo<xpcf::IConfigurable>()->getProperty("digitalHeight")->setIntegerValue(patternSize);
        m_img2worldMapper->bindTo<xpcf::IConfigurable>()->getProperty("worldWidth")->setFloatingValue(m_fiducialMarker->getWidth());
        m_img2worldMapper->bindTo<xpcf::IConfigurable>()->getProperty("worldHeight")->setFloatingValue(m_fiducialMarker->getHeight());
    }
    else {
        LOG_ERROR("The SolARFiducialMarkerPoseEstimator should only use a trackable of type FIDUCIAL_MARKER")
        return FrameworkReturnCode::_ERROR_;
    }
    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARFiducialMarkerPoseEstimator::estimate(const SRef<SolAR::datastructure::Image> image,
                                                               const SolAR::datastructure::CameraParameters & camParams,
                                                               SolAR::datastructure::Transform3Df & pose)
{
	SRef<Image>                     greyImage, binaryImage;
	std::vector<Contour2Df>			contours;
	std::vector<Contour2Df>			filtered_contours;
	std::vector<SRef<Image>>        patches;
	std::vector<Contour2Df>			recognizedContours;
	SRef<DescriptorBuffer>          recognizedPatternsDescriptors;
	std::vector<DescriptorMatch>    patternMatches;
	std::vector<Point2Df>			pattern2DPoints;
	std::vector<Point2Df>			img2DPoints;
	std::vector<Point3Df>			pattern3DPoints;

	// Convert Image from RGB to grey
	if (image->getNbChannels() != 1)
		m_imageConvertor->convert(image, greyImage, Image::ImageLayout::LAYOUT_GREY);
	else
		greyImage = image->copy();
	for (int num_threshold = 0; num_threshold < m_nbThreshold; num_threshold++)
	{
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
		// test if this last image is really a squared binary marker, and if it is the case, extract its descriptor
		if (m_patternDescriptorExtractor->extract(patches, filtered_contours, recognizedPatternsDescriptors, recognizedContours) != FrameworkReturnCode::_ERROR_)
		{
			// From extracted squared binary pattern, match the one corresponding to the squared binary marker
            if (m_patternMatcher->match(m_markerPatternDescriptor, recognizedPatternsDescriptors, patternMatches) == FrameworkReturnCode::_SUCCESS)
			{
				// Reindex the pattern to create two vector of points, the first one corresponding to marker corner, the second one corresponding to the poitsn of the contour
				m_patternReIndexer->reindex(recognizedContours, patternMatches, pattern2DPoints, img2DPoints);
				// Compute the 3D position of each corner of the marker
				m_img2worldMapper->map(pattern2DPoints, pattern3DPoints);
				// Refine corner locations
				m_cornerRefinement->refine(greyImage, img2DPoints);
				// Compute the pose of the camera using a Perspective n Points algorithm using only the 4 corners of the marker
				if (m_pnp->estimate(img2DPoints, pattern3DPoints, camParams, pose) == FrameworkReturnCode::_SUCCESS)
				{
					std::vector<Point2Df> projected2DPts;
					m_projector->project(pattern3DPoints, pose, camParams, projected2DPts);
					float errorReproj(0.f);
					for (int j = 0; j < projected2DPts.size(); ++j)
						errorReproj += (projected2DPts[j] - img2DPoints[j]).norm();
					errorReproj /= projected2DPts.size();
					LOG_DEBUG("Mean reprojection error: {}", errorReproj);
					if (errorReproj < m_maxReprojError)
						return FrameworkReturnCode::_SUCCESS;
					pose = Transform3Df::Identity();
					return FrameworkReturnCode::_ERROR_;
				}
			}
		}
	}
	return FrameworkReturnCode::_ERROR_;
}


}
}
}

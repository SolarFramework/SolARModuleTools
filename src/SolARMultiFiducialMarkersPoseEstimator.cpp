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

#include "SolARMultiFiducialMarkersPoseEstimator.h"
#include "core/Log.h"


namespace xpcf = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::TOOLS::SolARMultiFiducialMarkersPoseEstimator);


namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {


SolARMultiFiducialMarkersPoseEstimator::SolARMultiFiducialMarkersPoseEstimator():ConfigurableBase(xpcf::toUUID<SolARMultiFiducialMarkersPoseEstimator>())
{
    addInterface<SolAR::api::solver::pose::IMultiTrackablesPose>(this);
    declareInjectable<SolAR::api::image::IImageFilter>(m_imageFilterBinary);
    declareInjectable<SolAR::api::image::IImageConvertor>(m_imageConvertor);
    declareInjectable<SolAR::api::features::IContoursExtractor>(m_contoursExtractor);
    declareInjectable<SolAR::api::features::IContoursFilter>(m_contoursFilter);
    declareInjectable<SolAR::api::image::IPerspectiveController>(m_perspectiveController);
    declareInjectable<SolAR::api::features::IDescriptorsExtractorSBPattern>(m_patternDescriptorExtractor);
    declareInjectable<SolAR::api::features::IDescriptorMatcher>(m_patternMatcher);
    declareInjectable<SolAR::api::features::ISBPatternReIndexer>(m_patternReIndexer);
    declareInjectable<SolAR::api::solver::pose::I3DTransformFinderFrom2D3D>(m_pnp);
    declareInjectable<SolAR::api::features::ICornerRefinement>(m_cornerRefinement);
    declareInjectable<SolAR::api::geom::IProject>(m_projector);
	declareProperty("nbThreshold", m_nbThreshold);
	declareProperty("minThreshold", m_minThreshold);
	declareProperty("maxThreshold", m_maxThreshold);
	declareProperty("maxReprojError", m_maxReprojError);
    LOG_DEBUG("SolARMultiFiducialMarkersPoseEstimator constructor");
}

void SolARMultiFiducialMarkersPoseEstimator::setCameraParameters(const CamCalibration & intrinsicParams, const CamDistortion & distortionParams) {
	m_camMatrix = intrinsicParams;
	m_camDistortion = distortionParams;
	m_pnp->setCameraParameters(m_camMatrix, m_camDistortion);
	m_projector->setCameraParameters(m_camMatrix, m_camDistortion);
}

FrameworkReturnCode SolARMultiFiducialMarkersPoseEstimator::setTrackables(const std::vector<SRef<SolAR::datastructure::Trackable>> trackables)
{
    // components initialisation for marker detection
	m_nbMarkers = trackables.size();
    if (m_nbMarkers == 0)
        return FrameworkReturnCode::_ERROR_;

    if (trackables[0]->getType() == TrackableType::FIDUCIAL_MARKER)
    {
		for (int i = 0; i < m_nbMarkers; ++i) {
			SRef<FiducialMarker> fiducialMarker = xpcf::utils::dynamic_pointer_cast<FiducialMarker>(trackables[i]);
			// extract pattern code
			SquaredBinaryPattern pattern = fiducialMarker->getPattern();
			int patternSize = pattern.getSize();
			SRef<DescriptorBuffer> patternDesc;
			m_patternDescriptorExtractor->extract(pattern, patternDesc);
			m_markerPatternDescriptors[patternSize].push_back(patternDesc);
			LOG_DEBUG("Marker pattern:\n {}", pattern.getPatternMatrix());			
			// get 3D corner points
			std::vector<Point3Df> pts3D;
			fiducialMarker->getWorldCorners(pts3D);
			m_pattern3DPoints[patternSize].push_back(pts3D);
		}		
    }
    else {
        LOG_ERROR("The SolARMultiFiducialMarkersPoseEstimator should only use a trackable of type FIDUCIAL_MARKER")
        return FrameworkReturnCode::_ERROR_;
    }
    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMultiFiducialMarkersPoseEstimator::estimate(const SRef<Image> image, Transform3Df & pose)
{		
	SRef<Image> greyImage;
	// Convert Image from RGB to grey
	if (image->getNbChannels() != 1)
		m_imageConvertor->convert(image, greyImage, Image::ImageLayout::LAYOUT_GREY);
	else
		greyImage = image->copy();

	std::vector<Point2Df>	pts2D;
	std::vector<Point3Df>	pts3D;
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
			auto markersDescs = markers.second;
			int nbMarkers = markersDescs.size();
			SRef<DescriptorBuffer>  recognizedPatternsDescriptors;
			std::vector<Contour2Df>	recognizedContours;
			if (m_patternDescriptorExtractor->extract(patches, filtered_contours, recognizedPatternsDescriptors, recognizedContours) != FrameworkReturnCode::_ERROR_)
			{
				for (int num_marker = 0; num_marker < nbMarkers; num_marker++) {
					std::vector<DescriptorMatch>    patternMatches;
					std::vector<Point2Df>			pattern2DPoints;
					std::vector<Point2Df>			img2DPoints;
					// From extracted squared binary pattern, match the one corresponding to the squared binary marker
					if (m_patternMatcher->match(markersDescs[num_marker], recognizedPatternsDescriptors, patternMatches) == FrameworkReturnCode::_SUCCESS)
					{
						// Reindex the pattern to create two vector of points, the first one corresponding to marker corner, the second one corresponding to the poitsn of the contour
						m_patternReIndexer->reindex(recognizedContours, patternMatches, pattern2DPoints, img2DPoints);
						// Get 2D-3D points correspondences
						pts2D.insert(pts2D.end(), img2DPoints.begin(), img2DPoints.end());
						const std::vector<Point3Df>& pattern3DPoints = m_pattern3DPoints[patternSize][num_marker];
						pts3D.insert(pts3D.end(), pattern3DPoints.begin(), pattern3DPoints.end());
						nbFoundMarkers++;
					}
				}
			}
		}
		if (nbFoundMarkers > 0)
			break;
	}
	LOG_DEBUG("Number of detected markers: {}", nbFoundMarkers);
	LOG_DEBUG("Number of corners: {}", pts2D.size());
	if (nbFoundMarkers == 0)
		return FrameworkReturnCode::_ERROR_;
		
	// Refine corner locations
	m_cornerRefinement->refine(greyImage, pts2D);

	// Compute the pose of the camera using a Perspective n Points algorithm using all corners of the detected markers
	if (m_pnp->estimate(pts2D, pts3D, pose) == FrameworkReturnCode::_SUCCESS)
	{
		std::vector<Point2Df> projected2DPts;
		m_projector->project(pts3D, projected2DPts, pose);
		float errorReproj(0.f);
		for (int j = 0; j < projected2DPts.size(); ++j)
			errorReproj += (projected2DPts[j] - pts2D[j]).norm();
		errorReproj /= projected2DPts.size();
		LOG_DEBUG("Mean reprojection error: {}", errorReproj);
		if (errorReproj < m_maxReprojError)
			return FrameworkReturnCode::_SUCCESS;
		pose = Transform3Df::Identity();
	}
	return FrameworkReturnCode::_ERROR_;
}


}
}
}

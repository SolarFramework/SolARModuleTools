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

#include <boost/log/core.hpp>
#include "xpcf/xpcf.h"
#include "core/Log.h"
#include "api/input/devices/IARDevice.h"
#include "api/display/IImageViewer.h"
#include "api/features/IKeypointDetector.h"
#include "api/features/IDescriptorsExtractor.h"
#include "api/display/IMatchesOverlay.h"
#include "api/geom/IUndistortPoints.h"
#include "api/display/I3DPointsViewer.h"
#include "api/display/I2DOverlay.h"
#include "api/solver/map/IMapFilter.h"
#include "api/geom/I2DPointsRectification.h"
#include "api/features/IDescriptorMatcherStereo.h"
#include "api/geom/IDepthEstimation.h"
#include "api/geom/IReprojectionStereo.h"

using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;
namespace xpcf = org::bcom::xpcf;

int main(int argc, char *argv[])
{

#if NDEBUG
	boost::log::core::get()->set_logging_enabled(false);
#endif

	LOG_ADD_LOG_TO_CONSOLE();

	try {
		/* instantiate component manager*/
		/* this is needed in dynamic mode */
		SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

        std::string configxml = std::string("SolARTest_ModuleTools_StereoDepthEstimation_conf.xml");
		if (argc == 2)
			configxml = std::string(argv[1]);

		if (xpcfComponentManager->load(configxml.c_str()) != org::bcom::xpcf::_SUCCESS)
		{
			LOG_ERROR("Failed to load the configuration file {}", configxml.c_str());
			return -1;
		}

		// declare and create components
		LOG_INFO("Start creating components");
		auto arDevice = xpcfComponentManager->resolve<input::devices::IARDevice>();
		auto keypointsDetector = xpcfComponentManager->resolve<features::IKeypointDetector>();
		auto descriptorExtractor = xpcfComponentManager->resolve<features::IDescriptorsExtractor>();
		auto undistortKeypoints = xpcfComponentManager->resolve<api::geom::IUndistortPoints>();
		auto imageViewer = xpcfComponentManager->resolve<display::IImageViewer>();
		auto viewer3D = xpcfComponentManager->resolve<display::I3DPointsViewer>();
		auto stereoRectificator = xpcfComponentManager->resolve<geom::I2DPointsRectification>();
		auto stereoMatcher = xpcfComponentManager->resolve<features::IDescriptorMatcherStereo>();
		auto stereoDepthEstimator = xpcfComponentManager->resolve<geom::IDepthEstimation>();
		auto stereoReprojector = xpcfComponentManager->resolve<geom::IReprojectionStereo>();
		auto overlay2DGreen = xpcfComponentManager->resolve<display::I2DOverlay>("Green");
		auto overlay2DRed = xpcfComponentManager->resolve<display::I2DOverlay>("Red");
		LOG_INFO("Components created!");

		// get camera parameters
		CameraRigParameters camRigParams = arDevice->getCameraParameters();
		if (camRigParams.rectificationParams.find(std::make_pair(1, 2)) == camRigParams.rectificationParams.end()) {
			LOG_ERROR("Cannot find rectification parameters between camera 1 and 2");
			return -1;
		}
		RectificationParameters rectParamsL = camRigParams.rectificationParams[std::make_pair(1, 2)].first;
		RectificationParameters rectParamsR = camRigParams.rectificationParams[std::make_pair(1, 2)].second;
		CameraParameters camParamsL = camRigParams.cameraParams[1];
		CameraParameters camParamsR = camRigParams.cameraParams[2];
		
		// get focal and baseline
		float focal = camParamsL.intrinsic(0, 0);
		float baseline = rectParamsL.baseline;
		LOG_INFO("Focal: {}", focal);
		LOG_INFO("Baseline: {}", baseline);
		
		if (arDevice->start() == FrameworkReturnCode::_ERROR_)
		{
			LOG_ERROR("Cannot start loader");
			return -1;
		}
		LOG_INFO("Started!");		

		std::vector<SRef<CloudPoint>> allCloudPoints;
		int nbDropFrameCount(-1);
		while (true) {
			// get data
			std::vector<SRef<Image>> images;
			std::vector<Transform3Df> poses;
			std::chrono::system_clock::time_point timestamp;
			if (arDevice->getData(images, poses, timestamp) != FrameworkReturnCode::_SUCCESS) {
				LOG_ERROR("Error during capture");
				break;
			}
			// get left and right images
			SRef<Image> imageL, imageR;
			imageL = images[1];
			imageR = images[2];

			// get pose
			Transform3Df poseL = poses[1];

			// check 
			nbDropFrameCount++;
			if (nbDropFrameCount % 10 != 0)
				continue;
			else
				nbDropFrameCount = 0;

			// feature extraction and rectification left image
			SRef<DescriptorBuffer> descriptorsL;
			std::vector<Keypoint> keypointsL, undistortedKeypointsL, undistortedKeypointsLRect;
			keypointsDetector->detect(imageL, keypointsL);
			undistortKeypoints->undistort(keypointsL, camParamsL, undistortedKeypointsL);
			descriptorExtractor->extract(imageL, keypointsL, descriptorsL);
			stereoRectificator->rectify(undistortedKeypointsL, camParamsL, rectParamsL, undistortedKeypointsLRect);

			// feature extraction and rectification right image
			SRef<DescriptorBuffer> descriptorsR;
			std::vector<Keypoint> keypointsR, undistortedKeypointsR, undistortedKeypointsRRect;
			keypointsDetector->detect(imageR, keypointsR);
			undistortKeypoints->undistort(keypointsR, camParamsR, undistortedKeypointsR);
			descriptorExtractor->extract(imageR, keypointsR, descriptorsR);
			stereoRectificator->rectify(undistortedKeypointsR, camParamsR, rectParamsR, undistortedKeypointsRRect);

			// stereo feature matching
			std::vector<DescriptorMatch> matches;
			stereoMatcher->match(descriptorsL, descriptorsR, undistortedKeypointsLRect, undistortedKeypointsRRect,
				rectParamsL.type, matches);
			LOG_INFO("Number of matches: {}", matches.size());

			// depth estimation
			stereoDepthEstimator->estimate(undistortedKeypointsLRect, undistortedKeypointsRRect, matches, 
				focal, baseline, rectParamsL.type);
			stereoReprojector->reprojectToUnrectification(undistortedKeypointsLRect, rectParamsL, undistortedKeypointsL);

			// draw keypoints
			SRef<Image> displayImage = imageL->copy();
			for (int i = 0; i < undistortedKeypointsL.size(); ++i)
				if (undistortedKeypointsL[i].getDepth() > 0)
					overlay2DGreen->drawCircle(keypointsL[i], displayImage);
				else
					overlay2DRed->drawCircle(keypointsL[i], displayImage);			

			// reproject to cloud points
            SRef<Frame> frame = xpcf::utils::make_shared<Frame>(keypointsL, undistortedKeypointsL, descriptorsL, imageL, 1, poseL);
			std::vector<SRef<CloudPoint>> cloudPoints;
			stereoReprojector->reprojectToCloudPoints(frame, camParamsL.intrinsic, cloudPoints);
			LOG_INFO("Number of cloud points: {}", cloudPoints.size());
			allCloudPoints.insert(allCloudPoints.end(), cloudPoints.begin(), cloudPoints.end());

			// display			
			if (imageViewer->display(displayImage) != FrameworkReturnCode::_SUCCESS) break;
			if (viewer3D->display(allCloudPoints, poseL, {}, {}, cloudPoints) != FrameworkReturnCode::_SUCCESS) break;
		}

		// display all point cloud
		while (viewer3D->display(allCloudPoints, Transform3Df::Identity()) == FrameworkReturnCode::_SUCCESS);
	}

	catch (xpcf::Exception e)
	{
		LOG_ERROR("The following exception has been catch : {}", e.what());
		return -1;
	}
	return 0;
}

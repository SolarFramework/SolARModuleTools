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
#include "SolAR3DTransformEstimationSACFrom3D3D.h"
#include "core/Log.h"

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::TOOLS::SolAR3DTransformEstimationSACFrom3D3D);

namespace xpcf  = org::bcom::xpcf;

namespace SolAR {
using namespace datastructure;
using namespace api::geom;
namespace MODULES {
namespace TOOLS {

SolAR3DTransformEstimationSACFrom3D3D::SolAR3DTransformEstimationSACFrom3D3D() :ConfigurableBase(xpcf::toUUID<SolAR3DTransformEstimationSACFrom3D3D>())
{
	declareInterface<api::solver::pose::I3DTransformSACFinderFrom3D3D>(this);
	declareInjectable<api::geom::I3DTransform>(m_transform3D);
	declareInjectable<api::geom::IProject>(m_projector);
	declareInjectable<api::solver::map::IBundler>(m_bundler);
	declareProperty("iterationsCount", m_iterationsCount);
	declareProperty("reprojError", m_reprojError);
	declareProperty("distanceError", m_distanceError);
	declareProperty("confidence", m_confidence);
	declareProperty("minNbInliers", m_NbInliersToValidPose);
	declareProperty("optimizeSim3", m_optimizeSim3);
	srand(time(NULL));
	LOG_DEBUG(" SolAR3DTransformEstimationSACFrom3D3D constructor");
}

void randomIndices(int maxIndex, int nbIndices, std::vector<int> &indices) {
	while (indices.size() != nbIndices) {
		int index = rand() % maxIndex;
		if (std::find(indices.begin(), indices.end(), index) == indices.end())
			indices.push_back(index);
	}
}

bool find(const std::vector<Point3Df> & firstPoints3D,
		const std::vector<Point3Df> & secondPoints3D,
		Transform3Df & pose)
{
	// init 3D transformation
	pose.linear() = Eigen::Matrix3f::Identity(3, 3);
	pose.translation() = Eigen::Vector3f::Zero();

	Eigen::MatrixXf in, out;
	in.resize(firstPoints3D.size(), 3);
	out.resize(secondPoints3D.size(), 3);
	if (in.rows() != out.rows())
		return false;
	for (int i = 0; i < firstPoints3D.size(); i++) {
		in.row(i) = Eigen::Vector3f(firstPoints3D[i].getX(), firstPoints3D[i].getY(), firstPoints3D[i].getZ());
		out.row(i) = Eigen::Vector3f(secondPoints3D[i].getX(), secondPoints3D[i].getY(), secondPoints3D[i].getZ());
	}

	// First find the scale, by finding the ratio of sums of some distances,
	// then bring the datasets to the same scale.
	float dist_in = 0, dist_out = 0;
	for (int row = 0; row < in.rows() - 1; row++) {
		dist_in += (in.row(row + 1) - in.row(row)).norm();
		dist_out += (out.row(row + 1) - out.row(row)).norm();
	}
	if (dist_in <= 0 || dist_out <= 0)
		return false;

	float scale = dist_out / dist_in;
	out /= scale;

	// Find the centroids then shift to the origin
	Eigen::Vector3f in_ctr = Eigen::Vector3f::Zero();
	Eigen::Vector3f out_ctr = Eigen::Vector3f::Zero();
	for (int row = 0; row < in.rows(); row++) {
		in_ctr += in.row(row);
		out_ctr += out.row(row);
	}
	in_ctr /= in.rows();
	out_ctr /= out.rows();
	for (int row = 0; row < in.rows(); row++) {
		in.row(row) -= in_ctr;
		out.row(row) -= out_ctr;
	}

	// SVD
	Eigen::MatrixXf Cov = in.transpose() * out;
	Eigen::JacobiSVD<Eigen::MatrixXf> svd(Cov, Eigen::ComputeThinU | Eigen::ComputeThinV);

	// Find the rotation
	float d = (svd.matrixV() * svd.matrixU().transpose()).determinant();
	if (d > 0)
		d = 1.f;
	else
		d = -1.f;
	Eigen::Matrix3f I = Eigen::Matrix3f::Identity(3, 3);
	I(2, 2) = d;
	Eigen::Matrix3f R = svd.matrixV() * I * svd.matrixU().transpose();

	// The final transform
	pose.linear() = scale * R;
	pose.translation() = scale * (out_ctr - R * in_ctr);
	return true;
}

void getInliers(const std::vector<Point3Df> &firstPoints3DTrans, const std::vector<Point3Df> &secondPoints3D, const float &thres, std::vector<int> &inliers)
{
	for (int i = 0; i < firstPoints3DTrans.size(); ++i)
		if ((firstPoints3DTrans[i] - secondPoints3D[i]).norm() < thres)
			inliers.push_back(i);
}

void getInliersByProject(const std::vector<Point2Df> &firstProjected2DPts, const std::vector<Point2Df> &keypoints2, const std::vector<Point2Df> &secondProjected2DPts, const std::vector<Point2Df> &keypoints1, const float &thres, std::vector<int> &inliers)
{
	for (int i = 0; i < firstProjected2DPts.size(); ++i)
		if (((firstProjected2DPts[i] - keypoints2[i]).norm() < thres) && ((secondProjected2DPts[i] - keypoints1[i]).norm() < thres))
			inliers.push_back(i);
}

FrameworkReturnCode SolAR3DTransformEstimationSACFrom3D3D::estimate(const std::vector<Point3Df> & firstPoints3D,
																	const std::vector<Point3Df> & secondPoints3D,
																	Transform3Df & pose,
                                                                    std::vector<int> &inliers)
{	
	if ((firstPoints3D.size() != secondPoints3D.size()) || (firstPoints3D.size() < 3))
		return FrameworkReturnCode::_ERROR_;
	int iterations = m_iterationsCount;
	std::vector<int> bestInliers;
	Transform3Df bestPose;

	while (iterations != 0) {
		// get 3 random indices
		std::vector<int> indices;
        randomIndices(static_cast<int>(firstPoints3D.size()), 3, indices);
		// get 3 correspondences
		std::vector<Point3Df> points3D1, points3D2;
		for (auto &it : indices) {
			points3D1.push_back(firstPoints3D[it]);
			points3D2.push_back(secondPoints3D[it]);
		}
		// compute pose
		Transform3Df tmpPose;
		if (!find(points3D1, points3D2, tmpPose)) {
			iterations--;
			continue;
		}
		// transform first 3D points using the estimated pose
		std::vector<Point3Df> firstPoints3DTrans;
		m_transform3D->transform(firstPoints3D, tmpPose, firstPoints3DTrans);
		// get inliers
		std::vector<int> tmpInliers;
		getInliers(firstPoints3DTrans, secondPoints3D, m_distanceError, tmpInliers);
		if (tmpInliers.size() > bestInliers.size()) {
			bestPose = tmpPose;
			bestInliers.swap(tmpInliers);
		}
		// check confidence
		if ((float)(bestInliers.size()) / firstPoints3D.size() > m_confidence)
			break;

		iterations--;
	}

	if (bestInliers.size() < m_NbInliersToValidPose) {
		LOG_DEBUG("Number of inliers points must be valid ( equal and > to {}): {} inliers for {} input points", m_NbInliersToValidPose, bestInliers.size(), firstPoints3D.size());
		return FrameworkReturnCode::_ERROR_;
	}

	// find the best pose on all inliers
	std::vector<Point3Df> points3D1, points3D2;
	for (auto &it : bestInliers) {
		points3D1.push_back(firstPoints3D[it]);
		points3D2.push_back(secondPoints3D[it]);
	}
	// compute the best pose
	if (!find(points3D1, points3D2, pose)) 
		return FrameworkReturnCode::_ERROR_;
	inliers.swap(bestInliers);

	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolAR3DTransformEstimationSACFrom3D3D::estimate(const SRef<Keyframe> firstKeyframe,
                                                                    const SRef<Keyframe> secondKeyframe,
																	const std::vector<DescriptorMatch>& matches, 
																	const std::vector<Point3Df>& firstPoints3D, 
																	const std::vector<Point3Df>& secondPoints3D, 
																	Transform3Df & pose, 
                                                                    std::vector<int>& inliers)
{
	if ((firstPoints3D.size() != secondPoints3D.size()) || (matches.size() != firstPoints3D.size()) || (firstPoints3D.size() < 3))
		return FrameworkReturnCode::_ERROR_;

	// get poses
	const Transform3Df &pose1 = firstKeyframe->getPose();
	const Transform3Df &pose2 = secondKeyframe->getPose();
	// get keypoints
	std::vector<Point2Df> keypoints1, keypoints2;
	for (const auto &it : matches) {
		const Keypoint kp1 = firstKeyframe->getKeypoint(it.getIndexInDescriptorA());
		const Keypoint kp2 = secondKeyframe->getKeypoint(it.getIndexInDescriptorB());
		keypoints1.push_back(Point2Df(kp1.getX(), kp1.getY()));
		keypoints2.push_back(Point2Df(kp2.getX(), kp2.getY()));
	}
	
	int iterations = m_iterationsCount;
	std::vector<int> bestInliers;
	Transform3Df bestPose;

	while (iterations != 0) {
		// get 3 random indices
		std::vector<int> indices;
        randomIndices(static_cast<int>(firstPoints3D.size()), 3, indices);
		// get 3 correspondences
		std::vector<Point3Df> points3D1, points3D2;
		for (auto &it : indices) {
			points3D1.push_back(firstPoints3D[it]);
			points3D2.push_back(secondPoints3D[it]);
		}
		// compute pose
		Transform3Df tmpPose;
		if (!find(points3D1, points3D2, tmpPose)) {
			iterations--;
			continue;
		}
		// transform 3D points using the estimated pose
		std::vector<Point3Df> firstPoints3DTrans, secondPoints3DTrans;
		m_transform3D->transform(firstPoints3D, tmpPose, firstPoints3DTrans);
		m_transform3D->transform(secondPoints3D, tmpPose.inverse(), secondPoints3DTrans);
		// project the 3D tranformed points on the image
		std::vector< Point2Df > firstProjected2DPts, secondProjected2DPts;
		m_projector->project(firstPoints3DTrans, pose2, secondKeyframe->getCameraParameters(), firstProjected2DPts);
		m_projector->project(secondPoints3DTrans, pose1, firstKeyframe->getCameraParameters(), secondProjected2DPts);
		// get inliers
		std::vector<int> tmpInliers;
		getInliersByProject(firstProjected2DPts, keypoints2, secondProjected2DPts, keypoints1, m_reprojError, tmpInliers);
		if (tmpInliers.size() > bestInliers.size()) {
			bestPose = tmpPose;
			bestInliers.swap(tmpInliers);
		}
		// check confidence
		if ((float)(bestInliers.size()) / firstPoints3D.size() > m_confidence)
			break;

		iterations--;
	}

	if (bestInliers.size() < m_NbInliersToValidPose) {
		LOG_WARNING("Number of inliers points must be valid ( equal and > to {}): {} inliers for {} input points", m_NbInliersToValidPose, bestInliers.size(), firstPoints3D.size());
		return FrameworkReturnCode::_ERROR_;
	}

	// find the best pose on all inliers
	std::vector<Point3Df> bestPts3D1, bestPts3D2;
	std::vector<DescriptorMatch> bestMatches;
	for (auto &it : bestInliers) {
		bestPts3D1.push_back(firstPoints3D[it]);
		bestPts3D2.push_back(secondPoints3D[it]);
		bestMatches.push_back(matches[it]);
	}
	// compute the best pose
	if (!find(bestPts3D1, bestPts3D2, pose))
		return FrameworkReturnCode::_ERROR_;
	inliers.swap(bestInliers);
	if (m_optimizeSim3)
		m_bundler->optimizeSim3(firstKeyframe, secondKeyframe, bestMatches, bestPts3D1, bestPts3D2, pose);
	return FrameworkReturnCode::_SUCCESS;
}

}
}
}

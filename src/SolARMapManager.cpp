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

#include "SolARMapManager.h"
#include "core/Log.h"

namespace xpcf  = org::bcom::xpcf;


XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::TOOLS::SolARMapManager)

namespace SolAR {
using namespace datastructure;
using namespace api::storage;
using namespace api::reloc;
namespace MODULES {
namespace TOOLS {

SolARMapManager::SolARMapManager():ConfigurableBase(xpcf::toUUID<SolARMapManager>())
{
    declareInterface<IMapManager>(this);
    declareInjectable<IPointCloudManager>(m_pointCloudManager);
    declareInjectable<IKeyframesManager>(m_keyframesManager);
    declareInjectable<ICovisibilityGraphManager>(m_covisibilityGraphManager);
    declareInjectable<IKeyframeRetriever>(m_keyframeRetriever);	
	declareProperty("directory", m_directory);
	declareProperty("identificationFileName", m_identificationFileName);
	declareProperty("coordinateFileName", m_coordinateFileName);
	declareProperty("pointCloudManagerFileName", m_pcManagerFileName);
	declareProperty("keyframesManagerFileName", m_kfManagerFileName);
	declareProperty("covisibilityGraphFileName", m_covisGraphFileName);
	declareProperty("keyframeRetrieverFileName", m_kfRetrieverFileName);
	declareProperty("reprojErrorThreshold", m_reprojErrorThres);
	declareProperty("thresConfidence", m_thresConfidence);
	declareProperty("ratioRedundantObs", m_ratioRedundantObs);
	LOG_DEBUG("SolARMapManager constructor");
}

xpcf::XPCFErrorCode SolARMapManager::onConfigured()
{
	LOG_DEBUG(" SolARMapManager onConfigured");
	m_map = xpcf::utils::make_shared<datastructure::Map>();
	m_map->setPointCloud(m_pointCloudManager->getConstPointCloud());
	m_map->setKeyframeCollection(m_keyframesManager->getConstKeyframeCollection());
	m_map->setCovisibilityGraph(m_covisibilityGraphManager->getConstCovisibilityGraph());
	m_map->setKeyframeRetrieval(m_keyframeRetriever->getConstKeyframeRetrieval());
	return xpcf::XPCFErrorCode::_SUCCESS;
}

FrameworkReturnCode SolARMapManager::setMap(const SRef<Map> map)
{
	m_map = map;
	m_pointCloudManager->setPointCloud(m_map->getConstPointCloud());
	m_keyframesManager->setKeyframeCollection(m_map->getConstKeyframeCollection());
	m_covisibilityGraphManager->setCovisibilityGraph(m_map->getConstCovisibilityGraph());
	m_keyframeRetriever->setKeyframeRetrieval(m_map->getConstKeyframeRetrieval());
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMapManager::getMap(SRef<Map>& map)
{
	map = m_map;
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMapManager::getLocalPointCloud(const SRef<Keyframe> keyframe, const float minWeightNeighbor, std::vector<SRef<CloudPoint>>& localPointCloud) const
{	
	// get neighbor keyframes of the keyframe
	std::vector<uint32_t> neighKeyframesId;
	m_covisibilityGraphManager->getNeighbors(keyframe->getId(), minWeightNeighbor, neighKeyframesId);
	neighKeyframesId.push_back(keyframe->getId());
	// get all cloud point visibilities from keyframes
	std::map<uint32_t, std::map<uint32_t, uint32_t>> tmpIdxLocalMap;
	for (auto const &it : neighKeyframesId) {
		SRef<Keyframe> keyframe;
		if (m_keyframesManager->getKeyframe(it, keyframe) != FrameworkReturnCode::_SUCCESS)
			continue;
		const std::map<uint32_t, uint32_t> &visibility = keyframe->getVisibility();
		for (auto const &v : visibility)
			tmpIdxLocalMap[v.second][it] = v.first;
	}
	// get local point cloud
	for (auto const &it : tmpIdxLocalMap) {
		SRef<CloudPoint> point;
		if (m_pointCloudManager->getPoint(it.first, point) == FrameworkReturnCode::_SUCCESS)
			localPointCloud.push_back(point);
		else {
			for (auto const &v : it.second) {
				SRef<Keyframe> keyframe;
				if (m_keyframesManager->getKeyframe(v.first, keyframe) != FrameworkReturnCode::_SUCCESS)
					continue;
				keyframe->removeVisibility(v.second, it.first);
			}
		}
	}
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMapManager::addCloudPoint(const SRef<CloudPoint> cloudPoint)
{
	// add point to cloud
	m_pointCloudManager->addPoint(cloudPoint);
	const std::map<uint32_t, uint32_t>& pointVisibility = cloudPoint->getVisibility();
	std::vector<uint32_t> keyframeIds;
	// add visibility to keyframes
	for (auto const &v : pointVisibility) {		
		SRef<Keyframe> keyframe;
		if (m_keyframesManager->getKeyframe(v.first, keyframe) == FrameworkReturnCode::_SUCCESS) {
			keyframeIds.push_back(v.first);
			keyframe->addVisibility(v.second, cloudPoint->getId());
		}
	}
	// update covisibility graph
	for (int i = 0; i < keyframeIds.size() - 1; i++)
		for (int j = i + 1; j < keyframeIds.size(); j++)
			m_covisibilityGraphManager->increaseEdge(keyframeIds[i], keyframeIds[j], 1);
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMapManager::removeCloudPoint(const SRef<CloudPoint> cloudPoint)
{	
	const std::map<uint32_t, uint32_t>& pointVisibility = cloudPoint->getVisibility();
	std::vector<uint32_t> keyframeIds;
	// remove visibility from keyframes
	for (auto const &v : pointVisibility) {
		SRef<Keyframe> keyframe;
		if (m_keyframesManager->getKeyframe(v.first, keyframe) == FrameworkReturnCode::_SUCCESS) {
			keyframeIds.push_back(v.first);
			keyframe->removeVisibility(v.second, cloudPoint->getId());
		}
	}
	// update covisibility graph
	for (int i = 0; i < keyframeIds.size() - 1; i++)
		for (int j = i + 1; j < keyframeIds.size(); j++)
			m_covisibilityGraphManager->decreaseEdge(keyframeIds[i], keyframeIds[j], 1);

	// remove cloud point
	m_pointCloudManager->suppressPoint(cloudPoint->getId());
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMapManager::addKeyframe(const SRef<datastructure::Keyframe> keyframe)
{
	// add to keyframe manager
	m_keyframesManager->addKeyframe(keyframe);
	// add to keyframe retriever
	m_keyframeRetriever->addKeyframe(keyframe);
	return FrameworkReturnCode();
}

FrameworkReturnCode SolARMapManager::removeKeyframe(const SRef<Keyframe> keyframe)
{
	const std::map<uint32_t, uint32_t>& keyframeVisibility = keyframe->getVisibility();
	// remove visibility of point cloud
	for (auto const &v : keyframeVisibility) {
		SRef<CloudPoint> point;
		if (m_pointCloudManager->getPoint(v.second, point) == FrameworkReturnCode::_SUCCESS) {
			// remove this cloud point if the number of visibilities is less than 2
			if (point->getVisibility().size() <= 2)
				m_pointCloudManager->suppressPoint(point->getId());
			else
				point->removeVisibility(keyframe->getId());							
		}
	}
	// remove covisibility graph
	m_covisibilityGraphManager->suppressNode(keyframe->getId());
	// remove keyframe retriever
	m_keyframeRetriever->suppressKeyframe(keyframe->getId());
	// remove keyframe
	m_keyframesManager->suppressKeyframe(keyframe->getId());
	return FrameworkReturnCode::_SUCCESS;
}

int SolARMapManager::pointCloudPruning(const std::vector<SRef<CloudPoint>> &cloudPoints)
{
	// get cloud points
	std::vector<SRef<CloudPoint>> cloudPointsPruning;
	if (cloudPoints.size() == 0) {
		m_pointCloudManager->getAllPoints(cloudPointsPruning);
	}
	else {
		cloudPointsPruning = cloudPoints;
	}

	// check reprojection error to prune cloud points
	int count(0);
	for (const auto &it : cloudPointsPruning)
		if ((!it->isValid()) || (it->getReprojError() > m_reprojErrorThres) || (it->getConfidence() < m_thresConfidence)) {
			this->removeCloudPoint(it);
			count++;
		}

	return count;
}

int SolARMapManager::keyframePruning(const std::vector<SRef<Keyframe>>& keyframes)
{
	std::vector<SRef<Keyframe>> keyframesPruning;
	if (keyframes.size() == 0) {
		m_keyframesManager->getAllKeyframes(keyframesPruning);
	}
	else {
		keyframesPruning = keyframes;
	}

	int nbRemovedKfs(0);
	for (const auto &itKf : keyframesPruning) {
		if (itKf->getId() == 0)
			continue;
		const std::map<uint32_t, uint32_t>& pcVisibility = itKf->getVisibility();
		int nbRedundantObs(0);
		for (const auto &itPC : pcVisibility) {
			uint32_t idxPC = itPC.second;
			SRef<CloudPoint> cloudPoint;
			if (m_pointCloudManager->getPoint(idxPC, cloudPoint) == FrameworkReturnCode::_SUCCESS) {
				if (cloudPoint->getVisibility().size() >= 4)
					nbRedundantObs++;
			}
		}
		if (nbRedundantObs > m_ratioRedundantObs * pcVisibility.size()) {
			this->removeKeyframe(itKf);
			nbRemovedKfs++;
		}
	}
	return nbRemovedKfs;
}

FrameworkReturnCode SolARMapManager::saveToFile() const
{
	if (m_pointCloudManager->getNbPoints() == 0)
	{
		LOG_WARNING("Map is empty: nothing to save");
	}
	else
	{
		LOG_INFO("Saving the map to file...");
		boost::filesystem::create_directories(boost::filesystem::path(m_directory.c_str()));
		LOG_DEBUG("Save identification");
		std::ofstream ofs_iden(m_directory + "/" + m_identificationFileName, std::ios::binary);
		OutputArchive oa_iden(ofs_iden);
		SRef<Identification> identification;
		m_map->getIdentification(identification);
		oa_iden << identification;
		ofs_iden.close();
		LOG_DEBUG("Save coordinate system");
		std::ofstream ofs_coor(m_directory + "/" + m_coordinateFileName, std::ios::binary);
		OutputArchive oa_coor(ofs_coor);
		SRef<CoordinateSystem> coordinateSystem;
		m_map->getCoordinateSystem(coordinateSystem);
		oa_coor << coordinateSystem;
		ofs_coor.close();
		LOG_DEBUG("Save point cloud manager");
		if (m_pointCloudManager->saveToFile(m_directory + "/" + m_pcManagerFileName) == FrameworkReturnCode::_ERROR_)
			return FrameworkReturnCode::_ERROR_;
		LOG_DEBUG("Save keyframes manager");
		if (m_keyframesManager->saveToFile(m_directory + "/" + m_kfManagerFileName) == FrameworkReturnCode::_ERROR_)
			return FrameworkReturnCode::_ERROR_;
		LOG_DEBUG("Save covisibility graph");
		if (m_covisibilityGraphManager->saveToFile(m_directory + "/" + m_covisGraphFileName) == FrameworkReturnCode::_ERROR_)
			return FrameworkReturnCode::_ERROR_;
		LOG_DEBUG("Save keyframe retriever");
		if (m_keyframeRetriever->saveToFile(m_directory + "/" + m_kfRetrieverFileName) == FrameworkReturnCode::_ERROR_)
			return FrameworkReturnCode::_ERROR_;
		LOG_INFO("Save done!");
	}

	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMapManager::loadFromFile()
{
	LOG_INFO("Loading the map from file...");
	LOG_DEBUG("Load identification");
	std::ifstream ifs_iden(m_directory + "/" + m_identificationFileName, std::ios::binary);
	if (!ifs_iden.is_open())
    {
        LOG_WARNING("Cannot load map identification file with url: {}", m_directory + "/" + m_identificationFileName);
		return FrameworkReturnCode::_ERROR_;
    }
	InputArchive ia_iden(ifs_iden);
	SRef<Identification> identification;
	ia_iden >> identification;
	m_map->setIdentification(identification);
	ifs_iden.close();
	LOG_DEBUG("Load coordinate system");
	std::ifstream ifs_coor(m_directory + "/" + m_coordinateFileName, std::ios::binary);
	if (!ifs_coor.is_open())
    {
        LOG_WARNING("Cannot load map coordinate file with url: {}", m_directory + "/" + m_coordinateFileName);
		return FrameworkReturnCode::_ERROR_;
    }
    InputArchive ia_coor(ifs_coor);
	SRef<CoordinateSystem> coordinateSystem;
	ia_coor >> coordinateSystem;
	m_map->setCoordinateSystem(coordinateSystem);
	ifs_coor.close();
	LOG_DEBUG("Load point cloud manager");
	if (m_pointCloudManager->loadFromFile(m_directory + "/" + m_pcManagerFileName) == FrameworkReturnCode::_ERROR_)
    {
        LOG_WARNING("Cannot load map point cloud manager file with url: {}", m_directory + "/" + m_pcManagerFileName);
        return FrameworkReturnCode::_ERROR_;
    }
	m_map->setPointCloud(m_pointCloudManager->getConstPointCloud());
	LOG_DEBUG("Load keyframes manager");
	if (m_keyframesManager->loadFromFile(m_directory + "/" + m_kfManagerFileName) == FrameworkReturnCode::_ERROR_)
    {
        LOG_WARNING("Cannot load map keyframe manager file with url: {}", m_directory + "/" + m_kfManagerFileName);
        return FrameworkReturnCode::_ERROR_;
    }
	m_map->setKeyframeCollection(m_keyframesManager->getConstKeyframeCollection());
	LOG_DEBUG("Load covisibility graph");
	if (m_covisibilityGraphManager->loadFromFile(m_directory + "/" + m_covisGraphFileName) == FrameworkReturnCode::_ERROR_)
    {
        LOG_WARNING("Cannot load map covisibility graph file with url: {}", m_directory + "/" + m_covisGraphFileName);
		return FrameworkReturnCode::_ERROR_;
    }
	m_map->setCovisibilityGraph(m_covisibilityGraphManager->getConstCovisibilityGraph());
	LOG_DEBUG("Load keyframe retriever");
	if (m_keyframeRetriever->loadFromFile(m_directory + "/" + m_kfRetrieverFileName) == FrameworkReturnCode::_ERROR_)
    {
        LOG_WARNING("Cannot load map keyframe retriever file with url: {}", m_directory + "/" + m_kfRetrieverFileName);
        return FrameworkReturnCode::_ERROR_;
    }
	m_map->setKeyframeRetrieval(m_keyframeRetriever->getConstKeyframeRetrieval());
	if (m_pointCloudManager->getNbPoints() == 0)
	{
		LOG_WARNING("Loaded map is empty");
	}
	LOG_INFO("Load done!");
	return FrameworkReturnCode::_SUCCESS;
}


}
}
}

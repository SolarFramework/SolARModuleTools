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

#include "SolARCovisibilityGraphManager.h"
#include "xpcf/component/ComponentFactory.h"
#include "core/Log.h"

namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::TOOLS::SolARCovisibilityGraphManager);

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {

SolARCovisibilityGraphManager::SolARCovisibilityGraphManager():ComponentBase(xpcf::toUUID<SolARCovisibilityGraphManager>())
{
   addInterface<api::storage::ICovisibilityGraphManager>(this);
   m_covisibilityGraph = xpcf::utils::make_shared<CovisibilityGraph>();
   LOG_DEBUG("SolARCovisibilityGraphManager constructor");
}

FrameworkReturnCode SolARCovisibilityGraphManager::increaseEdge(const uint32_t node1_id, const uint32_t node2_id, const float weight)
{
	m_covisibilityGraph->acquireLock();
	return m_covisibilityGraph->increaseEdge(node1_id, node2_id, weight);
}

FrameworkReturnCode SolARCovisibilityGraphManager::decreaseEdge(const uint32_t node1_id, const uint32_t node2_id, const float weight)
{
	m_covisibilityGraph->acquireLock();
	return m_covisibilityGraph->decreaseEdge(node1_id, node2_id, weight);
}

FrameworkReturnCode SolARCovisibilityGraphManager::removeEdge(const uint32_t node1_id, const uint32_t node2_id)
{
	m_covisibilityGraph->acquireLock();
	return m_covisibilityGraph->removeEdge(node1_id, node2_id);
}

FrameworkReturnCode SolARCovisibilityGraphManager::getEdge(const uint32_t node1_id, const uint32_t node2_id, float & weight) const
{
	m_covisibilityGraph->acquireLock();
	return m_covisibilityGraph->getEdge(node1_id, node2_id, weight);
}

bool SolARCovisibilityGraphManager::isEdge(const uint32_t node1_id, const uint32_t node2_id) const
{
	m_covisibilityGraph->acquireLock();
	return m_covisibilityGraph->isEdge(node1_id, node2_id);
}

FrameworkReturnCode SolARCovisibilityGraphManager::getAllNodes(std::set<uint32_t>& nodes_id) const
{
	m_covisibilityGraph->acquireLock();
	return m_covisibilityGraph->getAllNodes(nodes_id);
}

FrameworkReturnCode SolARCovisibilityGraphManager::suppressNode(const uint32_t node_id)
{
	m_covisibilityGraph->acquireLock();
	return m_covisibilityGraph->suppressNode(node_id);
}

FrameworkReturnCode SolARCovisibilityGraphManager::getNeighbors(const uint32_t node_id, const float minWeight, std::vector<uint32_t>& neighbors, const uint32_t maxNbNeighbors) const
{
	m_covisibilityGraph->acquireLock();
	return m_covisibilityGraph->getNeighbors(node_id, minWeight, neighbors, maxNbNeighbors);
}

FrameworkReturnCode SolARCovisibilityGraphManager::minimalSpanningTree(std::vector<std::tuple<uint32_t, uint32_t, float>> &edges_weights, float &minTotalWeights)
{
	m_covisibilityGraph->acquireLock();
	return m_covisibilityGraph->minimalSpanningTree(edges_weights, minTotalWeights);
}

FrameworkReturnCode SolARCovisibilityGraphManager::maximalSpanningTree(std::vector<std::tuple<uint32_t, uint32_t, float>> &edges_weights, float &maxTotalWeights)
{
	m_covisibilityGraph->acquireLock();
	return m_covisibilityGraph->maximalSpanningTree(edges_weights, maxTotalWeights);
}

FrameworkReturnCode SolARCovisibilityGraphManager::getShortestPath(const uint32_t node1_id, const uint32_t node2_id, std::vector<uint32_t> &path)
{
	m_covisibilityGraph->acquireLock();
	return m_covisibilityGraph->getShortestPath(node1_id, node2_id, path);
}

FrameworkReturnCode SolARCovisibilityGraphManager::display() const
{
	m_covisibilityGraph->acquireLock();
	return m_covisibilityGraph->display();
}

FrameworkReturnCode SolARCovisibilityGraphManager::saveToFile(const std::string& file) const
{
	std::ofstream ofs(file, std::ios::binary);
	OutputArchive oa(ofs);
	oa << m_covisibilityGraph;
	ofs.close();
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARCovisibilityGraphManager::loadFromFile(const std::string& file)
{
	std::ifstream ifs(file, std::ios::binary);
	if (!ifs.is_open())
		return FrameworkReturnCode::_ERROR_;
    InputArchive ia(ifs);
	ia >> m_covisibilityGraph;
	ifs.close();
	return FrameworkReturnCode::_SUCCESS;
}

const SRef<datastructure::CovisibilityGraph>& SolARCovisibilityGraphManager::getConstCovisibilityGraph() const
{
	return m_covisibilityGraph;
}

std::unique_lock<std::mutex> SolARCovisibilityGraphManager::getCovisibilityGraph(SRef<datastructure::CovisibilityGraph>& covisibilityGraph)
{
	covisibilityGraph = m_covisibilityGraph;
	return m_covisibilityGraph->acquireLock();
}

void SolARCovisibilityGraphManager::setCovisibilityGraph(const SRef<datastructure::CovisibilityGraph> covisibilityGraph)
{
	m_covisibilityGraph = covisibilityGraph;
}



}
}
}

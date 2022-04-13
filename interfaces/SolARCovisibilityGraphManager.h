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

#ifndef SOLARCOVISIBILITYGRAPHMANAGER_H
#define SOLARCOVISIBILITYGRAPHMANAGER_H

#include "api/storage/ICovisibilityGraphManager.h"
#include "xpcf/component/ComponentBase.h"
#include "SolARToolsAPI.h"
#include <fstream>
#include <core/SerializationDefinitions.h>

namespace SolAR {
namespace MODULES {
namespace TOOLS {
/**
 * @class SolARCovisibilityGraphManager
 * @brief A storage component to store a covisibility graph where each vertex is an id of a keyframe and each edge is weighted by the number of common cloud points between two keyframes.
 */
class SOLAR_TOOLS_EXPORT_API SolARCovisibilityGraphManager : public org::bcom::xpcf::ComponentBase,
        public SolAR::api::storage::ICovisibilityGraphManager {
public:

	SolARCovisibilityGraphManager();
    ~SolARCovisibilityGraphManager() = default;

	/// @brief This method allow to increase edge between 2 nodes
	/// @param[in] id of 1st node
	/// @param[in] id of 2nd node
	/// @param[in] weight to increase
	/// @return FrameworkReturnCode::_SUCCESS_ if the addition succeed, else FrameworkReturnCode::_ERROR.
	FrameworkReturnCode increaseEdge(uint32_t node1_id, uint32_t node2_id, float weight) override;

	/// @brief This method allow to decrease edge between 2 nodes
	/// @param[in] id of 1st node
	/// @param[in] id of 2nd node
	/// @param[in] weight to decrease
	/// @return FrameworkReturnCode::_SUCCESS_ if the addition succeed, else FrameworkReturnCode::_ERROR.
	FrameworkReturnCode decreaseEdge(uint32_t node1_id, uint32_t node2_id, float weight) override;

	/// @brief This method allow to remove an edge between 2 nodes
	/// @param[in] id of 1st node
	/// @param[in] id of 2nd node
	/// @return FrameworkReturnCode::_SUCCESS_ if the addition succeed, else FrameworkReturnCode::_ERROR.
	FrameworkReturnCode removeEdge(uint32_t node1_id, uint32_t node2_id) override;

	/// @brief This method allow to get edge between 2 nodes
	/// @param[in] id of 1st node
	/// @param[in] id of 2nd node
	/// @param[out] weight of the edge
	/// @return FrameworkReturnCode::_SUCCESS_ if the addition succeed, else FrameworkReturnCode::_ERROR.
    FrameworkReturnCode getEdge(uint32_t node1_id, uint32_t node2_id, float &weight) const override;

	/// @brief This method allow to verify that exist an edge between 2 nodes
	/// @param[in] id of 1st node
	/// @param[in] id of 2nd node
	/// @return true if exist, else false
    bool isEdge(const uint32_t node1_id, const uint32_t node2_id) const override;

	/// @brief This method allow to get all nodes of the graph
	/// @param[out] ids of all nodes
	/// @return FrameworkReturnCode::_SUCCESS_ if the addition succeed, else FrameworkReturnCode::_ERROR.
    FrameworkReturnCode getAllNodes(std::set<uint32_t> & nodes_id) const override;

	/// @brief This method allow to suppress a node of the graph
	/// @param[in] id of the node to suppress
	/// @return FrameworkReturnCode::_SUCCESS_ if the addition succeed, else FrameworkReturnCode::_ERROR.
    FrameworkReturnCode suppressNode(const uint32_t node_id) override;

	/// @brief This method allow to get neighbors of a node in the graph
	/// @param[in] node_id id of the node to get neighbors
	/// @param[in] minWeight min value between this node and a neighbor to accept
	/// @param[out] neighbors a vector of neighbors sorted to greater weighted edge.
	/// @param[in] maxNbNeighbors the maximum number of neighbors. If it is zero, find all neighbors.
	/// @return FrameworkReturnCode::_SUCCESS_ if the addition succeed, else FrameworkReturnCode::_ERROR.
    FrameworkReturnCode getNeighbors(const uint32_t node_id, const float minWeight, std::vector<uint32_t> & neighbors, const uint32_t maxNbNeighbors = 0) const override;

	/// @brief This method allow to get minimal spanning tree of the graph
	/// @param[out] edges_weights: the minimal spanning tree graph including edges with weights
	/// @param[out] minTotalWeights: cost of the minimal spanning tree graph
	/// @return FrameworkReturnCode::_SUCCESS_ if the addition succeed, else FrameworkReturnCode::_ERROR.
    FrameworkReturnCode minimalSpanningTree(std::vector<std::tuple<uint32_t, uint32_t, float>> &edges_weights, float & minTotalWeights) override;

	/// @brief This method allow to get maximal spanning tree of the graph
	/// @param[out] edges_weights: the maximal spanning tree graph including edges with weights
	/// @param[out] maxTotalWeights: cost of the maximal spanning tree graph
	/// @return FrameworkReturnCode::_SUCCESS_ if the addition succeed, else FrameworkReturnCode::_ERROR.
    FrameworkReturnCode maximalSpanningTree(std::vector<std::tuple<uint32_t, uint32_t, float>> &edges_weights, float & maxTotalWeights) override;

	/// @brief This method allow to get the shortest (by number of vertices) path between 2 nodes
	/// @param[in] id of 1st node
	/// @param[in] id of 2nd node
	/// @param[out] the shortest path
	/// @return FrameworkReturnCode::_SUCCESS_ if the addition succeed, else FrameworkReturnCode::_ERROR.
    FrameworkReturnCode getShortestPath(const uint32_t node1_id, const uint32_t node2_id, std::vector<uint32_t> &path) override;

	/// @brief This method allow to display all vertices and weighted edges of the covisibility graph
    FrameworkReturnCode display() const override;

	/// @brief This method allows to save the graph to the external file
	/// @param[in] file the file name
	/// @return FrameworkReturnCode::_SUCCESS_ if the suppression succeed, else FrameworkReturnCode::_ERROR.
    FrameworkReturnCode saveToFile(const std::string& file) const override;

	/// @brief This method allows to load the graph from the external file
	/// @param[in] file the file name
	/// @return FrameworkReturnCode::_SUCCESS_ if the suppression succeed, else FrameworkReturnCode::_ERROR.
	FrameworkReturnCode loadFromFile(const std::string& file) override;

	/// @brief This method returns the covisibility graph
	/// @return the covisibility graph
	const SRef<SolAR::datastructure::CovisibilityGraph> & getConstCovisibilityGraph() const override;

	/// @brief This method returns the covisibility graph
	/// @param[out] covisibilityGraph the covisibility graph of map
	/// @return the covisibility graph
	std::unique_lock<std::mutex> getCovisibilityGraph(SRef<SolAR::datastructure::CovisibilityGraph>& covisibilityGraph) override;

	/// @brief This method is to set the covisibility graph
	/// @param[in] covisibilityGraph the covisibility graph of map
	void setCovisibilityGraph(const SRef<SolAR::datastructure::CovisibilityGraph> covisibilityGraph) override;
    
	void unloadComponent () override final;

 private:
	 SRef<SolAR::datastructure::CovisibilityGraph>	m_covisibilityGraph;
};

}
}
}

#endif // SOLARCOVISIBILITYGRAPHMANAGER_H

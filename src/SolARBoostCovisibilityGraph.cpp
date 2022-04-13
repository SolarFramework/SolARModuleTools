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

#include "SolARBoostCovisibilityGraph.h"
#include "xpcf/component/ComponentFactory.h"
#include "core/Log.h"


namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::TOOLS::SolARBoostCovisibilityGraph);


namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {

using namespace boost;

// join 2 vertex id to make an edge id
inline static uint64_t join(uint32_t a, uint32_t b) {
    if (a > b) std::swap(a, b);
    uint64_t a_b;
    uint32_t *_a_b_16 = (uint32_t*)&a_b;
    _a_b_16[0] = b;
    _a_b_16[1] = a;
    return a_b;
}

// divides a 64bit edge id into two vertex_id
inline static std::pair<uint32_t, uint32_t> separe(uint64_t a_b) {
    uint32_t *_a_b_16 = (uint32_t*)&a_b;
    return std::make_pair(_a_b_16[1], _a_b_16[0]);
}

SolARBoostCovisibilityGraph::SolARBoostCovisibilityGraph():ComponentBase(xpcf::toUUID<SolARBoostCovisibilityGraph>())
{
    addInterface<api::storage::ICovisibilityGraphManager>(this);
	LOG_DEBUG("SolARBoostCovisibilityGraph constructor");
}

FrameworkReturnCode SolARBoostCovisibilityGraph::increaseEdge(const uint32_t node1_id, const uint32_t node2_id, const float weight)
{
    // std::unique_lock<std::mutex> lock(m_mutex);
	if (node1_id == node2_id)
		return FrameworkReturnCode::_ERROR_;	

    if(!isNode(node1_id))
        addNode(node1_id);
    if(!isNode(node2_id))
        addNode(node2_id);

    if( isNode(node1_id) &&  isNode(node2_id))
    {
        vertex_t vertex_id_1  = m_map[node1_id];
        vertex_t vertex_id_2  = m_map[node2_id];
        edge_info_t edge_info = boost::edge(vertex_id_1, vertex_id_2, m_graph);
        if(edge_info.second)
        {
            edge_t edge_id = edge_info.first;
            EdgeProperties& edgeProperties = m_graph[edge_id];
            edgeProperties.weight = edgeProperties.weight + weight;
        }else{
            // unexistant edge
            addEdge(node1_id, node2_id, weight);
        }
    }

	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARBoostCovisibilityGraph::decreaseEdge(const uint32_t node1_id, const uint32_t node2_id, const float weight)
{
    // std::unique_lock<std::mutex> lock(m_mutex);
	if (node1_id == node2_id)
		return FrameworkReturnCode::_ERROR_;

    if( isNode(node1_id) &&  isNode(node2_id))
    {
        vertex_t vertex_id_1  = m_map[node1_id];
        vertex_t vertex_id_2  = m_map[node2_id];
        edge_info_t edge_info = boost::edge(vertex_id_1, vertex_id_2, m_graph);
        if(edge_info.second)
        {
            edge_t edge_id = edge_info.first;
            EdgeProperties& edgeProperties = m_graph[edge_id];
            if (edgeProperties.weight > weight){
                edgeProperties.weight = edgeProperties.weight - weight;
            }else{
                removeEdge(node1_id,node2_id);
            }

        } // else the edge does not exist and cannot be decreased

    } // else the nodes don't exist


	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARBoostCovisibilityGraph::removeEdge(const uint32_t node1_id, const uint32_t node2_id)
{
    // std::unique_lock<std::mutex> lock(m_mutex);
    if( isNode(node1_id) &&  isNode(node2_id))
    {
        vertex_t vertex_id_1  = m_map[node1_id];
        vertex_t vertex_id_2  = m_map[node2_id];
        edge_info_t edge_info = boost::edge(vertex_id_1, vertex_id_2, m_graph);
        if(edge_info.second)
        {
            edge_t edge_id = edge_info.first;
            m_graph.remove_edge(edge_id);
        }else{
            // unexistant edge
            return FrameworkReturnCode::_ERROR_;
        }
    }else{
        // unexistant nodes
        return FrameworkReturnCode::_ERROR_;
    }

	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARBoostCovisibilityGraph::getEdge(uint32_t node1_id, uint32_t node2_id, float & weight) const
{
    // std::unique_lock<std::mutex> lock(m_mutex);

    if( isNode(node1_id) &&  isNode(node2_id))
    {
        vertex_t vertex_id_1  = m_map.at(node1_id);
        vertex_t vertex_id_2  = m_map.at(node2_id);
        edge_info_t edge_info = boost::edge(vertex_id_1, vertex_id_2, m_graph);
        if(edge_info.second)
        {
            edge_t edge_id = edge_info.first;
            const EdgeProperties& edgeProperties = m_graph[edge_id];
            weight = edgeProperties.weight;
        }else{
            // unexistant edge
            return FrameworkReturnCode::_ERROR_;
        }
    }else{
        // unexistant nodes
        return FrameworkReturnCode::_ERROR_;
    }

	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARBoostCovisibilityGraph::getAllNodes(std::set<uint32_t>& nodes_id) const
{
    // std::unique_lock<std::mutex> lock(m_mutex);
    std::pair<vertex_iterator_t, vertex_iterator_t> it_vertex = vertices(m_graph);
    nodes_id.clear();
    for( ; it_vertex.first != it_vertex.second; ++it_vertex.first)
       nodes_id.insert(get(boost::vertex_bundle, m_graph)[*it_vertex.first].frame_id) ;

	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARBoostCovisibilityGraph::suppressNode(const uint32_t node_id)
{
    // std::unique_lock<std::mutex> lock(m_mutex);
    if(isNode(node_id))
    {
        // delete all edges connected to this node
        vertex_t vertex_id = m_map[node_id];
        clear_vertex(vertex_id, m_graph);
        // removing node
        remove_vertex(vertex_id, m_graph);
        // remove reference from map
        m_map.erase(node_id);

    }// else is already removed
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARBoostCovisibilityGraph::getNeighbors(uint32_t node_id, float minWeight, std::vector<uint32_t>& neighbors, const uint32_t maxNbNeighbors) const
{
    //std::unique_lock<std::mutex> lock(m_mutex);
    // this version does not sort neighboors TODO ?
    neighbors.clear();
	int nbNeighbors = std::numeric_limits<int>::max();
	if (maxNbNeighbors > 0)
		nbNeighbors = maxNbNeighbors;
    if(isNode(node_id))
    {
        vertex_t vertex_id = m_map.at(node_id);
        std::pair<in_edge_iterator_t, in_edge_iterator_t> it_edge = in_edges(vertex_id, m_graph);
        for( ; it_edge.first != it_edge.second; ++it_edge.first)
        {
           edge_t edge_id = *it_edge.first;
           EdgeProperties edge_properties = get(boost::edge_bundle, m_graph)[edge_id];
           if(edge_properties.weight > minWeight)
           {
               vertex_t v1    = source(edge_id, m_graph);
               vertex_t v2    = target(edge_id, m_graph);
               if(node_id==m_graph[v1].frame_id)
               {
                    neighbors.push_back(m_graph[v2].frame_id);
               }
			   else{
                    neighbors.push_back(m_graph[v1].frame_id);
               }
			   if (neighbors.size() == nbNeighbors)
				   break;
           }
        }
    }else{
        // no node
        return FrameworkReturnCode::_ERROR_;
    }

	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARBoostCovisibilityGraph::minimalSpanningTree(std::vector<std::tuple<uint32_t, uint32_t, float>> &edges_weights, float &minTotalWeights)
{
    // std::unique_lock<std::mutex> lock(m_mutex);
    //
    std::vector<edge_t> spanning_tree;
    property_map<CoGraph, float EdgeProperties::*>::type propmapWeight = boost::get(&EdgeProperties::weight, m_graph); // use unit weight but can use also coVisibility weight
    IndexMap        mapIndex;
    associative_property_map<IndexMap> propmapIndex(mapIndex);
    int c = 0;
    CoGraph::vertex_iterator v, vend;
    for (boost::tie(v, vend) = boost::vertices(m_graph); v != vend; ++v)
    {
        // vertex_t v_id = map[graph[*v].frame_id]; // useless
        vertex_t v_id = *v;
        boost::put(propmapIndex,       v_id, c++);
    }
    kruskal_minimum_spanning_tree(m_graph, std::back_inserter(spanning_tree),weight_map(propmapWeight).vertex_index_map(propmapIndex));
    minTotalWeights = 0.0;
    edges_weights.clear();
    for (std::vector < edge_t >::iterator ei = spanning_tree.begin(); ei != spanning_tree.end(); ++ei)
    {
        edge_t edge_id = *ei;
        EdgeProperties edge_properties = boost::get(boost::edge_bundle, m_graph)[edge_id];
        vertex_t v1         = boost::source(edge_id, m_graph);
        vertex_t v2         = boost::target(edge_id, m_graph);
        uint32_t frame_id_1 = m_graph[v1].frame_id;
        uint32_t frame_id_2 = m_graph[v2].frame_id;
        std::tuple<uint32_t, uint32_t, float> edge_tuple = std::make_tuple(frame_id_1,frame_id_2,edge_properties.weight);
        edges_weights.push_back(edge_tuple);
        minTotalWeights+=edge_properties.weight;
    }

	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARBoostCovisibilityGraph::maximalSpanningTree(std::vector<std::tuple<uint32_t, uint32_t, float>> &edges_weights, float &maxTotalWeights)
{
    // std::unique_lock<std::mutex> lock(m_mutex);
    // inputs
    std::vector<edge_t> spanning_tree;

    // additive inverse of each weight
    CoGraph::edge_iterator e, eend;
    for (boost::tie(e, eend) = edges(m_graph); e != eend; ++e)
    {
        edge_t e_id = *e;
        EdgeProperties& edgeProperties = m_graph[e_id];
        edgeProperties.weight = -1.0f * edgeProperties.weight;
    }

    property_map<CoGraph, float EdgeProperties::*>::type propmapAdditiveInverseWeight = get(&EdgeProperties::weight, m_graph); // use unit weight but can use also coVisibility weight
    IndexMap        mapIndex;
    associative_property_map<IndexMap>  propmapIndex(mapIndex);

    int c = 0;
    CoGraph::vertex_iterator v, vend;
    for (boost::tie(v, vend) = boost::vertices(m_graph); v != vend; ++v)
    {
        // vertex_t v_id = map[graph[*v].frame_id]; // useless
        vertex_t v_id = *v;
        boost::put(propmapIndex,       v_id, c++);
    }

    // minimum spanning tree on additive inversed weight
    kruskal_minimum_spanning_tree(m_graph, std::back_inserter(spanning_tree),weight_map(propmapAdditiveInverseWeight).vertex_index_map(propmapIndex));

    // additive inverse of each weight back
    for (boost::tie(e, eend) = edges(m_graph); e != eend; ++e)
    {
        edge_t e_id = *e;
        EdgeProperties& edgeProperties = m_graph[e_id];
        edgeProperties.weight          = -1.0f * edgeProperties.weight;
    }

    //
    maxTotalWeights = 0.0;
    for (std::vector < edge_t >::iterator ei = spanning_tree.begin(); ei != spanning_tree.end(); ++ei)
    {
        edge_t edge_id = *ei;
        EdgeProperties edge_properties = get(boost::edge_bundle, m_graph)[edge_id];
        vertex_t v1         = boost::source(edge_id, m_graph);
        vertex_t v2         = boost::target(edge_id, m_graph);
        uint32_t frame_id_1 = m_graph[v1].frame_id;
        uint32_t frame_id_2 = m_graph[v2].frame_id;
        std::tuple<uint32_t, uint32_t, float> edge_tuple = std::make_tuple(frame_id_1,frame_id_2,edge_properties.weight);
        edges_weights.push_back(edge_tuple);
        maxTotalWeights += edge_properties.weight;
    }

	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARBoostCovisibilityGraph::getShortestPath(uint32_t node1_id, uint32_t node2_id, std::vector<uint32_t> &path)
{
    // std::unique_lock<std::mutex> lock(m_mutex);
    if ( isNode(node1_id) && isNode(node2_id))
    {
        vertex_t _source      = m_map[node1_id];
        // inputs
        vertex_t _destination = m_map[node2_id];
        boost::property_map<CoGraph, float EdgeProperties::*>::type propmapWeight = boost::get(&EdgeProperties::unit, m_graph); // use unit weight but can use also coVisibility weight
        IndexMap        mapIndex;
        // outputs
        PredecessorMap  mapPredecessor;
        DistanceMap     mapDistance;
        boost::associative_property_map<IndexMap>       propmapIndex(mapIndex);
        boost::associative_property_map<PredecessorMap> propmapPredecessor(mapPredecessor);
        boost::associative_property_map<DistanceMap>    propmapDistance(mapDistance);
        int c = 0;
        CoGraph::vertex_iterator v, vend;
        for (boost::tie(v, vend) = vertices(m_graph); v != vend; ++v)
        {
            // vertex_t v_id = map[m_graph[*v].frame_id]; // useless
            vertex_t v_id = *v;
            boost::put(propmapIndex,       v_id, c++);
            boost::put(propmapPredecessor, v_id, v_id);
            boost::put(propmapDistance,    v_id, 1.0f);
        }
        //
        dijkstra_shortest_paths(m_graph, _destination,
                                 weight_map(propmapWeight)
                                .vertex_index_map(propmapIndex)
                                .predecessor_map(propmapPredecessor)
                                .distance_map(propmapDistance)); //

        //
        path.clear();
        vertex_t _current = _source;
        path.push_back( m_graph[_current].frame_id );
        vertex_t _next = propmapPredecessor[_source];
        while(_current != _next )
        {
            _current = _next;
            path.push_back( m_graph[_current].frame_id );
            _next = propmapPredecessor[_current];
        }
        // totalCost = propmapDistance[_source] ;
        if(_current == _destination)
        {
            // short path found
        }else{
            // unreachable destination
            path.clear();
            // totalCost = -1.0;
            return FrameworkReturnCode::_ERROR_;
        }
    }
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARBoostCovisibilityGraph::display() const
{
    // std::unique_lock<std::mutex> lock(m_mutex);
    // std::unique_lock<std::mutex> lock(m_boost_cg_mutex);
	// display vertices
    std::pair<vertex_iterator_t, vertex_iterator_t> it_vertex = vertices(m_graph);
    for( ; it_vertex.first != it_vertex.second; ++it_vertex.first)
    {
       std::cout << get(boost::vertex_bundle, m_graph)[*it_vertex.first].frame_id << " ";
    }
    std::cout << std::endl;
    std::pair<edge_iterator_t, edge_iterator_t> it_edge = edges(m_graph);
    for( ; it_edge.first != it_edge.second; ++it_edge.first)
    {
       edge_t edge_id = *it_edge.first;
       EdgeProperties edge_properties = get(boost::edge_bundle, m_graph)[edge_id];
       vertex_t v1    = source(edge_id, m_graph);
       vertex_t v2    = target(edge_id, m_graph);
       std::cout      << m_graph[v1].frame_id<<" - "<<m_graph[v2].frame_id << " : " << edge_properties.weight << std::endl;
    }

	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARBoostCovisibilityGraph::saveToFile(const std::string& file) const
{
    // create a covisibility graph datastructure
	SRef<CovisibilityGraph> covisibilityGraph = xpcf::utils::make_shared<CovisibilityGraph>();
	std::set<uint32_t> nodeIds;
	this->getAllNodes(nodeIds);

	// copy data
	for (auto it1 = nodeIds.begin(); it1 != nodeIds.end(); it1++)
		for (auto it2 = std::next(it1); it2 != nodeIds.end(); it2++) {
			float weight;
			if (this->getEdge(*it1, *it2, weight) == FrameworkReturnCode::_SUCCESS)
				covisibilityGraph->increaseEdge(*it1, *it2, weight);
		}
    
	std::ofstream ofs(file, std::ios::binary);
	OutputArchive oa(ofs);
    oa << covisibilityGraph;
    ofs.close();

    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARBoostCovisibilityGraph::loadFromFile(const std::string& file)
{
	// create a covisibility graph datastructure
	SRef<CovisibilityGraph> covisibilityGraph = xpcf::utils::make_shared<CovisibilityGraph>();

	// load data
	std::ifstream ifs(file, std::ios::binary);
	if (!ifs.is_open())
		return FrameworkReturnCode::_ERROR_;
	InputArchive ia(ifs);
    ia >> covisibilityGraph;
    ifs.close();

	// copy data
	std::set<uint32_t> nodeIds;
	covisibilityGraph->getAllNodes(nodeIds);

	// copy data
	for (auto it1 = nodeIds.begin(); it1 != nodeIds.end(); it1++)
		for (auto it2 = std::next(it1); it2 != nodeIds.end(); it2++) {
			float weight;
			if (covisibilityGraph->getEdge(*it1, *it2, weight) == FrameworkReturnCode::_SUCCESS)
				this->addEdge(*it1, *it2, weight);
		}
    return FrameworkReturnCode::_SUCCESS;
}

const SRef<datastructure::CovisibilityGraph>& SolARBoostCovisibilityGraph::getConstCovisibilityGraph() const
{
	LOG_ERROR("Not implemented");
	return nullptr;
}

std::unique_lock<std::mutex> SolARBoostCovisibilityGraph::getCovisibilityGraph(SRef<datastructure::CovisibilityGraph>& covisibilityGraph)
{
	LOG_ERROR("Not implemented");
	return std::unique_lock<std::mutex>();
}

void SolARBoostCovisibilityGraph::setCovisibilityGraph(const SRef<datastructure::CovisibilityGraph> covisibilityGraph)
{
	LOG_ERROR("Not implemented");
}

bool SolARBoostCovisibilityGraph::isEdge(const uint32_t node1_id, const uint32_t node2_id) const
{
    // std::unique_lock<std::mutex> lock(m_boost_cg_mutex);
    bool is_edge = false;
    if( isNode(node1_id) && isNode(node2_id))
    {
        vertex_t vertex_id_1  = m_map.at(node1_id);
        vertex_t vertex_id_2  = m_map.at(node2_id);
        edge_info_t edge_info = edge(vertex_id_1, vertex_id_2, m_graph);
        is_edge = edge_info.second;
    }
    return is_edge;
}

FrameworkReturnCode SolARBoostCovisibilityGraph::addEdge(const uint32_t node_id_1, const uint32_t node_id_2, const float weight)
{
    // std::unique_lock<std::mutex> lock(m_boost_cg_mutex);
    if ( !isNode(node_id_1))
        addNode(node_id_1);
    if ( !isNode(node_id_2))
        addNode(node_id_2);

    vertex_t vertex_id_1  = m_map[node_id_1];
    vertex_t vertex_id_2  = m_map[node_id_2];
    if(SolARBoostCovisibilityGraph::isEdge(node_id_1, node_id_2))
    {
        edge_info_t edge_info = boost::edge(vertex_id_1, vertex_id_2, m_graph);
        edge_t edge_id                                      = edge_info.first;
        EdgeProperties& edgeProperties = m_graph[edge_id];
        edgeProperties.weight = weight;

    }else{
        boost::add_edge(vertex_id_1, vertex_id_2, EdgeProperties(weight), m_graph);
    }
    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARBoostCovisibilityGraph::addNode(const uint32_t node_id)
{
    // std::unique_lock<std::mutex> lock(m_boost_cg_mutex);
    if(!isNode(node_id))
    {
        vertex_t vertex_id = add_vertex(VertexProperties(node_id), m_graph);
        m_map[node_id]       = vertex_id;
    }// else the node is already present
    return FrameworkReturnCode::_SUCCESS;
}

bool SolARBoostCovisibilityGraph::isNode(const uint32_t node_id) const
{
    // std::unique_lock<std::mutex> lock(m_boost_cg_mutex);
    return !(m_map.find(node_id) == m_map.end()) ;
}

FrameworkReturnCode SolARBoostCovisibilityGraph::clear()
{
    // std::unique_lock<std::mutex> lock(m_boost_cg_mutex);
    m_map.clear();
    m_graph.clear();
    return FrameworkReturnCode::_SUCCESS;
}






}
}
}

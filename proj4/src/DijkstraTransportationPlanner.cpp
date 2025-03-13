#include "DijkstraTransportationPlanner.h"
#include "DijkstraPathRouter.h"
#include "BusSystemIndexer.h"
#include "GeographicUtils.h"
#include "StreetMap.h"
#include <unordered_map>
#include <set>
#include <vector>
#include <memory>
#include <algorithm>

// Define the SImplementation struct
struct CDijkstraTransportationPlanner::SImplementation {
        
    std::unordered_map<CStreetMap::TNodeID, CPathRouter::TVertexID> NodeToVertex;
    std::unordered_map<CPathRouter::TVertexID, CStreetMap::TNodeID> VertexToNode;
    std::vector<CStreetMap::TNodeID> SortedNodeIDs;
    std::shared_ptr<CStreetMap> DStreetMap;
    std::shared_ptr<CBusSystemIndexer> DBusSystemIndexer;
    double DWalkSpeed;
    double DBikeSpeed;
    double DDefaultSpeedLimit;
    double DBusStopTime;
    int DPrecomputeTime;

    void ReadSortNodeIDs() {
        std::size_t NumNodes = DStreetMap->NodeCount();
        for (std::size_t Index = 0; Index < NumNodes; Index++) {
            auto Node = DStreetMap->NodeByIndex(Index);
            SortedNodeIDs.push_back(Node->ID());
        }
        std::sort(SortedNodeIDs.begin(), SortedNodeIDs.end());
        return;
    }

    SImplementation(std::shared_ptr<SConfiguration> config) {
        // Get the configuration parameters
        DStreetMap = config->StreetMap();
        DBusSystemIndexer = std::make_shared<CBusSystemIndexer>(config->BusSystem());
        DWalkSpeed = config->WalkSpeed();
        DBikeSpeed = config->BikeSpeed();
        DDefaultSpeedLimit = config->DefaultSpeedLimit();
        DBusStopTime = config->BusStopTime();
        DPrecomputeTime = config->PrecomputeTime();
        ReadSortNodeIDs();
    }

    std::size_t NodeCount() const noexcept {
        return DStreetMap->NodeCount();
    }

    std::shared_ptr<CStreetMap::SNode> SortedNodeByIndex(std::size_t index){
        if (!(index < NodeCount())) {
            return nullptr;
        }
        auto NodeID = SortedNodeIDs[index];
        return DStreetMap->NodeByID(NodeID);            
    }

    void CreateStreetNodes(std::shared_ptr<CDijkstraPathRouter> pathRouter) {
        for (auto NodeID : SortedNodeIDs) {
            auto VertexID = pathRouter->AddVertex(NodeID);
            NodeToVertex[NodeID] = VertexID;
            VertexToNode[VertexID] = NodeID;
        }
    }                
    
    void CreateShortestPathEdges(std::shared_ptr<CDijkstraPathRouter> pathRouter){
        std::size_t NumWays = DStreetMap->WayCount();
        for (std::size_t Index = 0; Index < NumWays; Index++) {
            auto Way = DStreetMap->WayByIndex(Index);
            std::size_t NumNodes = Way->NodeCount();
            
            bool oneWay = false;
            if (Way->GetAttribute("oneway") == "yes") {
                oneWay = true;
            }

            for (std::size_t NodeIndex = 0; NodeIndex < NumNodes - 1; NodeIndex++) {
                auto Node1 = Way->GetNodeID(NodeIndex); // This returns an ID, not the node itself
                auto Node2 = Way->GetNodeID(NodeIndex + 1); // This returns an ID, not the node itself
                
                // Find the vertices corresponding to the nodes
                auto Node1Vertex = NodeToVertex.find(Node1);
                auto Node2Vertex = NodeToVertex.find(Node2);

                if (Node1Vertex != NodeToVertex.end() && Node2Vertex != NodeToVertex.end()) {
                    auto Node1Location = DStreetMap->NodeByID(Node1)->Location();
                    auto Node2Location = DStreetMap->NodeByID(Node2)->Location();
                    auto EdgeWeight = SGeographicUtils::HaversineDistanceInMiles(Node1Location, Node2Location);
                    
                    if (oneWay) {
                        pathRouter->AddEdge(Node1Vertex->second, Node2Vertex->second, EdgeWeight, false);
                        continue;
                    } else {
                        pathRouter->AddEdge(Node1Vertex->second, Node2Vertex->second, EdgeWeight, true);
                        continue;
                    }
                }
            }
        }
    }

    double FindShortestPath(CStreetMap::TNodeID src, CStreetMap::TNodeID dest, std::vector<CStreetMap::TNodeID>& path) {
        auto ShortestPathRouter = std::make_shared<CDijkstraPathRouter>();
        CreateStreetNodes(ShortestPathRouter);
        CreateShortestPathEdges(ShortestPathRouter);
        return ShortestPathRouter->FindShortestPath(NodeToVertex[src], NodeToVertex[dest], path);
    }

    // // Find the NodeID of the bus stop & the vertex ID 
    // void CreateFastestPathBusEdges(std::shared_ptr<CDijkstraPathRouter> pathRouter){
    //     auto NumRoutes = DBusSystemIndexer->RouteCount();
    //     for (std::size_t Index = 0; Index < NumRoutes; Index++) {
    //         auto Route = DBusSystemIndexer->RouteByIndex(Index);
    //         auto NumStops = Route->StopCount();
    //         for (std::size_t StopIndex = 0; StopIndex < NumStops - 1; StopIndex++) {
    //             auto Stop1NodeID = Route->NodeID(StopIndex);
    //             auto Stop2NodeID = Route->NodeID(StopIndex + 1);
    //             auto Stop1Vertex = NodeToVertex.find(Stop1NodeID);
    //             auto Stop2Vertex = NodeToVertex.find(Stop2NodeID);
    //             if (Stop1Vertex != NodeToVertex.end() && Stop2Vertex != NodeToVertex.end()) {
    //                 auto Stop1Location = DStreetMap->NodeByID(Stop1NodeID)->Location();
    //                 auto Stop2Location = DStreetMap->NodeByID(Stop2NodeID)->Location();
    //                 auto EdgeWeight = SGeographicUtils::HaversineDistanceInMiles(Stop1Location, Stop2Location);
    //                 EdgeWeight /= DefaultSpeedLimit;
    //                 EdgeWeight += (DBusStopTime / 3600);
    //                 pathRouter->AddEdge(Stop1Vertex->second, Stop2Vertex->second, EdgeWeight, false);
    //                 BusEdges.insert(std::make_pair(Stop1NodeID, Stop2NodeID));
    //             }
    //         }
    //     }
    // } 

    void CreateFastestPathEdgesBusWalk(std::shared_ptr<CDijkstraPathRouter> pathRouter){
        std::size_t NumWays = DStreetMap->WayCount();
        for (std::size_t Index = 0; Index < NumWays; Index++) {
            auto Way = DStreetMap->WayByIndex(Index);
            std::size_t NumNodes = Way->NodeCount();
            
            auto SpeedLimit = DDefaultSpeedLimit;
            if (Way->GetAttribute("maxspeed") != "") {
                SpeedLimit = std::stod(Way->GetAttribute("maxspeed"));
            }

            for (std::size_t NodeIndex = 0; NodeIndex < NumNodes - 1; NodeIndex++) {
                auto Node1 = Way->GetNodeID(NodeIndex); // This returns an ID, not the node itself
                auto Node2 = Way->GetNodeID(NodeIndex + 1); // This returns an ID, not the node itself
                
                // Find the vertices corresponding to the nodes
                auto Node1Vertex = NodeToVertex.find(Node1);
                auto Node2Vertex = NodeToVertex.find(Node2);

                if (Node1Vertex != NodeToVertex.end() && Node2Vertex != NodeToVertex.end()) {
                    auto Node1Location = DStreetMap->NodeByID(Node1)->Location();
                    auto Node2Location = DStreetMap->NodeByID(Node2)->Location();
                    auto Distance = SGeographicUtils::HaversineDistanceInMiles(Node1Location, Node2Location);
                    auto WalkEdgeWeight = Distance / DWalkSpeed;
                    pathRouter->AddEdge(Node1Vertex->second, Node2Vertex->second, WalkEdgeWeight, true);

                    if (DBusSystemIndexer->RouteBetweenNodeIDs(Node1, Node2)) {
                        auto BusEdgeWeight = (Distance / SpeedLimit) + (DBusStopTime / 3600);
                        pathRouter->AddEdge(Node1Vertex->second, Node2Vertex->second, BusEdgeWeight, false);
                    }
                }
            }
        }
    }

    void CreateFastestPathBikingEdges(std::shared_ptr<CDijkstraPathRouter> pathRouter){
        std::size_t NumWays = DStreetMap->WayCount();
        for (std::size_t Index = 0; Index < NumWays; Index++) {
            auto Way = DStreetMap->WayByIndex(Index);
            std::size_t NumNodes = Way->NodeCount();

            if (Way->GetAttribute("bicyle") == "no") {
                continue;
            }

            bool oneWay = false;
            if (Way->GetAttribute("oneway") == "yes") {
                oneWay = true;
            }
            
            for (std::size_t NodeIndex = 0; NodeIndex < NumNodes - 1; NodeIndex++) {
                auto Node1 = Way->GetNodeID(NodeIndex); // This returns an ID, not the node itself
                auto Node2 = Way->GetNodeID(NodeIndex + 1); // This returns an ID, not the node itself
                
                // Find the vertices corresponding to the nodes
                auto Node1Vertex = NodeToVertex.find(Node1);
                auto Node2Vertex = NodeToVertex.find(Node2);

                if (Node1Vertex != NodeToVertex.end() && Node2Vertex != NodeToVertex.end()) {
                    auto Node1Location = DStreetMap->NodeByID(Node1)->Location();
                    auto Node2Location = DStreetMap->NodeByID(Node2)->Location();
                    auto EdgeWeight = SGeographicUtils::HaversineDistanceInMiles(Node1Location, Node2Location);
                    EdgeWeight /= DBikeSpeed;
                    if (oneWay) {
                        pathRouter->AddEdge(Node1Vertex->second, Node2Vertex->second, EdgeWeight, false);
                        continue;
                    } else {
                        pathRouter->AddEdge(Node1Vertex->second, Node2Vertex->second, EdgeWeight, true);
                        continue;
                    }
                }
            }
        }
    }

    double FindFastestPath(CStreetMap::TNodeID src, CStreetMap::TNodeID dest, std::vector<TTripStep>& path) {
        std::vector <CPathRouter::TVertexID> BusWalkPath;
        std::vector <CPathRouter::TVertexID> BikePath;

        auto WalkBusRouter = std::make_shared<CDijkstraPathRouter>();
        CreateStreetNodes(WalkBusRouter);
        CreateFastestPathEdgesBusWalk(WalkBusRouter);
        auto fastestWalkBusPath = WalkBusRouter->FindShortestPath(NodeToVertex[src], NodeToVertex[dest], BusWalkPath);

        auto BikeRouter = std::make_shared<CDijkstraPathRouter>();
        CreateStreetNodes(BikeRouter);
        CreateFastestPathBikingEdges(BikeRouter);
        auto fastestBikePath = BikeRouter->FindShortestPath(NodeToVertex[src], NodeToVertex[dest], BikePath);

        path.clear();
        if (fastestWalkBusPath < fastestBikePath) {
            auto PathLength = BusWalkPath.size();
            for (std::size_t Index = 0; Index < PathLength - 1; Index++) {
                auto Node = VertexToNode[BusWalkPath[Index]];
                auto NextNode = VertexToNode[BusWalkPath[Index + 1]];
                if (DBusSystemIndexer->RouteBetweenNodeIDs(Node, NextNode)) {
                    path.push_back({CTransportationPlanner::ETransportationMode::Bus, Node});
                    path.push_back({CTransportationPlanner::ETransportationMode::Bus, NextNode});
                    Index++;
                } else {
                    path.push_back({CTransportationPlanner::ETransportationMode::Walk, Node});
                }
            }
            return fastestWalkBusPath;                
        } else {
            for (auto Vertex : BikePath) {
                path.push_back({CTransportationPlanner::ETransportationMode::Bike, VertexToNode[Vertex]});
            }
            return fastestBikePath;
        }
    }
};

CDijkstraTransportationPlanner::CDijkstraTransportationPlanner(std::shared_ptr<SConfiguration> config) {
    DImplementation = std::make_unique<SImplementation>(config);
}

CDijkstraTransportationPlanner::~CDijkstraTransportationPlanner() = default;

std::size_t CDijkstraTransportationPlanner::NodeCount() const noexcept {
    return DImplementation->NodeCount();
}

std::shared_ptr<CStreetMap::SNode> CDijkstraTransportationPlanner::SortedNodeByIndex(std::size_t index) const noexcept {
    return DImplementation->SortedNodeByIndex(index);
}

double CDijkstraTransportationPlanner::FindShortestPath(CStreetMap::TNodeID src, CStreetMap::TNodeID dest, std::vector<CStreetMap::TNodeID>& path) {
    return DImplementation->FindShortestPath(src, dest, path);
}

double CDijkstraTransportationPlanner::FindFastestPath(CStreetMap::TNodeID src, CStreetMap::TNodeID dest, std::vector<TTripStep>& path) {
    return DImplementation->FindFastestPath(src, dest, path);
}

bool CDijkstraTransportationPlanner::GetPathDescription(const std::vector<TTripStep>& path, std::vector<std::string>& desc) const {
    return false;
}
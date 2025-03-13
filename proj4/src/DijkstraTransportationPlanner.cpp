#include "DijkstraTransportationPlanner.h"
#include "DijkstraPathRouter.h"
#include "BusSystemIndexer.h"
#include "GeographicUtils.h"
#include "StreetMap.h"
#include <unordered_map>
#include <set>
#include <map>
#include <vector>
#include <memory>
#include <algorithm>

// Define the SImplementation struct
struct CDijkstraTransportationPlanner::SImplementation {
        
    std::unordered_map<CStreetMap::TNodeID, CPathRouter::TVertexID> NodeToVertex;
    std::unordered_map<CPathRouter::TVertexID, CStreetMap::TNodeID> VertexToNode;
    std::vector<CStreetMap::TNodeID> SortedNodeIDs;
    std::map<std::pair<CStreetMap::TNodeID, CStreetMap::TNodeID>, CStreetMap::TWayID> NodePairToWay;
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

    void StoreWays(){
        std::size_t NumWays = DStreetMap->WayCount();
        for (std::size_t Index = 0; Index < NumWays; Index++) {
            auto Way = DStreetMap->WayByIndex(Index);
            std::size_t NumNodes = Way->NodeCount();
            for (std::size_t NodeIndex = 0; NodeIndex < NumNodes - 1; NodeIndex++) {
                auto Node1 = Way->GetNodeID(NodeIndex);
                auto Node2 = Way->GetNodeID(NodeIndex + 1);
                NodePairToWay[std::make_pair(Node1, Node2)] = Way->ID();
            }
        }
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
        StoreWays();
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
        std::vector<CStreetMap::TNodeID> tempPath;
        auto pathDist = ShortestPathRouter->FindShortestPath(NodeToVertex[src], NodeToVertex[dest], tempPath);
        path.clear();
        for (auto Vertex : tempPath) {
            path.push_back(VertexToNode[Vertex]);
        }
        return pathDist;
    }

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
            path.push_back({CTransportationPlanner::ETransportationMode::Walk, VertexToNode[BusWalkPath[0]]});
            for (std::size_t Index = 1; Index < PathLength; Index++) {
                auto Node = VertexToNode[BusWalkPath[Index]];
                auto PrevNode = VertexToNode[BusWalkPath[Index - 1]];
                if (DBusSystemIndexer->RouteBetweenNodeIDs(PrevNode, Node)) {
                    path.push_back({CTransportationPlanner::ETransportationMode::Bus, Node});
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

    // bool GetPathWays(const std::vector<TTripStep>& path, std::vector<CStreetMap::TWayID>& Ways) const{
    //     auto PathLength = path.size();
    //     Ways.clear();
    //     for (std::size_t CurrentIndex = 1; CurrentIndex < PathLength; CurrentIndex++){
    //         auto CurrentNodeID = path[CurrentIndex].second;
    //         auto PrevNodeID = path[CurrentIndex - 1].second;
    //         auto PairWayID = NodePairToWay.find(std::make_pair(PrevNodeID, CurrentNodeID));
    //         if (PairWayID == NodePairToWay.end()) {
    //             return false;
    //         }
    //         Ways.push_back(PairWayID->second);
    //     }
    //     return true;                         
    // }
    
    // bool GetPathDescription(const std::vector<TTripStep>& path, std::vector<std::string>& desc) const {
    //     auto PathLength = path.size();
    //     CStreetMap::TNodeID StartNodeID = path[0].second;
    //     desc.push_back("Start at " + GeographicUtils::ConvertLLToDMS(DStreetMap->NodeByID(StartNodeID)->Location()));

        
    //     for (std::size_t CurrentIndex = 1; CurrentIndex < PathLength; CurrentIndex++) {
    //         auto CurrentMode = path[CurrentIndex].first;
    //         auto CurrentNodeID = path[CurrentIndex].second;
    //         auto PrevNodeID = path[CurrentIndex - 1].second;
    //         auto PairWayID = NodePairToWay.find(std::make_pair(PrevNodeID, CurrentNodeID));
    //         if (PairWayID == NodePairToWay.end()) {
    //             return false;
    //         }
    //         auto WayID = PairWayID->second;
    //         auto Way = DStreetMap->WayByID(WayID);
    //         auto StreetName = Way->GetAttribute("name");


        //     switch (CurrentMode) {
        //         case CTransportationPlanner::ETransportationMode::Walk:
                                    
        //             modeStr = "Walk";
        //             break;
        //         case CTransportationPlanner::ETransportationMode::Bike:
        //             modeStr = "Bike";
        //             break;
        //         case CTransportationPlanner::ETransportationMode::Bus:
        //             modeStr = "Bus";
        //             break;
        //     }

    // }
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
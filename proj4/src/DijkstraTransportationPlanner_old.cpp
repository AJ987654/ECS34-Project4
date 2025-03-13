#include "DijkstraTransportationPlanner.h"
#include "DijkstraPathRouter.h"
#include "BusSystemIndexer.h"
#include "GeographicUtils.h"
#include <unordered_map>

// Define the SImplementation struct
struct CDijkstraTransportationPlanner::SImplementation {
    
    std::unordered_map<TNodeID, TVertexID> NodeToVertex;
    std::unordered_map<TNodeID, TVertexID> BusNodeToVertex;
    std::shared_ptr<CStreetMap> DStreetMap;
    std::shared_ptr<CBusSystemIndexer> DBusSystemIndexer;
    double DWalkSpeed;
    double DBikeSpeed;
    double DDefaultSpeedLimit;
    double DBusStopTime;
    int DPrecomputeTime;

    SImplementation(std::shared_ptr<SConfiguration> config) {
        // Get the configuration parameters
        DStreetMap = config->StreetMap();
        BusSystemIndexer = std::make_shared<CBusSystemIndexer>(config->BusSystem());
        WalkSpeed = config->WalkSpeed();
        BikeSpeed = config->BikeSpeed();
        DefaultSpeedLimit = config->DefaultSpeedLimit();
        BusStopTime = config->BusStopTime();
        PrecomputeTime = config->PrecomputeTime();
    }

    void CreateNodes(std::shared_ptr<CDijkstraPathRouter> pathRouter) {
        std::size_t NumNodes = DStreetMap->NodeCount();
        for (std::size_t Index = 0; Index < NumNodes; Index++) {
            auto Node = DStreetMap->NodeByIndex(Index);
            auto NodeID = Node->ID();
            auto VertexID = pathRouter->AddVertex(NodeID);
            NodeToVertex[NodeID] = VertexID;
        }                
    }

    void CreateBusNodes(std::shared_ptr<CDijkstraPathRouter> pathRouter) {
        std::size_t NumStops = DBusSystemIndexer->StopCount();
        for (std::size_t Index = 0; Index < NumStops; Index++) {
            auto Stop = DBusSystemIndexer->SortedStopByIndex(Index);
            auto NodeID = Stop->NodeID();
            auto VertexID = pathRouter->AddVertex(NodeID);
            BusNodeToVertex[NodeID] = VertexID;
        }
    }

    void CreateEdges(std::shared_ptr<CDijkstraPathRouter> pathRouter, bool fastest, bool bike){
        std::size_t NumWays = DStreetMap->WayCount();
        for (std::size_t Index = 0; Index < NumWays; Index++) {
            auto Way = streetMap->WayByIndex(Index);
            std::size_t NumNodes = Way->NodeCount();
            
            bool oneWay = false;
            if (Way->GetAttribute("oneway") == "yes") {
                oneWay = true;
            }

            bool bikeAllowed = true;
            if (Way->GetAttribute("bicyle") == "no") {
                bikeAllowed = false;
            }

            if (bike && !bikeAllowed) {
                continue;
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
                    if (fastest) {
                        if (bike) {
                            EdgeWeight /= DBikeSpeed;
                        } else {
                            EdgeWeight /= DWalkSpeed;
                        }
                    }
                    if (oneWay) {
                        if (fastest && !bike) {
                            pathRouter->AddEdge(Node1Vertex->second, Node2Vertex->second, EdgeWeight, true);
                            continue;
                        } else {
                            pathRouter->AddEdge(Node1Vertex->second, Node2Vertex->second, EdgeWeight, false);
                            continue;
                        }
                    } else {
                        pathRouter->AddEdge(Node1Vertex->second, Node2Vertex->second, EdgeWeight, true);
                        continue;
                    }
                }
            }
        }
    }

    void CreateBusEdges(std::shared_ptr<CDijkstraPathRouter> pathRouter, bool fastest) {
        std::size_t NumRoutes = DBusSystemIndexer->RouteCount();
        for (std::size_t Index = 0; Index < NumRoutes; Index++) {
            auto Route = DBusSystemIndexer->SortedRouteByIndex(Index);
            std::size_t NumStops = Route->StopCount();
            for (std::size_t StopIndex = 0; StopIndex < NumStops - 1; StopIndex++) {
                auto Stop1 = Route->GetStopID(StopIndex);
                auto Stop2 = Route->GetStopID(StopIndex + 1);
                
    
    double FindShortestPath(TNodeID src, TNodeID dest, std::vector<TNodeID>& path) {
        auto WalkShortestPathRouter = std::make_shared<CDijkstraPathRouter>();
        auto BikeShortestPathRouter = std::make_shared<CDijkstraPathRouter>();
        auto BusShortestPathRouter = std::make_shared<CDijkstraPathRouter>();
        CreateNodes(WalkShortestPathRouter);
        CreateNodes(BikeShortestPathRouter);
        CreateBusNodes(BusShortestPathRouter);
        CreateEdges(WalkShortestPathRouter, false, false);
        CreateEdges(BikeShortestPathRouter, false, true);
};

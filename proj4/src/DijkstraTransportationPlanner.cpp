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
#include <iostream>
#include <sstream>
#include <iomanip>
#include <string>

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

    std::string DoubleToStringWithOneDecimal(double value) const {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(1) << value;
        return oss.str();
    }

    std::set<std::string> Intersection(const std::set<std::string>& set1, const std::set<std::string>& set2) const {
        std::set<std::string> result;
        for (const auto& elem : set1) {
            if (set2.find(elem) != set2.end()) {
                result.insert(elem);
            }
        }
        return result;
    }

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
            for (std::size_t NodeIndex1 = 0; NodeIndex1 < NumNodes; NodeIndex1++) {
                for (std::size_t NodeIndex2 = NodeIndex1 + 1; NodeIndex2 < NumNodes; NodeIndex2++) {
                    auto Node1 = Way->GetNodeID(NodeIndex1);
                    auto Node2 = Way->GetNodeID(NodeIndex2);
                    NodePairToWay[std::make_pair(Node1, Node2)] = Way->ID();
                    NodePairToWay[std::make_pair(Node2, Node1)] = Way->ID();
                }
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

    bool GetPathWays(const std::vector<TTripStep>& path, std::vector<CStreetMap::TWayID>& Ways) const{
        auto PathLength = path.size();
        Ways.clear();
        for (std::size_t CurrentIndex = 1; CurrentIndex < PathLength; CurrentIndex++){
            auto CurrentNodeID = path[CurrentIndex].second;
            auto PrevNodeID = path[CurrentIndex - 1].second;
            auto PairWayID = NodePairToWay.find(std::make_pair(PrevNodeID, CurrentNodeID));
            if (PairWayID == NodePairToWay.end()) {
                std::cout << "No way found between nodes " << PrevNodeID << " and " << CurrentNodeID << std::endl;
                return false;
            }
            Ways.push_back(PairWayID->second);
        }
        // for (const auto& WayID : Ways) {
        //     std::cout << "Way ID: " << WayID << std::endl;
        // }
        return true;                         
    }

    std::vector<std::string> GetPathStreetNames(std::vector<CStreetMap::TWayID>& Ways) const {
        std::vector<std::string> StreetNames;
        for (auto WayID : Ways) {
            auto Way = DStreetMap->WayByID(WayID);
            auto StreetName = Way->GetAttribute("name");
            StreetNames.push_back(StreetName);
        }
        // for (const auto& name : StreetNames) {
        //     std::cout << name << std::endl;
        // }
        return StreetNames;
    }

    std::vector<std::set<std::string>> GetPathRoutes(const std::vector<TTripStep>& path) const{
        std::vector<std::set<std::string>> PathRouteNames;
        auto PathLength = path.size();
        for (std::size_t CurrentIndex = 1; CurrentIndex < PathLength; CurrentIndex++) {
            auto CurrentNodeID = path[CurrentIndex].second;
            auto PrevNodeID = path[CurrentIndex - 1].second;
            std::unordered_set<std::shared_ptr<CBusSystem::SRoute>> RoutesBetweenNodes;
            std::set<std::string> RouteNames;
            if (DBusSystemIndexer->RoutesByNodeIDs(PrevNodeID, CurrentNodeID, RoutesBetweenNodes)) {
                for (auto Route : RoutesBetweenNodes) {
                    RouteNames.insert(Route->Name());
                }
                PathRouteNames.push_back(RouteNames);
            } else {
                PathRouteNames.push_back({});
            }
        }
        return PathRouteNames;
    }

    bool GetPathDescription(const std::vector<TTripStep>& path, std::vector<std::string>& desc) const {
        auto PathLength = path.size();
        std::vector<CStreetMap::TWayID> Ways;
        std::vector<std::set<std::string>> Routes;

        if (PathLength == 0) {
            return false;
        }
        
        CStreetMap::TNodeID StartNodeID = path[0].second;
        desc.push_back("Start at " + SGeographicUtils::ConvertLLToDMS(DStreetMap->NodeByID(StartNodeID)->Location()));
        
        if (!GetPathWays(path, Ways)) {
            return false;
        }

        auto NumWays = Ways.size();
        auto StreetNames = GetPathStreetNames(Ways);

        bool bike = false;
    
        if (path[0].first == CTransportationPlanner::ETransportationMode::Bike){
            bike = true;
        }
        
        // if (!bike){
        //     std::vector<std::set<std::string>> Routes = GetPathRoutes(path); 
        //     for (const auto& routeSet : Routes) {
        //         std::cout << "Routes: ";
        //         for (const auto& route : routeSet) {
        //             std::cout << route << " ";
        //         }
        //         std::cout << std::endl;
        //     }
        // }

        if (bike){
            std::vector<CStreetMap::TNodeID> UniqueNodes;
            std::vector<CStreetMap::TWayID> UniqueWays;
            std::vector<std::string> UniqueStreetNames;
            auto PrevWayID = CStreetMap::InvalidWayID;
            for (size_t Index = 0; Index < NumWays; Index++){
                // std::cout << "Prev Way ID: " << PrevWayID << std::endl;
                // std::cout << "Current Way ID: " << Ways[Index] << std::endl;
                auto CurrWayID = Ways[Index];
                if (CurrWayID != PrevWayID){
                    UniqueWays.push_back(CurrWayID);
                    UniqueNodes.push_back(path[Index].second);
                    UniqueStreetNames.push_back(StreetNames[Index]);
                }
                PrevWayID = CurrWayID;
            }
            UniqueNodes.push_back(path[PathLength - 1].second);

            auto NumUniqueWays = UniqueWays.size();
            // std::cout << "Num Unique Ways: " << NumUniqueWays << std::endl;
            for (size_t UniqueIndex = 0; UniqueIndex < NumUniqueWays; UniqueIndex++){
                // std::cout << "Unique Index: " << UniqueIndex << std::endl;
                auto CurrentNodeID = UniqueNodes[UniqueIndex];
                auto NextNodeID = UniqueNodes[UniqueIndex + 1];
                
                auto StreetName = UniqueStreetNames[UniqueIndex];
                std::string NextStreetName = "End";
                if (UniqueIndex != (NumUniqueWays - 1)){
                    NextStreetName = UniqueStreetNames[UniqueIndex + 1];
                }
               
                std::string street_info;
                if (StreetName != ""){
                    street_info = " along " + StreetName;
                } else {
                    street_info = " toward " + NextStreetName;
                }
                // std::cout << "Street Info: " << street_info << std::endl;
                auto bearing = SGeographicUtils::CalculateBearing(DStreetMap->NodeByID(CurrentNodeID)->Location(), DStreetMap->NodeByID(NextNodeID)->Location());
                auto distance = SGeographicUtils::HaversineDistanceInMiles(DStreetMap->NodeByID(CurrentNodeID)->Location(), DStreetMap->NodeByID(NextNodeID)->Location());
                desc.push_back("Bike " + SGeographicUtils::BearingToDirection(bearing) + street_info + " for " + DoubleToStringWithOneDecimal(distance) + " mi");
            }
            desc.push_back("End at " + SGeographicUtils::ConvertLLToDMS(DStreetMap->NodeByID(path[PathLength - 1].second)->Location()));
            return true;
        } else {

            std::vector<std::set<std::string>> Routes = GetPathRoutes(path); 
            // for (const auto& routeSet : Routes) {
            //     std::cout << "Routes: ";
            //     for (const auto& route : routeSet) {
            //         std::cout << route << " ";
            //     }
            //     std::cout << std::endl;
            // }

            std::vector<CStreetMap::TNodeID> UniqueNodes;
            std::vector<CStreetMap::TWayID> UniqueWays;
            std::vector<std::string> UniqueStreetNames;
            std::vector<CTransportationPlanner::ETransportationMode> UniqueModes;
            auto PrevWayID = CStreetMap::InvalidWayID;

            std::vector<std::string> UniqueRoutes;
            std::set<std::string> CurrCommonRoutes;
            std::set<std::string> PrevCommonRoutes = {"Walk"};

            // Define a placeholder value
            const uint64_t PlaceholderValue = std::numeric_limits<uint64_t>::max();

            for (size_t Index = 0; Index < NumWays; Index++){
                auto CurrWayID = Ways[Index];
                auto CurrRouteSet = Routes[Index]; // This is an unordered set of route names 
                if (path[Index + 1].first == CTransportationPlanner::ETransportationMode::Walk){
                    if (CurrWayID != PrevWayID){
                        UniqueWays.push_back(CurrWayID);
                        UniqueNodes.push_back(path[Index].second);
                        UniqueStreetNames.push_back(StreetNames[Index]);
                        UniqueRoutes.push_back("Walk");
                        UniqueModes.push_back(CTransportationPlanner::ETransportationMode::Walk);
                        PrevCommonRoutes = {"Walk"};
                    }
                    PrevWayID = CurrWayID;
                } else {
                    if (PrevCommonRoutes == std::set<std::string>{"Walk"}){
                        CurrCommonRoutes = CurrRouteSet;
                        UniqueNodes.push_back(path[Index].second);
                    } else {
                        CurrCommonRoutes = Intersection(PrevCommonRoutes, CurrRouteSet);
                    }
                    if (Index == (NumWays - 1)){
                        // UniqueWays.push_back(CStreetMap::InvalidWayID);
                        UniqueWays.push_back(PlaceholderValue);
                        // UniqueNodes.push_back(path[Index].second);
                        UniqueStreetNames.push_back("Bus");
                        UniqueRoutes.push_back(*CurrCommonRoutes.begin());
                        UniqueModes.push_back(CTransportationPlanner::ETransportationMode::Bus);
                    } else {
                        auto NextMode = path[Index + 2].first;
                        if (NextMode == CTransportationPlanner::ETransportationMode::Walk){
                            // UniqueWays.push_back(CStreetMap::InvalidWayID);
                            // UniqueNodes.push_back(path[Index].second);
                            UniqueWays.push_back(PlaceholderValue);
                            UniqueStreetNames.push_back("Bus");
                            UniqueRoutes.push_back(*CurrCommonRoutes.begin());
                            UniqueModes.push_back(CTransportationPlanner::ETransportationMode::Bus);
                            PrevWayID = CStreetMap::InvalidWayID;
                        } else {
                            PrevCommonRoutes = CurrCommonRoutes;
                        }
                    }    
                }
            }
            UniqueNodes.push_back(path[PathLength - 1].second);
            
            auto NumUniqueWays = UniqueWays.size();
            for (size_t UniqueIndex = 0; UniqueIndex < NumUniqueWays; UniqueIndex++){
                auto CurrentNodeID = UniqueNodes[UniqueIndex];
                auto NextNodeID = UniqueNodes[UniqueIndex + 1];
                auto CurrentMode = UniqueModes[UniqueIndex];
                
                if (CurrentMode == CTransportationPlanner::ETransportationMode::Walk){
                    auto StreetName = UniqueStreetNames[UniqueIndex];
                    std::string NextStreetName = "End";
                    if (UniqueIndex != (NumUniqueWays - 1)){
                        NextStreetName = UniqueStreetNames[UniqueIndex + 1];
                    }
                    std::string street_info;
                    if (StreetName != ""){
                        street_info = " along " + StreetName;
                    } else {
                        street_info = " toward " + NextStreetName;
                    }
                    auto bearing = SGeographicUtils::CalculateBearing(DStreetMap->NodeByID(CurrentNodeID)->Location(), DStreetMap->NodeByID(NextNodeID)->Location());
                    auto distance = SGeographicUtils::HaversineDistanceInMiles(DStreetMap->NodeByID(CurrentNodeID)->Location(), DStreetMap->NodeByID(NextNodeID)->Location());
                    desc.push_back("Walk " + SGeographicUtils::BearingToDirection(bearing) + street_info + " for " + DoubleToStringWithOneDecimal(distance) + " mi");
                } else {
                    std::string RouteName = UniqueRoutes[UniqueIndex];
                    auto Stop1 = DBusSystemIndexer->StopByNodeID(CurrentNodeID);
                    auto Stop2 = DBusSystemIndexer->StopByNodeID(NextNodeID);
                    std::string Stop1Name = std::to_string(Stop1->ID());
                    std::string Stop2Name = std::to_string(Stop2->ID());
                    desc.push_back("Take Bus " + RouteName + " from stop " + Stop1Name + " to stop " + Stop2Name);
                }
            }
            desc.push_back("End at " + SGeographicUtils::ConvertLLToDMS(DStreetMap->NodeByID(path[PathLength - 1].second)->Location()));
            return true;
        }
        return false;
    }
    //     if (bike){
    //         auto CurrentStartingNodeID = StartNodeID;
    //         for (std::size_t CurrentIndex = 1; CurrentIndex < PathLength - 1; CurrentIndex++) {
    //             auto CurrentWayID = Ways[CurrentIndex - 1];
    //             auto NextWayID = Ways[CurrentIndex];
    //             auto CurrentNodeID = path[CurrentIndex].second;
    //             if (CurrentWayID != NextWayID) {
    //                 auto StreetName = StreetNames[CurrentIndex - 1];
    //                 auto NextStreetName = StreetNames[CurrentIndex];
    //                 std::string street_info;
    //                 if (StreetName != "") {
    //                     std::string street_info = " along " + StreetName;
    //                 } else{
    //                     std::string street_info = " toward " + NextStreetName;
    //                 }
    //                 auto bearing = SGeographicUtils::CalculateBearing(DStreetMap->NodeByID(CurrentStartingNodeID)->Location(), DStreetMap->NodeByID(CurrentNodeID)->Location());
    //                 auto distance = SGeographicUtils::HaversineDistanceInMiles(DStreetMap->NodeByID(CurrentStartingNodeID)->Location(), DStreetMap->NodeByID(CurrentNodeID)->Location());
    //                 desc.push_back("Bike " + SGeographicUtils::BearingToDirection(bearing) + street_info + " for " + DoubleToStringWithOneDecimal(distance) + " mi");
    //                 CurrentStartingNodeID = CurrentNodeID;
    //             }
    //         }
    //         auto LastStreet = StreetNames[PathLength - 2];
    //         auto LastNode = path[PathLength - 1].second;
    //         auto bearing = SGeographicUtils::CalculateBearing(DStreetMap->NodeByID(CurrentStartingNodeID)->Location(), DStreetMap->NodeByID(LastNode)->Location());
    //         auto distance = SGeographicUtils::HaversineDistanceInMiles(DStreetMap->NodeByID(CurrentStartingNodeID)->Location(), DStreetMap->NodeByID(LastNode)->Location());
    //         if (LastStreet != "") {
    //             desc.push_back("Bike " + SGeographicUtils::BearingToDirection(bearing) + " along " + LastStreet + " for " + DoubleToStringWithOneDecimal(distance) + " mi");
    //         } else {
    //             desc.push_back("Bike toward End for " + DoubleToStringWithOneDecimal(distance) + " mi");
    //         }
    //         desc.push_back("End at " + SGeographicUtils::ConvertLLToDMS(DStreetMap->NodeByID(path[PathLength - 1].second)->Location()));
    //         return true;
    //     } else {
    //         auto CurrentStartingNodeID = StartNodeID;
    //         for (std::size_t CurrentIndex = 1; CurrentIndex < PathLength - 1; CurrentIndex++) {
    //             auto CurrentWayID = Ways[CurrentIndex - 1];
    //             auto NextWayID = Ways[CurrentIndex];
    //             auto CurrentMode = path[CurrentIndex].first;
    //             auto NextMode = path[CurrentIndex + 1].first;
    //             auto CurrentNodeID = path[CurrentIndex].second;
    //         }
    //         desc.push_back("End at " + SGeographicUtils::ConvertLLToDMS(DStreetMap->NodeByID(path[PathLength - 1].second)->Location()));
    //         return true;
    //     }
    //     return false;
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
    return DImplementation->GetPathDescription(path, desc);
}
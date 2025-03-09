#include "CSVBusSystem.h"
#include "StreetMap.h"
#include "DSVReader.h"
#include "StringUtils.h"
#include <memory>
#include <vector>
#include <unordered_map>
#include <string>
#include <iostream>

// Internal implementation struct
struct CCSVBusSystem::SImplementation {

    struct CConcreteStop : public CBusSystem::SStop{
        TStopID DStopID;
        CStreetMap::TNodeID DNodeID;

        TStopID ID() const noexcept override {
            return DStopID;
        }

        CStreetMap::TNodeID NodeID() const noexcept override {
            return DNodeID;
        }
    };

    struct CConcreteRoute : public CBusSystem::SRoute{
        std::string DRouteName;
        std::vector<TStopID> DStopIDs;

        std::string Name() const noexcept override {
            return DRouteName;
        }

        std::size_t StopCount() const noexcept override {
            return DStopIDs.size();
        }

        TStopID GetStopID(std::size_t index) const noexcept override {
            if (index >= DStopIDs.size()) {
                return CBusSystem::InvalidStopID;
            }
            return DStopIDs[index];
        }
    };

    // Utility function to check if a string is a valid integer
    bool IsNumber(const std::string& str) {
        if (str.empty()) {
            return false;
        }
        for (char c : str) {
            if (!std::isdigit(c)) {
                return false;
            }
        }
        return true;
    }

    std::vector<std::shared_ptr<CConcreteStop>> Stops;
    std::vector<std::shared_ptr<CConcreteRoute>> Routes;
    int StopCount = 0;
    int RouteCount = 0;

    SImplementation(std::shared_ptr< CDSVReader > stopsrc, std::shared_ptr< CDSVReader > routesrc){
        std::vector<std::string> row;
        // std::cout << "SImplementation Constructor" << std::endl;
        while (!stopsrc->End()){
            if (stopsrc->ReadRow(row)){
                auto stop_id = StringUtils::Strip(row[0]);
                auto node_id = StringUtils::Strip(row[1]);
                if (!IsNumber(stop_id) || !IsNumber(node_id)){
                    continue;
                } else {
                    auto stop = std::make_shared<CConcreteStop>();
                    stop->DStopID = std::stoull(stop_id);
                    stop->DNodeID = std::stoull(node_id);
                    Stops.push_back(stop);
                    StopCount++;
                    // std::cout << "Stop ID: " << stop->DStopID << " Node ID: " << stop->DNodeID << std::endl;
                }
            }
        }

        while (!routesrc->End()){
            if (routesrc->ReadRow(row)){
                auto route_name = StringUtils::Strip(row[0]);
                auto stop_id = StringUtils::Strip(row[1]);
                bool found = false;
                if (!IsNumber(stop_id)){
                    continue;
                } else {
                    for (auto &Route : Routes){
                        if (Route->DRouteName == route_name){
                            Route->DStopIDs.push_back(std::stoull(stop_id));
                            found = true;
                            // std::cout << "Route Name: " << Route->DRouteName << " Stop ID: " << Route->DStopIDs[0] << std::endl;
                            break;
                        }
                    }
                    if (!found){
                        auto route = std::make_shared<CConcreteRoute>();
                        route->DRouteName = route_name;
                        route->DStopIDs.push_back(std::stoull(row[1]));
                        Routes.push_back(route);
                        RouteCount++;
                        // std::cout << "Route Name: " << route->DRouteName << " Stop ID: " << route->DStopIDs[0] << std::endl;
                    }
                }
            }
        }
    }
};


// Constructor
CCSVBusSystem::CCSVBusSystem(std::shared_ptr<CDSVReader> stopsrc, std::shared_ptr<CDSVReader> routesrc){
    // std::cout << "CCSVBusSystem Constructor" << std::endl;
    DImplementation = std::make_unique<SImplementation>(stopsrc, routesrc);
}

// Destructor
CCSVBusSystem::~CCSVBusSystem() = default;

// Returns the number of stops in the system
std::size_t CCSVBusSystem::StopCount() const noexcept {
    return DImplementation->StopCount;
}

// Returns the number of routes in the system
std::size_t CCSVBusSystem::RouteCount() const noexcept {
    return DImplementation->RouteCount;
}

// Returns the SStop specified by the index, nullptr if index >= StopCount()
std::shared_ptr<CBusSystem::SStop> CCSVBusSystem::StopByIndex(std::size_t index) const noexcept {
    if (index >= DImplementation->Stops.size()) {
        return nullptr;
    }
    return DImplementation->Stops[index];
}

// Returns the SStop specified by the stop id, nullptr if id is not in the stops
std::shared_ptr<CBusSystem::SStop> CCSVBusSystem::StopByID(TStopID id) const noexcept {
    for (auto &Stop : DImplementation->Stops){
        if (Stop->ID() == id){
            return Stop;
        }
    }
    return nullptr;
}

// Returns the SRoute specified by the index, nullptr if index >= RouteCount()
std::shared_ptr<CBusSystem::SRoute> CCSVBusSystem::RouteByIndex(std::size_t index) const noexcept {
    if (index >= DImplementation->Routes.size()) {
        return nullptr;
    }
    return DImplementation->Routes[index];
}

// Returns the SRoute specified by the name, nullptr if name is not in the routes
std::shared_ptr<CBusSystem::SRoute> CCSVBusSystem::RouteByName(const std::string& name) const noexcept {
    for (auto &Route : DImplementation->Routes){
        if (Route->Name() == name){
            return Route;
        }
    }
    return nullptr;
}
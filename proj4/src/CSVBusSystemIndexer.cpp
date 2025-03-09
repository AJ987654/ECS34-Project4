#include "BusSystemIndexer.h"
#include "CSVBusSystem.h"
#include "DSVReader.h"
#include "StringDataSource.h"
#include <memory>
#include <unordered_map>
#include <vector>
#include <unordered_set>
#include <string>
#include <iostream>
#include <algorithm> // For std::sort

struct CBusSystemIndexer::SImplementation {
    std::vector<std::shared_ptr<SStop>> Stops;
    std::vector<std::shared_ptr<SRoute>> Routes;
    std::unordered_map<TNodeID, std::shared_ptr<SStop>> NodeIDToStop;

    struct CConcreteStop : public SStop {
        CBusSystem::TStopID DStopID;
        CStreetMap::TNodeID DNodeID;

        CBusSystem::TStopID ID() const noexcept override {
            return DStopID;
        }

        CStreetMap::TNodeID NodeID() const noexcept override {
            return DNodeID;
        }
    };

    struct CConcreteRoute : public SRoute {
        std::string DRouteName;
        std::vector<CBusSystem::TStopID> DStopIDs;

        std::string Name() const noexcept override {
            return DRouteName;
        }

        std::size_t StopCount() const noexcept override {
            return DStopIDs.size();
        }

        CBusSystem::TStopID GetStopID(std::size_t index) const noexcept override {
            if (index >= DStopIDs.size()) {
                return CBusSystem::InvalidStopID;
            }
            return DStopIDs[index];
        }
    };

    SImplementation(std::shared_ptr<CBusSystem> bussystem) {
        // Load stops
        for (std::size_t StopIndex = 0; StopIndex < bussystem->StopCount(); StopIndex++) {
            auto Stop = bussystem->StopByIndex(StopIndex);
            if (Stop) {
                auto ConcreteStop = std::make_shared<CConcreteStop>();
                ConcreteStop->DStopID = Stop->ID();
                ConcreteStop->DNodeID = Stop->NodeID();
                NodeIDToStop[ConcreteStop->DNodeID] = ConcreteStop;
                Stops.push_back(ConcreteStop);
            }
        }

        // Sort stops by ID
        std::sort(Stops.begin(), Stops.end(), [](const std::shared_ptr<SStop>& a, const std::shared_ptr<SStop>& b) {
            return a->ID() < b->ID();
        });

        // Load routes
        for (std::size_t RouteIndex = 0; RouteIndex < bussystem->RouteCount(); RouteIndex++) {
            auto Route = bussystem->RouteByIndex(RouteIndex);
            if (Route) {
                auto ConcreteRoute = std::make_shared<CConcreteRoute>();
                ConcreteRoute->DRouteName = Route->Name();
                for (std::size_t i = 0; i < Route->StopCount(); i++) {
                    ConcreteRoute->DStopIDs.push_back(Route->GetStopID(i));
                }
                Routes.push_back(ConcreteRoute);
            }
        }

        // Sort routes by name
        std::sort(Routes.begin(), Routes.end(), [](const std::shared_ptr<SRoute>& a, const std::shared_ptr<SRoute>& b) {
            return a->Name() < b->Name();
        });
    }

    std::shared_ptr<SStop> StopByNodeID(TNodeID id) const noexcept {
        auto StopIterator = NodeIDToStop.find(id);
        if (StopIterator != NodeIDToStop.end()) {
            return StopIterator->second;
        }
        return nullptr;
    }
};

// CBusSystemIndexer member functions

CBusSystemIndexer::CBusSystemIndexer(std::shared_ptr<CBusSystem> bussystem) {
    DImplementation = std::make_unique<SImplementation>(bussystem);
}

CBusSystemIndexer::~CBusSystemIndexer() = default;

std::size_t CBusSystemIndexer::StopCount() const noexcept {
    return DImplementation->Stops.size();
}

std::size_t CBusSystemIndexer::RouteCount() const noexcept {
    return DImplementation->Routes.size();
}

std::shared_ptr<CBusSystemIndexer::SStop> CBusSystemIndexer::SortedStopByIndex(std::size_t index) const noexcept {
    if (index >= StopCount()) {
        return nullptr;
    }
    return DImplementation->Stops[index];
}

std::shared_ptr<CBusSystemIndexer::SRoute> CBusSystemIndexer::SortedRouteByIndex(std::size_t index) const noexcept {
    if (index >= DImplementation->Routes.size()) {
        return nullptr;
    }
    return DImplementation->Routes[index];
}

std::shared_ptr<CBusSystemIndexer::SStop> CBusSystemIndexer::StopByNodeID(TNodeID id) const noexcept {
    return DImplementation->StopByNodeID(id);
}

bool CBusSystemIndexer::RoutesByNodeIDs(TNodeID src, TNodeID dest, std::unordered_set<std::shared_ptr<SRoute>>& routes) const noexcept {
    auto SourceStop = StopByNodeID(src);
    auto DestinationStop = StopByNodeID(dest);
    if (SourceStop && DestinationStop) {
        for (auto& Route : DImplementation->Routes) {
            bool FoundSource = false;
            bool FoundDestination = false;
            for (std::size_t StopIndex = 0; StopIndex < Route->StopCount(); StopIndex++) {
                if (Route->GetStopID(StopIndex) == SourceStop->ID()) {
                    FoundSource = true;
                }
                if (Route->GetStopID(StopIndex) == DestinationStop->ID()) {
                    FoundDestination = true;
                }
            }
            if (FoundSource && FoundDestination) {
                routes.insert(Route);
            }
        }
        return !routes.empty();
    }
    return false;
}

bool CBusSystemIndexer::RouteBetweenNodeIDs(TNodeID src, TNodeID dest) const noexcept {
    std::unordered_set<std::shared_ptr<SRoute>> Routes;
    return RoutesByNodeIDs(src, dest, Routes);
}
#include "TransportationPlannerCommandLine.h"
#include "TransportationPlanner.h"
#include "BusSystem.h"
#include "DataSource.h"
#include "DataSink.h"
#include "DataFactory.h"
#include <memory>
#include <string>
#include <iostream>
#include <sstream>
#include <vector>

// CTransportationPlannerCommandLine member functions
// Constructor for the Transportation Planner Command Line
CTransportationPlannerCommandLine(std::shared_ptr<CDataSource> cmdsrc, std::shared_ptr<CDataSink> outsink, std::shared_ptr<CDataSink> errsink, std::shared_ptr<CDataFactory> results, std::shared_ptr<CTransportationPlanner> planner) {
    DImplementation = std::make_unique<CBusSystem::SImplementation>(cmdsrc, outsink, errsink, results, planner);
};
// Destructor for the Transportation Planner Command Line
~CTransportationPlannerCommandLine() = default;
// Processes the commands input to the
bool ProcessCommands();
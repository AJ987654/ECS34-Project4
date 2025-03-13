#include "TransportationPlannerCommandLine.h"
#include "TransportationPlanner.h"
#include "DataSource.h"
#include "DataSink.h"
#include "DataFactory.h"
#include <iostream>
#include <string>

// CTransportationPlannerCommandLine member functions
// Constructor for the Transportation Planner Command Line
CTransportationPlannerCommandLine(std::shared_ptr<CDataSource> cmdsrc, std::shared_ptr<CDataSink> outsink, std::shared_ptr<CDataSink> errsink, std::shared_ptr<CDataFactory> results, std::shared_ptr<CTransportationPlanner> planner) {
    m_cmdsrc = cmdsrc;
    m_outsink = outsink;
    m_errsink = errsink;
    m_results = results;
    m_planner = planner;
}
// Destructor for the Transportation Planner Command Line
~CTransportationPlannerCommandLine() = default;
// Runs the command line interface
// Processes the commands input to the
bool ProcessCommands() {
    std::string command;
    while (m_cmdsrc->Read(command)) {
        if (command == "exit") {
            return true;
        } else if (command == "plan") {
            std::string start, end;
            m_cmdsrc->Read(start);
            m_cmdsrc->Read(end);
            m_planner->Plan(start, end);
            m_results->Write(m_planner->GetResults());
        } else {
            m_errsink->Write("Invalid command: " + command);
        }
    }
    return false;
}
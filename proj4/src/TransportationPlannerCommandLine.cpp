#include "TransportationPlannerCommandLine.h"
#include "TransportationPlanner.h"
#include "DataSource.h"
#include "DataSink.h"
#include "DataFactory.h"
#include <iostream>
#include <string>

class CTransportationPlannerCommandLine {
public:
    // Constructor for the Transportation Planner Command Line
    CTransportationPlannerCommandLine(std::shared_ptr<CDataSource> cmdsrc, std::shared_ptr<CDataSink> outsink, std::shared_ptr<CDataSink> errsink, std::shared_ptr<CDataFactory> results, std::shared_ptr<CTransportationPlanner> planner) {
        this->m_cmdsrc = cmdsrc;
        this->m_outsink = outsink;
        this->m_errsink = errsink;
        this->m_results = results;
        this->m_planner = planner;
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

private:
    std::shared_ptr<CDataSource> m_cmdsrc;
    std::shared_ptr<CDataSink> m_outsink;
    std::shared_ptr<CDataSink> m_errsink;
    std::shared_ptr<CDataFactory> m_results;
    std::shared_ptr<CTransportationPlanner> m_planner;
};
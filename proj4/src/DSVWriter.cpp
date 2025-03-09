#include "DSVWriter.h"
#include "StringUtils.h"
#include <memory>
#include <vector>
#include <string>
#include <sstream>

// Define the SImplementation struct
struct CDSVWriter::SImplementation {
    std::shared_ptr<CDataSink> DataSink;
    char Delimiter;
    bool QuoteAll;

    SImplementation(std::shared_ptr<CDataSink> sink, char delimiter, bool quoteall)
        : DataSink(sink), Delimiter(delimiter), QuoteAll(quoteall) {}

    // Helper function to escape quotes in a string
    std::string EscapeQuotes(const std::string& str) const {
        std::string result;
        for (char ch : str) {
            if (ch == '"') {
                result += "\"\"";
            } else {
                result += ch;
            }
        }
        return result;
    }

    // Helper function to determine if a value needs to be quoted
    bool NeedsQuoting(const std::string& value) const {
        if (QuoteAll) {
            return true;
        }
        return value.find(Delimiter) != std::string::npos ||
               value.find('"') != std::string::npos ||
               value.find('\n') != std::string::npos;
    }
};

// Constructor
CDSVWriter::CDSVWriter(std::shared_ptr<CDataSink> sink, char delimiter, bool quoteall) {
    DImplementation = std::make_unique<SImplementation>(sink, delimiter, quoteall);
}

// Destructor
CDSVWriter::~CDSVWriter() = default;

// WriteRow function
bool CDSVWriter::WriteRow(const std::vector<std::string>& row) {
    std::ostringstream output;
    for (size_t i = 0; i < row.size(); i++) {
        std::string value = row[i];
        if (DImplementation->NeedsQuoting(value)) {
            value = DImplementation->EscapeQuotes(value);
            output << "\"" << value << "\"";
        } else {
            output << value;
        }
        if (i < row.size() - 1) {
            output << DImplementation->Delimiter;
        }
    }
    output << "\n"; // Add newline at the end of the row

    // Convert the output string to a vector<char>
    std::string outputStr = output.str();
    std::vector<char> outputVec(outputStr.begin(), outputStr.end());

    // Write the output to the data sink
    return DImplementation->DataSink->Write(outputVec);
}
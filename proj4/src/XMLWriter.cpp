#include "XMLWriter.h"
#include "DataSink.h"
#include <stack>
#include <vector>

// Private implementation struct
struct CXMLWriter::SImplementation {
    std::shared_ptr<CDataSink> DSink; // Data sink for writing XML
    std::stack<std::string> DEndElements; // Stack to track open elements

    // Constructor
    SImplementation(std::shared_ptr<CDataSink> sink)
        : DSink(sink) {}

    // Write a string to the data sink
    bool WriteString(const std::string& str) {
        std::vector<char> data(str.begin(), str.end());
        return DSink->Write(data);
    }

    // Write a character to the data sink
    bool WriteChar(char ch) {
        return DSink->Put(ch);
    }
};

// CXMLWriter constructor
CXMLWriter::CXMLWriter(std::shared_ptr<CDataSink> sink)
    : DImplementation(std::make_unique<SImplementation>(sink)) {}

// CXMLWriter destructor
CXMLWriter::~CXMLWriter() {
    Flush(); // Ensure all open elements are closed
}

// Flush the writer (close all open elements)
bool CXMLWriter::Flush() {
    while (!DImplementation->DEndElements.empty()) {
        const std::string& Element = DImplementation->DEndElements.top();
        DImplementation->WriteString("</");
        DImplementation->WriteString(Element);
        DImplementation->WriteChar('>');
        DImplementation->DEndElements.pop();
    }
    return true;
}

std::string HandleEscapeSequences(std::string str) {
    std::string result;
    for (size_t i = 0; i < str.size(); i++) {
        if (str[i] == '&') {
            result += "&amp;";
        } else if (str[i] == '<') {
            result += "&lt;";
        } else if (str[i] == '>') {
            result += "&gt;";
        } else if (str[i] == '"') {
            result += "&quot;";
        } else if (str[i] == '\'') {
            result += "&apos;";
        } else {
            result += str[i];
        }
    }
    return result;
}

// Write an XML entity to the data sink
bool CXMLWriter::WriteEntity(const SXMLEntity& entity) {
    switch (entity.DType) {
        case SXMLEntity::EType::StartElement:
            DImplementation->WriteChar('<');
            DImplementation->WriteString(entity.DNameData);
            for (const auto& Attr : entity.DAttributes) {
                DImplementation->WriteChar(' ');
                DImplementation->WriteString(Attr.first);
                DImplementation->WriteChar('=');
                DImplementation->WriteChar('"');
                DImplementation->WriteString(HandleEscapeSequences(Attr.second));
                DImplementation->WriteChar('"');
            }
            DImplementation->WriteChar('>');
            DImplementation->DEndElements.push(entity.DNameData);
            break;

        case SXMLEntity::EType::EndElement:
            DImplementation->WriteString("</");
            DImplementation->WriteString(entity.DNameData);
            DImplementation->WriteChar('>');
            break;

        case SXMLEntity::EType::CompleteElement:
            DImplementation->WriteChar('<');
            DImplementation->WriteString(entity.DNameData);
            for (const auto& Attr : entity.DAttributes) {
                DImplementation->WriteChar(' ');
                DImplementation->WriteString(Attr.first);
                DImplementation->WriteChar('=');
                DImplementation->WriteChar('"');
                DImplementation->WriteString(Attr.second);
                DImplementation->WriteChar('"');
            }
            DImplementation->WriteString("/>");
            break;

        case SXMLEntity::EType::CharData:
            DImplementation->WriteString(HandleEscapeSequences(entity.DNameData));
            break;
    }
    return true;
}

#include "OpenStreetMap.h"
#include "XMLReader.h"
#include <vector> // Add this include for std::vector
#include <string> // Add this include for std::string
#include <memory> // Add this include for std::make_shared

// Internal implementation struct
struct COpenStreetMap::SImplementation {
    struct SNodeImpl : public CStreetMap::SNode {
        TNodeID OSM_NodeID;
        TLocation OSM_Node_Location;
        std::vector<std::pair<std::string, std::string>> OSM_Node_Attributes; // Define Attributes type
        
        TNodeID ID() const noexcept override {
            return OSM_NodeID;
        }

        TLocation Location() const noexcept override {
            return OSM_Node_Location;
        }

        std::size_t AttributeCount() const noexcept override {
            return OSM_Node_Attributes.size();
        }

        std::string GetAttributeKey(std::size_t index) const noexcept override {
            // If index is valid, return the key
            if (index < AttributeCount()) {
                return OSM_Node_Attributes[index].first;
            }
            // Otherwise, return an empty string
            return "";
        }

        bool HasAttribute(const std::string &key) const noexcept override {
            // Check if the key exists by iterating through the attributes
            for (const auto &Attribute : OSM_Node_Attributes) {
                if (Attribute.first == key) {
                    return true;
                }
            }
            return false;
        }

        std::string GetAttribute(const std::string &key) const noexcept override {
            // Iterate over the attributes to find the key
            for (const auto &Attribute : OSM_Node_Attributes) {
                if (Attribute.first == key) {
                    return Attribute.second;
                }
            }
            // Return an empty string if the key is not found
            return "";
        }
    };

    struct SWayImpl : public CStreetMap::SWay {
        TWayID OSM_WayID;
        std::vector<TNodeID> OSM_NodeIDs;
        std::vector<std::pair<std::string, std::string>> OSM_Way_Attributes; // Define Attributes type

        TWayID ID() const noexcept override {
            return OSM_WayID;
        }

        std::size_t NodeCount() const noexcept override {
            return OSM_NodeIDs.size();
        }

        TNodeID GetNodeID(std::size_t index) const noexcept override {
            if (index >= NodeCount()) {
                return CStreetMap::InvalidNodeID;
            }
            return OSM_NodeIDs[index];
        }

        std::size_t AttributeCount() const noexcept override {
            return OSM_Way_Attributes.size();
        }

        std::string GetAttributeKey(std::size_t index) const noexcept override {
            // If index is valid, return the key
            if (index < AttributeCount()) {
                return OSM_Way_Attributes[index].first;
            }
            // Otherwise, return an empty string
            return "";
        }

        bool HasAttribute(const std::string &key) const noexcept override {
            // Check if the key exists by iterating through the attributes
            for (const auto &Attribute : OSM_Way_Attributes) {
                if (Attribute.first == key) {
                    return true;
                }
            }
            return false;
        }

        std::string GetAttribute(const std::string &key) const noexcept override {
            // Iterate over the attributes to find the key
            for (const auto &Attribute : OSM_Way_Attributes) {
                if (Attribute.first == key) {
                    return Attribute.second;
                }
            }
            // Return an empty string if the key is not found
            return "";
        }
    };

    std::shared_ptr<CXMLReader> OSM_XMLReader;
    std::vector<std::shared_ptr<SNodeImpl>> OSM_Nodes;
    std::vector<std::shared_ptr<SWayImpl>> OSM_Ways;

    SImplementation(std::shared_ptr<CXMLReader> src) {
        OSM_XMLReader = src;
        SXMLEntity Entity;
        while (OSM_XMLReader->ReadEntity(Entity, true)) {
            if (Entity.DType == SXMLEntity::EType::StartElement) {
                if (Entity.DNameData == "node"){
                    auto Node = std::make_shared<SNodeImpl>();

                    // First extract the ID and location of the node
                    // std::stoull converts a string to an unsigned long long
                    Node->OSM_NodeID = std::stoull(Entity.AttributeValue("id"));
                    // std::stod converts a string to a double
                    Node->OSM_Node_Location.first = std::stod(Entity.AttributeValue("lat"));
                    Node->OSM_Node_Location.second = std::stod(Entity.AttributeValue("lon"));
                    
                    if (Entity.DAttributes.size() > 3){
                        // Add all the attributes of the node
                        for (size_t Index = 3; Index < Entity.DAttributes.size(); Index++) {
                            Node->OSM_Node_Attributes.push_back(std::make_pair(Entity.DAttributes[Index].first, Entity.DAttributes[Index].second));
                        }
                    }
                    // Keep reading till you hit the node end tag 
                    while (OSM_XMLReader->ReadEntity(Entity, true)) {
                        // If the entity is an end element and the name is "node", break
                        if (Entity.DType == SXMLEntity::EType::EndElement && Entity.DNameData == "node") {
                            OSM_Nodes.push_back(Node);  // Add the node to the list of nodes
                            break;
                        }
                        // If the entity is a start element and the name is "tag", add the attributes
                        if (Entity.DType == SXMLEntity::EType::StartElement && Entity.DNameData == "tag") {
                            // Add the tag attributes to the node
                            Node->OSM_Node_Attributes.push_back(std::make_pair(Entity.DAttributes[0].second, Entity.DAttributes[1].second));                                
                        }
                    }
                } else if (Entity.DNameData == "way") {
                    auto Way = std::make_shared<SWayImpl>();
                    Way->OSM_WayID = std::stoull(Entity.AttributeValue("id"));
                    while (OSM_XMLReader->ReadEntity(Entity, true)) {
                        if (Entity.DType == SXMLEntity::EType::EndElement && Entity.DNameData == "way") {
                            OSM_Ways.push_back(Way);
                            break;
                        }
                        if (Entity.DType == SXMLEntity::EType::StartElement && Entity.DNameData == "nd") {
                            // Add the node ID to the way
                            Way->OSM_NodeIDs.push_back(std::stoull(Entity.AttributeValue("ref")));
                        } else if (Entity.DType == SXMLEntity::EType::StartElement && Entity.DNameData == "tag") {
                            // Add the tag attributes to the way
                            Way->OSM_Way_Attributes.push_back(std::make_pair(Entity.DAttributes[0].second, Entity.DAttributes[1].second));
                        }
                    }
                }
            }
        }
    }
};

COpenStreetMap::COpenStreetMap(std::shared_ptr<CXMLReader> src){
    DImplementation = std::make_unique<SImplementation>(src);
} 

COpenStreetMap::~COpenStreetMap() = default;

std::size_t COpenStreetMap::NodeCount() const noexcept {
    return DImplementation->OSM_Nodes.size();
}

std::size_t COpenStreetMap::WayCount() const noexcept {
    return DImplementation->OSM_Ways.size();
}

std::shared_ptr<CStreetMap::SNode> COpenStreetMap::NodeByIndex(std::size_t index) const noexcept {
    if (index < NodeCount()) {
        return DImplementation->OSM_Nodes[index];
    }
    return nullptr;
}

std::shared_ptr<CStreetMap::SNode> COpenStreetMap::NodeByID(TNodeID id) const noexcept {
    for (const auto &Node : DImplementation->OSM_Nodes) {
        if (Node->ID() == id) {
            return Node;
        }
    }
    return nullptr;
}

std::shared_ptr<CStreetMap::SWay> COpenStreetMap::WayByIndex(std::size_t index) const noexcept {
    if (index < WayCount()) {
        return DImplementation->OSM_Ways[index];
    }
    return nullptr;
}

std::shared_ptr<CStreetMap::SWay> COpenStreetMap::WayByID(TWayID id) const noexcept {
    for (const auto &Way : DImplementation->OSM_Ways) {
        if (Way->ID() == id) {
            return Way;
        }
    }
    return nullptr;
}
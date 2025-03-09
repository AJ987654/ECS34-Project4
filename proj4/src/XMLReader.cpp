#include "XMLReader.h"
#include "StringUtils.h"
#include "XMLEntity.h"
#include <memory>
#include <vector>
#include <string>
#include <expat.h>
#include <iostream>
#include <queue>
#include <cstring>
<<<<<<< HEAD
#include <unordered_map>
=======
#include <XMLWriter.h>
#include <StringDataSink.h>
#include "StringDataSource.h"
//#include "StringDataSource.cpp"
>>>>>>> 63dd653a08ee146db3f56e27567c0af505614459

struct CXMLReader::SImplementation {
    std::shared_ptr<CDataSource> DSource;
    XML_Parser DParser;
    std::queue<SXMLEntity> DEntityQueue;

<<<<<<< HEAD
    std::string ReaderHandleEscapeSequences(std::string str) {
        std::string result = str;
        // std::cout << "Original: " << result << std::endl;
        std::unordered_map<std::string, char> escapeMap = {
            {"&amp;", '&'},
            {"&lt;", '<'},
            {"&gt;", '>'},
            {"&quot;", '"'},
            {"&apos;", '\''}
        };
        for (auto it = escapeMap.begin(); it != escapeMap.end(); ++it) {
            // Account for the fact that the second value in the map is a char and needs to be converted to a string
            result = StringUtils::Replace(result, it->first, std::string(1, it->second));
            // std::cout << "Result: " << result << std::endl; 
        }
        return result;
    }

=======
>>>>>>> 63dd653a08ee146db3f56e27567c0af505614459
    SImplementation(std::shared_ptr<CDataSource> src) : DSource(src) {
        DParser = XML_ParserCreate(nullptr);
        XML_SetUserData(DParser, this);
        XML_SetElementHandler(DParser, StartElementHandler, EndElementHandler);
        XML_SetCharacterDataHandler(DParser, CharacterDataHandler);
        DEntityQueue = std::queue<SXMLEntity>();
    }

    ~SImplementation() {
        XML_ParserFree(DParser);
    }

    static void StartElementHandler(void *userData, const XML_Char *name, const XML_Char **atts) {
        // std::cout << "Element started" << std::endl;
        SXMLEntity newEntity;
        auto impl = static_cast<SImplementation*>(userData);
        newEntity.DType = SXMLEntity::EType::StartElement;
        newEntity.DNameData = name;
        newEntity.DAttributes = std::vector<std::pair<std::string, std::string>>();
        for (int i = 0; atts[i]; i += 2) {
            char* namePtr = (char*) atts[i];
            char* valuePtr = (char*) atts[i + 1];
<<<<<<< HEAD
            std::string name = impl->ReaderHandleEscapeSequences(std::string(namePtr, strlen(namePtr)));
            std::string value = impl->ReaderHandleEscapeSequences(std::string(valuePtr, strlen(valuePtr)));
=======
            std::string name = std::string(namePtr, strlen(namePtr));
            std::string value = std::string(valuePtr, strlen(valuePtr));
>>>>>>> 63dd653a08ee146db3f56e27567c0af505614459
            newEntity.DAttributes.push_back(std::make_pair(name, value));
        }
        impl->DEntityQueue.push(newEntity);
    }

<<<<<<< HEAD
    static void EndElementHandler(void *userData, const XML_Char *name) {
        // std::cout << "Element ended" << std::endl;
        SXMLEntity newEntity;
        auto impl = static_cast<SImplementation*>(userData);
        newEntity.DType = SXMLEntity::EType::EndElement;
        newEntity.DNameData = name;
        impl->DEntityQueue.push(newEntity);
    }

=======
>>>>>>> 63dd653a08ee146db3f56e27567c0af505614459
    static void CharacterDataHandler(void *userData, const XML_Char *s, int len) {
        std::string content = std::string(s, len);
        SXMLEntity newEntity;
        auto impl = static_cast<SImplementation*>(userData);
        newEntity.DType = SXMLEntity::EType::CharData;
        newEntity.DNameData = content;
        impl->DEntityQueue.push(newEntity);
    }

<<<<<<< HEAD
=======
    static void EndElementHandler(void *userData, const XML_Char *name) {
        // std::cout << "Element ended" << std::endl;
        SXMLEntity newEntity;
        auto impl = static_cast<SImplementation*>(userData);
        newEntity.DType = SXMLEntity::EType::EndElement;
        newEntity.DNameData = name;
        impl->DEntityQueue.push(newEntity);
    }

>>>>>>> 63dd653a08ee146db3f56e27567c0af505614459
    bool GetLatestEntity(SXMLEntity& entity, bool skipcdata) {  
        if (DEntityQueue.size() > 0) {
            if (skipcdata) {
                while (DEntityQueue.front().DType == SXMLEntity::EType::CharData) {
                    DEntityQueue.pop();
                }
            }
<<<<<<< HEAD
            SXMLEntity newEntity;
            bool charData = false;
            while (DEntityQueue.front().DType == SXMLEntity::EType::CharData) {
                newEntity.DType = SXMLEntity::EType::CharData;
                auto content_front = DEntityQueue.front().DNameData;
                DEntityQueue.pop();
                newEntity.DNameData = newEntity.DNameData + content_front;
                charData = true;
            } 
            entity = newEntity;
            if (!charData) {
                entity = DEntityQueue.front();
                DEntityQueue.pop();
            }
=======
            entity = DEntityQueue.front();
            DEntityQueue.pop();
>>>>>>> 63dd653a08ee146db3f56e27567c0af505614459
            return true;
        }
        return false;
    }

    bool ReadEntity(SXMLEntity& entity, bool skipcdata) {
        size_t bufferSize = 1024;
        std::vector<char> Buffer(bufferSize);
        while (!DSource->End()) {
            if (DSource->Read(Buffer, bufferSize)) {
                std::string content = std::string(Buffer.data(), Buffer.size());
                // std::cout << "Buffer: " << content << std::endl;
                if (!XML_Parse(DParser, Buffer.data(), Buffer.size(), XML_FALSE)) {
                    break;
                }  
            }
        }
        XML_Parse(DParser, nullptr, 0, XML_TRUE);
        return GetLatestEntity(entity, skipcdata);
    }
};

// Constructor
CXMLReader::CXMLReader(std::shared_ptr<CDataSource> src) {
    DImplementation = std::make_unique<SImplementation>(src);
}

// Destructor
CXMLReader::~CXMLReader() = default;

// End() function
bool CXMLReader::End() const {
    return DImplementation->DSource->End();
}

// ReadEntity() function
bool CXMLReader::ReadEntity(SXMLEntity& entity, bool skipcdata) {
    return DImplementation->ReadEntity(entity, skipcdata);
}

<<<<<<< HEAD
=======
// int main() {
//     auto InStream = std::make_shared<CStringDataSource>( "<elem attr=\"&amp;&quot;&apos;&lt;&gt;\">&amp;&quot;&apos;&lt;&gt;</elem>");
//     CXMLReader Reader(InStream);
//     SXMLEntity Entity;

//     //Reader.ReadEntity(Entity);
    
//     std::cout << (Reader.ReadEntity(Entity)) << std::endl;
//     std::cout << (Entity.DType == SXMLEntity::EType::StartElement) << std::endl;
//     std::cout << (Entity.DNameData == "elem") << std::endl;
//     std::cout << (Entity.DNameData) << std::endl;
//     std::cout << (Entity.DAttributes.size() == 1) << std::endl;
//     std::cout << (Entity.DAttributes.size()) << std::endl;
//     std::cout << (Entity.AttributeExists("attr")) << std::endl;
//     std::cout << (Entity.AttributeValue("attr") == "&\"'<>") << std::endl;
//     std::cout << (Entity.AttributeValue("attr")) << std::endl;


//     // EXPECT_EQ(Entity.DType, SXMLEntity::EType::StartElement);
//     // EXPECT_EQ(Entity.DNameData, "elem");
//     // EXPECT_EQ(Entity.DAttributes.size(), 1);
//     // EXPECT_TRUE(Entity.AttributeExists("attr"));
//     // EXPECT_EQ(Entity.AttributeValue("attr"), "&\"'<>");
    
//     // EXPECT_TRUE(Reader.ReadEntity(Entity));
//     // EXPECT_EQ(Entity.DType, SXMLEntity::EType::CharData);
//     // EXPECT_EQ(Entity.DNameData, "&\"'<>");
//     // EXPECT_EQ(Entity.DAttributes.size(), 0);
    
//     // EXPECT_TRUE(Reader.ReadEntity(Entity));
//     // EXPECT_EQ(Entity.DType, SXMLEntity::EType::EndElement);
//     // EXPECT_EQ(Entity.DNameData, "elem");
//     // EXPECT_EQ(Entity.DAttributes.size(), 0);
    
//     // EXPECT_TRUE(Reader.End());
// }
>>>>>>> 63dd653a08ee146db3f56e27567c0af505614459

#include "OpenStreetMap.h"
#include "StringDataSource.h"
#include "XMLReader.h"
#include <gtest/gtest.h>
#include <memory>
#include <sstream>

// Test fixture for COpenStreetMap
class COpenStreetMapTest : public ::testing::Test {
    protected:
        std::shared_ptr<CXMLReader> XMLReader;
        std::shared_ptr<COpenStreetMap> OpenStreetMap;

        void SetUp() override {
            std::string xmlData = R"(
                <osm>
                    <node id="1" lat="10.0" lon="20.0">
                        <tag k="name" v="Node1"/>
                    </node>
                    <node id="2" lat="30.0" lon="40.0">
                        <tag k="name" v="Node2"/>
                    </node>
                    <way id="1">
                        <nd ref="1"/>
                        <nd ref="2"/>
                        <tag k="name" v="Way1"/>
                    </way>
                </osm>
            )";
            auto dataSource = std::make_shared<CStringDataSource>(xmlData);
            XMLReader = std::make_shared<CXMLReader>(dataSource);
            OpenStreetMap = std::make_shared<COpenStreetMap>(XMLReader);
        }
};

// Test NodeCount
TEST_F(COpenStreetMapTest, NodeCount) {
    EXPECT_EQ(OpenStreetMap->NodeCount(), 2);
}

// Test WayCount
TEST_F(COpenStreetMapTest, WayCount) {
    EXPECT_EQ(OpenStreetMap->WayCount(), 1);
}

// Test NodeByIndex
TEST_F(COpenStreetMapTest, NodeByIndex) {
    auto node = OpenStreetMap->NodeByIndex(0);
    ASSERT_NE(node, nullptr);
    EXPECT_EQ(node->ID(), 1);
    EXPECT_EQ(node->Location().first, 10.0);
    EXPECT_EQ(node->Location().second, 20.0);
    EXPECT_TRUE(node->HasAttribute("name"));
    EXPECT_EQ(node->GetAttribute("name"), "Node1");
}

// Test NodeByID
TEST_F(COpenStreetMapTest, NodeByID) {
    auto node = OpenStreetMap->NodeByID(2);
    ASSERT_NE(node, nullptr);
    EXPECT_EQ(node->ID(), 2);
    EXPECT_EQ(node->Location().first, 30.0);
    EXPECT_EQ(node->Location().second, 40.0);
    EXPECT_TRUE(node->HasAttribute("name"));
    EXPECT_EQ(node->GetAttribute("name"), "Node2");
}

// Test WayByIndex
TEST_F(COpenStreetMapTest, WayByIndex) {
    auto way = OpenStreetMap->WayByIndex(0);
    ASSERT_NE(way, nullptr);
    EXPECT_EQ(way->ID(), 1);
    EXPECT_EQ(way->NodeCount(), 2);
    EXPECT_EQ(way->GetNodeID(0), 1);
    EXPECT_EQ(way->GetNodeID(1), 2);
    EXPECT_TRUE(way->HasAttribute("name"));
    EXPECT_EQ(way->GetAttribute("name"), "Way1");
}

// Test WayByID
TEST_F(COpenStreetMapTest, WayByID) {
    auto way = OpenStreetMap->WayByID(1);
    ASSERT_NE(way, nullptr);
    EXPECT_EQ(way->ID(), 1);
    EXPECT_EQ(way->NodeCount(), 2);
    EXPECT_EQ(way->GetNodeID(0), 1);
    EXPECT_EQ(way->GetNodeID(1), 2);
    EXPECT_TRUE(way->HasAttribute("name"));
    EXPECT_EQ(way->GetAttribute("name"), "Way1");
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
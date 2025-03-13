#include "DijkstraPathRouter.h"
#include <vector>
#include <unordered_map>
#include <queue>
#include <limits>
#include <any>
#include <chrono>
#include <algorithm>
#include <iostream>

// Define the SImplementation struct
struct CDijkstraPathRouter::SImplementation {

    // Create a vertex struct to store the vertex tag and adjacency list
    struct Vertex {
        std::any Tag;
        std::vector<std::pair<TVertexID, double>> Edges; // Adjacency list with weights
    };

    // Define a custom comparator for the priority queue
    struct CompareDist {
        // Since we are working with references to the distance map, when it is modified, the priority queue is updated
        std::unordered_map<TVertexID, double>& Dist;
        // Initialize the comparator with the distance map
        CompareDist(std::unordered_map<TVertexID, double>& dist) : Dist(dist) {}
        // Compare the distances of two vertices
        bool operator()(TVertexID lhs, TVertexID rhs) const {
            // Similar to std::greater but using the distance map
            // Comparisons that return false have higher priority, so this creates a min-heap
            return Dist[lhs] > Dist[rhs];
        }
    };

    // Define the graph as an unordered map of vertices
    std::unordered_map<TVertexID, Vertex> Graph;
    // Initialize the next vertex ID to 0 when the struct is created
    TVertexID NextVertexID = 0;
    // Define infinity as the maximum value of a double
    const double INF = std::numeric_limits<double>::infinity();

    TVertexID AddVertex(std::any tag) noexcept {
        // Increment the vertex ID
        NextVertexID++;
        Vertex NewVertex;
        NewVertex.Tag = tag;
        // Add the new vertex to the graph
        Graph[NextVertexID] = NewVertex;
        return NextVertexID;
    }

    std::any GetVertexTag(TVertexID id) const noexcept {
        auto Tag = Graph.find(id);
        // find() returns an iterator to the end of the map if the key is not found
        if (Tag != Graph.end()) {
            return Tag->second.Tag;
        }
        return std::any();
    }

    bool AddEdge(TVertexID src, TVertexID dest, double weight, bool bidir) noexcept {
        if (Graph.find(src) == Graph.end() || Graph.find(dest) == Graph.end()) {
            return false;
        }
        // Graph[src] returns a vertex object
        // Since Graph[src].Edges is an unordered map in the vertex object
        Graph[src].Edges.push_back(std::pair(dest, weight)); // Add edge from src to dest with weight
        if (bidir) {
            Graph[dest].Edges.push_back(std::pair(src, weight)); // Add edge from dest to src with weight
        }
        return true;
    }

    bool Precompute(std::chrono::steady_clock::time_point deadline) noexcept {
        // Perform any desired precomputation here
        // For example, we can precompute shortest paths between all pairs of vertices
        // using the Floyd-Warshall algorithm or any other suitable algorithm

        // Check if the current time is past the deadline
        while (std::chrono::steady_clock::now() < deadline) {
            // std::cout << "Precomputing..." << std::endl;
        }

        // Return true if precomputation was successful, false otherwise
        return true;
    }

    double FindShortestPath(TVertexID src, TVertexID dest, std::vector<TVertexID> &path) noexcept {
        // Initialize the distance map and parent vertex map
        std::unordered_map<TVertexID, double> Dist;
        std::unordered_map<TVertexID, TVertexID> ParentVertex; // Parent vertex helps reconstruct the path
        
        // Initialize the distance of all vertices to infinity and the parent vertex to -1
        for (auto &Vertex : Graph) {
            Dist[Vertex.first] = INF;
            ParentVertex[Vertex.first] = -1;
        }

        // Define a priority queue with the custom comparator
        std::priority_queue<TVertexID, std::vector<TVertexID>, CompareDist> PQ{CompareDist(Dist)};
        
        // Set the distance of the source vertex to 0 and push it to the priority queue
        Dist[src] = 0;
        PQ.push(src);

        // Perform Dijkstra's algorithm
        while (!PQ.empty()) {
            // Get the vertex with the smallest distance
            TVertexID u = PQ.top();
            // Remove the vertex from the priority queue
            PQ.pop();
            
            // Break if the destination vertex is reached
            if (u == dest) {
                break;
            }

            // Iterate over the adjacent vertices
            // Graph[u].Edges is the adjacency list of vertex u
            for (const auto& [v, weight] : Graph[u].Edges) {
                // If the distance to vertex v through u is shorter than the current distance to v
                // Update the distance and parent vertex, and push v to the priority queue
                if (Dist[u] + weight < Dist[v]) {
                    Dist[v] = Dist[u] + weight;
                    ParentVertex[v] = u;
                    PQ.push(v); // Should automatically update the priority queue ordering
                }
            }
            // std::cout << "Current vertex: " << u << ", Distance: " << Dist[u] << std::endl;
        }

        // Reconstruct the path
        path.clear();
        TVertexID CurrentVertex = dest;
        
        // Traverse the parent vertices from the destination to the source
        for (size_t i = 0; i < Graph.size(); i++) {
            path.push_back(CurrentVertex);
            CurrentVertex = ParentVertex[CurrentVertex]; // Update the current vertex to the parent vertex
            if (CurrentVertex == -1) {
                break;
            }
        }
        std::reverse(path.begin(), path.end()); // Reverse the path to get the correct order

        if (path.empty()) {
            path.clear();
            return CPathRouter::NoPathExists;
        } else if (path[0] != src || path.back() != dest) {
            path.clear();
            return CPathRouter::NoPathExists;
        }
        // Return the distance to the destination vertex
        return Dist[dest];
    }
};
            

CDijkstraPathRouter::CDijkstraPathRouter() {
    // Constructor implementation
    DImplementation = std::make_unique<SImplementation>();
}

CDijkstraPathRouter::~CDijkstraPathRouter() = default;

std::size_t CDijkstraPathRouter::VertexCount() const noexcept {
    return DImplementation->Graph.size();
}

CDijkstraPathRouter::TVertexID CDijkstraPathRouter::AddVertex(std::any tag) noexcept {
    return DImplementation->AddVertex(tag);
}

std::any CDijkstraPathRouter::GetVertexTag(TVertexID id) const noexcept {
    return DImplementation->GetVertexTag(id);
}

bool CDijkstraPathRouter::AddEdge(TVertexID src, TVertexID dest, double weight, bool bidir) noexcept {
    return DImplementation->AddEdge(src, dest, weight, bidir);
}

bool CDijkstraPathRouter::Precompute(std::chrono::steady_clock::time_point deadline) noexcept {
    return DImplementation->Precompute(deadline);
}

double CDijkstraPathRouter::FindShortestPath(TVertexID src, TVertexID dest, std::vector<TVertexID> &path) noexcept {
    return DImplementation->FindShortestPath(src, dest, path);
}
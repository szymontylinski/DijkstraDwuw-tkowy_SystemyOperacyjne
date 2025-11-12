#pragma once

#include <vector>
#include <random>
#include <utility> // Dla std::pair

// Używamy typu long long dla wag, aby uniknąć przepełnienia
using Cost = long long;
// Para {wierzchołek docelowy, koszt}
using Edge = std::pair<int, Cost>;
// Lista sąsiedztwa
using AdjList = std::vector<std::vector<Edge>>;

class Graph {
public:
    // Konstruktor tworzy pusty graf o N wierzchołkach
    Graph(int n) : num_vertices(n), adj(n) {}

    // Dodaje dwukierunkową krawędź
    void addEdge(int u, int v, Cost weight) {
        adj[u].push_back({v, weight});
        adj[v].push_back({u, weight});
    }

    int getVertexCount() const {
        return num_vertices;
    }

    const std::vector<Edge>& getNeighbors(int u) const {
        return adj[u];
    }

    // Funkcja statyczna do generowania losowego grafu
    // N = liczba wierzchołków
    // density = procentowe wypełnienie krawędzi (0.0 do 1.0) 
    static Graph generateRandom(int N, double density) {
        Graph g(N);
        std::mt19937 rng(std::random_device{}()); // Generator losowy
        std::uniform_int_distribution<int> vertex_dist(0, N - 1);
        std::uniform_int_distribution<Cost> weight_dist(1, 100);

        // Maksymalna liczba krawędzi w grafie nieskierowanym
        long long max_edges = static_cast<long long>(N) * (N - 1) / 2;
        long long num_edges = static_cast<long long>(max_edges * density);

        for (long long i = 0; i < num_edges; ++i) {
            int u = vertex_dist(rng);
            int v = vertex_dist(rng);
            if (u == v) { // Pomiń pętle własne
                i--;
                continue;
            }
            g.addEdge(u, v, weight_dist(rng));
        }
        return g;
    }

private:
    int num_vertices;
    AdjList adj;
};
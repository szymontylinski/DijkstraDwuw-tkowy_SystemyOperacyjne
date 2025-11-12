#pragma once

#include "Graph.h.cpp"
#include <queue>
#include <limits>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <iostream>

// Definicja "nieskończoności" dla kosztów
const Cost INFINITY_COST = std::numeric_limits<Cost>::max();
// Para {koszt, wierzchołek} używana w kolejkach priorytetowych
using PrioPair = std::pair<Cost, int>;

// --- Algorytm Jednowątkowy (Baza odniesienia) ---
//

class SingleThreadDijkstra {
public:
    Cost solve(const Graph& graph, int start, int end) {
        int N = graph.getVertexCount();
        std::vector<Cost> dist(N, INFINITY_COST);
        // Kolejka priorytetowa (min-heap)
        std::priority_queue<PrioPair, std::vector<PrioPair>, std::greater<PrioPair>> pq;

        dist[start] = 0;
        pq.push({0, start});

        while (!pq.empty()) {
            Cost d = pq.top().first;
            int u = pq.top().second;
            pq.pop();

            // Jeśli znaleźliśmy krótszą ścieżkę wcześniej, ignoruj
            if (d > dist[u]) {
                continue;
            }

            // Znaleziono cel
            if (u == end) {
                return dist[u];
            }

            // Relaksacja krawędzi
            for (const auto& edge : graph.getNeighbors(u)) {
                int v = edge.first;
                Cost weight = edge.second;
                if (dist[u] + weight < dist[v]) {
                    dist[v] = dist[u] + weight;
                    pq.push({dist[v], v});
                }
            }
        }

        // Jeśli pętla się zakończyła, a nie znaleźliśmy 'end',
        // to znaczy, że graf jest niespójny
        return INFINITY_COST;
    }
};

// --- Dwukierunkowy Algorytm Dwuwątkowy ---
// [cite: 4, 5]

class BidirectionalDijkstra {
public:
    Cost solve(const Graph& graph, int start, int end) {
        int N = graph.getVertexCount();

        // Współdzielone dane dla obu wątków
        std::vector<Cost> dist_fwd(N, INFINITY_COST);
        std::vector<Cost> dist_bwd(N, INFINITY_COST);
        std::vector<bool> visited_fwd(N, false);
        std::vector<bool> visited_bwd(N, false);

        // Kolejki priorytetowe - każda lokalna dla swojego wątku
        std::priority_queue<PrioPair, std::vector<PrioPair>, std::greater<PrioPair>> pq_fwd;
        std::priority_queue<PrioPair, std::vector<PrioPair>, std::greater<PrioPair>> pq_bwd;

        // Najlepsza znaleziona dotąd ścieżka (atomowa, bezpieczna wątkowo)
        std::atomic<Cost> mu(INFINITY_COST);
        // Mutex chroniący *wszystkie* współdzielone wektory
        std::mutex mtx;

        // Inicjalizacja
        dist_fwd[start] = 0;
        dist_bwd[end] = 0;
        pq_fwd.push({0, start});
        pq_bwd.push({0, end});

        // Lambda dla funkcji wątku
        auto thread_func = [&](
            std::priority_queue<PrioPair, std::vector<PrioPair>, std::greater<PrioPair>>& pq,
            std::vector<Cost>& dist,
            std::vector<bool>& visited,
            const std::vector<Cost>& other_dist,
            const std::vector<bool>& other_visited,
            bool is_forward)
        {
            while (true) {
                Cost d;
                int u;

                // --- SEKCJA KRYTYCZNA 1: Pobranie wierzchołka ---
                {
                    std::lock_guard<std::mutex> lock(mtx);
                    if (pq.empty()) {
                        break; // Kolejka pusta, kończymy wątek
                    }
                    d = pq.top().first;
                    u = pq.top().second;
                    pq.pop();

                    if (d > dist[u]) {
                        continue; // Już przetworzone
                    }

                    visited[u] = true; // Oznacz jako "settled"

                    // Sprawdź, czy spotkaliśmy drugi front
                    if (other_visited[u]) {
                        mu.store(std::min(mu.load(), d + other_dist[u]));
                    }
                }
                // --- KONIEC SEKCJI KRYTYCZNEJ 1 ---

                // Warunek stopu: jeśli koszt, który właśnie wyjęliśmy (d),
                // jest już większy niż najlepsza znaleziona ścieżka (mu),
                // nie musimy dalej szukać z tego wierzchołka.
                if (d > mu.load()) {
                    break;
                }

                // Relaksacja (poza główną blokadą, ale z blokadą na czas aktualizacji)
                for (const auto& edge : graph.getNeighbors(u)) {
                    int v = edge.first;
                    Cost weight = edge.second;
                    Cost new_dist = d + weight;

                    // --- SEKCJA KRYTYCZNA 2: Aktualizacja sąsiada ---
                    {
                        std::lock_guard<std::mutex> lock(mtx);
                        if (new_dist < dist[v]) {
                            dist[v] = new_dist;
                            pq.push({new_dist, v});
                        }
                    }
                    // --- KONIEC SEKCJI KRYTYCZNEJ 2 ---
                }
            }
        };

        // Uruchomienie dwóch wątków
        std::thread t_fwd(thread_func, std::ref(pq_fwd), std::ref(dist_fwd),
                          std::ref(visited_fwd), std::cref(dist_bwd),
                          std::cref(visited_bwd), true);

        std::thread t_bwd(thread_func, std::ref(pq_bwd), std::ref(dist_bwd),
                          std::ref(visited_bwd), std::cref(dist_fwd),
                          std::cref(visited_fwd), false);

        // Czekaj na zakończenie obu
        t_fwd.join();
        t_bwd.join();

        // Wynik
        return mu.load();
    }
};
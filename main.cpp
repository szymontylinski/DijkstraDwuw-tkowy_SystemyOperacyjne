#include "Graph.h.cpp"
#include "Solvers.h.cpp"
#include <iostream>
#include <vector>
#include <string>
#include <chrono>   // Do pomiaru czasu
#include <iomanip>  // Do ładnego formatowania
#include <random>

// Funkcja pomocnicza do pomiaru czasu wykonania
template<typename Func>
double time_execution_ms(Func f) {
    auto start = std::chrono::high_resolution_clock::now();
    f(); // Wywołaj przekazaną funkcję (lambda)
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> ms = end - start;
    return ms.count();
}

int main() {
    std::vector<int> N_values = {1000, 5000, 10000, 15000, 20000 }; // Zmniejszyłem dla szybszych testów
    std::vector<double> densities = {0.25, 0.50, 0.75, 1.00}; //
    // Dodałem testy dla grafów rzadkich
    // std::vector<double> densities = {0.01, 0.05, 0.25, 0.50};
    int iterations = 5; // Uśrednianie z 5 iteracji

    SingleThreadDijkstra single_solver;
    BidirectionalDijkstra multi_solver;

    // Generator do losowania start/end
    std::mt19937 rng(std::random_device{}());

    // --- Nagłówek tabeli wyników ---
    std::cout << std::left
              << std::setw(10) << "N"
              << std::setw(10) << "Density"
              << std::setw(20) << "Avg Single (ms)"
              << std::setw(20) << "Avg Multi (ms)"
              << std::setw(15) << "Speedup"
              << std::setw(10) << "Paths"
              << "\n";
    std::cout << std::string(85, '-') << "\n";

    // --- Pętla testująca ---
    for (int N : N_values) {
        for (double d : densities) {
            double total_single_ms = 0;
            double total_multi_ms = 0;
            int paths_found = 0;

            std::uniform_int_distribution<int> vertex_dist(0, N - 1);

            for (int i = 0; i < iterations; ++i) {
                Graph g = Graph::generateRandom(N, d);
                int start = vertex_dist(rng);
                int end = vertex_dist(rng);

                if (start == end) { i--; continue; } // Powtórz iterację

                Cost dist_single = 0;
                Cost dist_multi = 0;

                // Pomiar czasu dla algorytmu jednowątkowego
                total_single_ms += time_execution_ms([&]() {
                    dist_single = single_solver.solve(g, start, end);
                });

                // Pomiar czasu dla algorytmu dwuwątkowego
                total_multi_ms += time_execution_ms([&]() {
                    dist_multi = multi_solver.solve(g, start, end);
                });

                // Weryfikacja poprawności i obsługa grafów niespójnych
                if (dist_single != dist_multi) {
                    std::cerr << "BLAD: Niespojne wyniki! "
                              << dist_single << " vs " << dist_multi << "\n";
                }
                if (dist_single != INFINITY_COST) {
                    paths_found++;
                }
            }

            double avg_single = total_single_ms / iterations;
            double avg_multi = total_multi_ms / iterations;
            double speedup = (avg_single / avg_multi); //

            // --- Drukowanie wyników ---
            std::cout << std::left
                      << std::setw(10) << N
                      << std::setw(10) << (d * 100)
                      << std::setw(20) << avg_single
                      << std::setw(20) << avg_multi
                      << std::setw(15) << speedup
                      << std::setw(10) << (std::to_string(paths_found) + "/" + std::to_string(iterations))
                      << "\n";
        }
    }

    return 0;
}
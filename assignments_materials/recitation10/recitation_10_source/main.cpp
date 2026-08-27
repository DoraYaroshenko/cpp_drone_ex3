#include <include/Registrar.h>

#include <atomic>
#include <exception>
#include <filesystem>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace {

constexpr int instancesPerFactory = 3;
constexpr int valuesPerInstance = 3;
constexpr int workerCount = 2;

[[nodiscard]] std::string runAlgorithm(Recitation9::RegisteredAlgorithm& registration,
                                       Recitation9::Range range,
                                       int instance) {
    // The algorithm is created inside the worker that executes it.
    auto algorithm = registration.factory(range);

    std::ostringstream line;
    line << registration.name << " #" << instance << ':';
    for (int count = 0; count < valuesPerInstance; ++count) {
        line << ' ' << algorithm->getNext();
    }
    line << '\n';
    return line.str();
}

void runPath(const std::filesystem::path& path,
             Recitation9::Range range,
             std::mutex& output_mutex) {
    auto registrations = Recitation9::Registrar::instance().load(path);

    for (int instance = 1; instance <= instancesPerFactory; ++instance) {
        for (auto& registration : registrations) {
            const std::string line = runAlgorithm(registration, range, instance);
            const std::lock_guard lock{output_mutex};
            std::cout << line;
        }
    }
}

[[nodiscard]] bool runThreaded(int argc,
                               char** argv,
                               int first_path,
                               Recitation9::Range range,
                               std::mutex& output_mutex) {
    bool succeeded = true;
    std::atomic<int> next_path{first_path};
    std::vector<std::thread> workers;
    workers.reserve(workerCount);

    for (int worker_number = 1; worker_number <= workerCount; ++worker_number) {
        workers.emplace_back([&](int assigned_number) {
            while (true) {
                const int index = next_path.fetch_add(1);
                if (index >= argc) {
                    return;
                }

                {
                    const std::lock_guard lock{output_mutex};
                    std::cout << "Thread " << assigned_number << " took algorithm \""
                              << std::filesystem::path{argv[index]}.filename().string()
                              << "\"\n";
                }

                try {
                    runPath(argv[index], range, output_mutex);
                } catch (const std::exception& error) {
                    const std::lock_guard lock{output_mutex};
                    succeeded = false;
                    std::cerr << "Error loading " << argv[index] << ": "
                              << error.what() << '\n';
                }
            }
        }, worker_number);
    }

    for (std::thread& worker : workers) {
        worker.join();
    }
    return succeeded;
}

void printUsage() {
    std::cerr << "Usage: algorithm_demo <minimum> <maximum> [--threads] <algorithm.so>...\n";
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 4) {
        printUsage();
        return 1;
    }

    try {
        const Recitation9::Range range{std::stoi(argv[1]), std::stoi(argv[2])};
        const bool threaded = std::string_view{argv[3]} == "--threads";
        const int first_path = threaded ? 4 : 3;
        if (first_path == argc) {
            printUsage();
            return 1;
        }

        std::mutex output_mutex;
        if (threaded) {
            return runThreaded(argc, argv, first_path, range, output_mutex) ? 0 : 1;
        }

        for (int index = first_path; index < argc; ++index) {
            runPath(argv[index], range, output_mutex);
        }
        return 0;
    } catch (const std::exception& error) {
        printUsage();
        std::cerr << "Error: " << error.what() << '\n';
        return 1;
    }
}

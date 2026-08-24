#include "PluginLoader.h"
#include "MappingAlgorithmRegistrar.h"
#include "MissionControlRegistrar.h"
#include <iostream>
#include <string>

int main(int argc, char** argv) {
    simulator::PluginLoader loader;

    std::cout << "Simulator starting..." << std::endl;

    // Basic loading loop for any .so files passed as arguments
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg.find(".so") != std::string::npos) {
            std::cout << "Loading plugin: " << arg << std::endl;
            if (!loader.loadLibrary(arg)) {
                std::cerr << "Failed to load: " << arg << std::endl;
            }
        }
    }

    // Check what was registered
    auto& algoFactories = simulator::MappingAlgorithmRegistrar::getInstance().getFactories();
    auto& mcFactories = simulator::MissionControlRegistrar::getInstance().getFactories();

    std::cout << "Registered Algorithms: " << algoFactories.size() << std::endl;
    std::cout << "Registered Mission Controls: " << mcFactories.size() << std::endl;

    // Note: The actual instantiation of IMissionControl and IMappingAlgorithm 
    // will happen here, followed by running the simulation threads.

    // Cleanup: Clear registrars before unloading the libraries
    simulator::MappingAlgorithmRegistrar::getInstance().clear();
    simulator::MissionControlRegistrar::getInstance().clear();

    // As loader goes out of scope, its destructor calls closeLibraries(),
    // ensuring .so files are closed AFTER all objects (if any) are destroyed.
    std::cout << "Simulator finished." << std::endl;
    return 0;
}

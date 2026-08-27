#pragma once

#include <string>
#include <vector>

namespace simulator {

class PluginLoader {
public:
    PluginLoader() = default;
    ~PluginLoader();

    // Prevent copying or moving to ensure handles are managed safely
    PluginLoader(const PluginLoader&) = delete;
    PluginLoader& operator=(const PluginLoader&) = delete;

    // Loads a .so file and returns true if successful. Throws or returns false on error.
    bool loadLibrary(const std::string& path);

    // Closes all loaded libraries. Must be called after all plugin-created objects are destroyed.
    void closeLibraries();

private:
    std::vector<void*> handles_;
};

} // namespace simulator

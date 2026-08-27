#pragma once

#include <common/AlgorithmFactory.h>

#include <filesystem>
#include <mutex>
#include <string>
#include <vector>

namespace Recitation9 {

struct RegisteredAlgorithm {
    std::string name;
    AlgorithmFactory factory;
};

class Registrar {
public:
    [[nodiscard]] static Registrar& instance();

    Registrar(const Registrar&) = delete;
    Registrar& operator=(const Registrar&) = delete;
    ~Registrar();

    // Called by registration objects while load() is executing dlopen().
    void add(std::string name, AlgorithmFactory factory);
    [[nodiscard]] std::vector<RegisteredAlgorithm> load(
        const std::filesystem::path& path);

private:
    class LibraryHandle {
    public:
        explicit LibraryHandle(std::filesystem::path path);
        ~LibraryHandle();

        LibraryHandle(const LibraryHandle&) = delete;
        LibraryHandle& operator=(const LibraryHandle&) = delete;
        LibraryHandle(LibraryHandle&& other) noexcept;
        LibraryHandle& operator=(LibraryHandle&& other) noexcept;

    private:
        void close() noexcept;

        std::filesystem::path path_;
        void* handle_ = nullptr;
    };

    Registrar() = default;

    std::mutex load_mutex_;
    std::vector<LibraryHandle> libraries_;
    std::vector<RegisteredAlgorithm> algorithms_;
};

} // namespace Recitation9

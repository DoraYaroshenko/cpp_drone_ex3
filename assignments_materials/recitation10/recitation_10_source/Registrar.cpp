#include <include/Registrar.h>

#include <common/AlgorithmRegistration.h>

#include <dlfcn.h>

#include <cstddef>
#include <iostream>
#include <stdexcept>
#include <utility>

namespace Recitation9 {

Registrar& Registrar::instance() {
    static Registrar registrar;
    return registrar;
}

Registrar::~Registrar() {
    // std::function may contain code from a plugin, so destroy factories first.
    algorithms_.clear();
    libraries_.clear();
}

Registrar::LibraryHandle::LibraryHandle(std::filesystem::path path)
    : path_(std::move(path)) {
    dlerror();
    handle_ = dlopen(path_.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (handle_ == nullptr) {
        const char* error = dlerror();
        throw std::runtime_error(
            "Cannot load " + path_.string() + ": " +
            (error == nullptr ? "unknown dlopen error" : error));
    }
    std::cerr << "Loaded library: " << path_.string() << '\n';
}

Registrar::LibraryHandle::~LibraryHandle() {
    close();
}

Registrar::LibraryHandle::LibraryHandle(LibraryHandle&& other) noexcept
    : path_(std::move(other.path_)),
      handle_(std::exchange(other.handle_, nullptr)) {}

Registrar::LibraryHandle& Registrar::LibraryHandle::operator=(
    LibraryHandle&& other) noexcept {
    if (this != &other) {
        close();
        path_ = std::move(other.path_);
        handle_ = std::exchange(other.handle_, nullptr);
    }
    return *this;
}

void Registrar::LibraryHandle::close() noexcept {
    if (handle_ == nullptr) {
        return;
    }
    std::cerr << "Unloading library: " << path_.string() << '\n';
    dlclose(handle_);
    handle_ = nullptr;
}

void Registrar::add(std::string name, AlgorithmFactory factory) {
    // add() runs synchronously inside the mutex-protected dlopen() call.
    std::cerr << "Registered algorithm: " << name << '\n';
    algorithms_.push_back({std::move(name), std::move(factory)});
}

std::vector<RegisteredAlgorithm> Registrar::load(const std::filesystem::path& path) {
    const std::lock_guard lock{load_mutex_};
    const std::size_t registrations_before = algorithms_.size();

    LibraryHandle library{path};

    if (algorithms_.size() == registrations_before) {
        throw std::runtime_error(path.string() + " did not register an algorithm");
    }

    libraries_.push_back(std::move(library));
    return std::vector<RegisteredAlgorithm>(
        algorithms_.begin() + static_cast<std::ptrdiff_t>(registrations_before),
        algorithms_.end());
}

AlgorithmRegistration::AlgorithmRegistration(std::string name, AlgorithmFactory factory) {
    Registrar::instance().add(std::move(name), std::move(factory));
}

} // namespace Recitation9

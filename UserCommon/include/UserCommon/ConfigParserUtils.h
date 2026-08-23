#pragma once

#include <yaml-cpp/yaml.h>

#include <functional>
#include <stdexcept>
#include <string>


namespace user_common_330371063_324976703 {
using namespace common;



template<typename T>
T get_with_default(const YAML::Node& node, const std::string& key, const T& default_val) {
    if (node[key]) {
        return node[key].as<T>();
    }
    return default_val;
}

template<typename T>
T get_with_check(const YAML::Node& node, 
                 const std::string& key, 
                 const T& default_val, 
                 const std::function<bool(const T&)>& check_fn, 
                 const std::string& err_msg) {
    T val = get_with_default(node, key, default_val);
    if (!check_fn(val)) {
        throw std::invalid_argument("Config value check failed for '" + key + "': " + err_msg);
    }
    return val;
}



} // namespace user_common_330371063_324976703

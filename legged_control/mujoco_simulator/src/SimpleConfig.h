#pragma once
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

// Minimal replacement for ocs2::loadData::loadCppDataType
// Reads simple INI-style config: section { key value }
// Comments: # or ; to end of line

namespace {

bool loadConfigValue(const std::string& filePath, const std::string& key, std::string& value) {
    std::ifstream f(filePath);
    if (!f.is_open()) return false;
    std::string line, section;
    while (std::getline(f, line)) {
        auto pos = line.find_first_of("#;");
        if (pos != std::string::npos) line = line.substr(0, pos);
        size_t s = line.find_first_not_of(" \t\r\n");
        if (s == std::string::npos) continue;
        size_t e = line.find_last_not_of(" \t\r\n");
        line = line.substr(s, e - s + 1);
        if (line.empty()) continue;
        if (line.back() == '{') {
            std::string sec = line.substr(0, line.size() - 1);
            size_t ss = sec.find_first_not_of(" \t");
            if (ss != std::string::npos) {
                size_t se = sec.find_last_not_of(" \t");
                section = sec.substr(ss, se - ss + 1);
            }
            continue;
        }
        if (line == "}") { section.clear(); continue; }
        std::istringstream iss(line);
        std::string k, v;
        if (iss >> k >> v) {
            std::string fullKey = section.empty() ? k : section + "." + k;
            if (fullKey == key) { value = v; return true; }
        }
    }
    return false;
}

} // namespace

inline bool loadConfigDouble(const std::string& filePath, const std::string& key, double& value) {
    std::string v;
    if (!loadConfigValue(filePath, key, v)) return false;
    try { value = std::stod(v); return true; }
    catch (...) { return false; }
}

inline bool loadConfigBool(const std::string& filePath, const std::string& key, bool& value) {
    std::string v;
    if (!loadConfigValue(filePath, key, v)) return false;
    value = (v == "true" || v == "1");
    return true;
}

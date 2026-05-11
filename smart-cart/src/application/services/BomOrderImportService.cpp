#include "application/services/BomOrderImportService.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <map>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace smartcart::application::services {
namespace {

using json = nlohmann::json;
namespace fs = std::filesystem;

std::string runCommand(const std::string& command) {
    std::string output;
    FILE* pipe = popen(command.c_str(), "r");
    if (pipe == nullptr) {
        throw std::runtime_error("Не удалось запустить команду unzip");
    }

    char buffer[4096];
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        output += buffer;
    }

    const int rc = pclose(pipe);
    if (rc != 0) {
        throw std::runtime_error("unzip завершился с ошибкой");
    }
    return output;
}

std::string shellQuote(const fs::path& path) {
    std::string value = path.string();
    std::string quoted = "'";
    for (const char ch : value) {
        if (ch == '\'') {
            quoted += "'\\''";
        } else {
            quoted += ch;
        }
    }
    quoted += "'";
    return quoted;
}

std::string unzipEntry(const fs::path& path, const std::string& entry) {
    return runCommand("unzip -p " + shellQuote(path) + " " + entry);
}

std::string xmlUnescape(std::string value) {
    const std::vector<std::pair<std::string, std::string>> replacements = {
        {"&quot;", "\""},
        {"&apos;", "'"},
        {"&lt;", "<"},
        {"&gt;", ">"},
        {"&amp;", "&"},
    };

    for (const auto& [from, to] : replacements) {
        std::size_t pos = 0;
        while ((pos = value.find(from, pos)) != std::string::npos) {
            value.replace(pos, from.size(), to);
            pos += to.size();
        }
    }
    return value;
}

std::vector<std::string> readSharedStrings(const fs::path& path) {
    std::vector<std::string> values;
    std::string xml;
    try {
        xml = unzipEntry(path, "xl/sharedStrings.xml");
    } catch (...) {
        return values;
    }

    const std::regex siRe("<si[^>]*>(.*?)</si>", std::regex::icase);
    const std::regex textRe("<t[^>]*>(.*?)</t>", std::regex::icase);
    for (auto it = std::sregex_iterator(xml.begin(), xml.end(), siRe);
         it != std::sregex_iterator();
         ++it)
    {
        std::string value;
        const std::string si = (*it)[1].str();
        for (auto textIt = std::sregex_iterator(si.begin(), si.end(), textRe);
             textIt != std::sregex_iterator();
             ++textIt)
        {
            value += xmlUnescape((*textIt)[1].str());
        }
        values.push_back(value);
    }
    return values;
}

int columnIndex(const std::string& cellRef) {
    int result = 0;
    for (const char ch : cellRef) {
        if (!std::isalpha(static_cast<unsigned char>(ch))) {
            break;
        }
        result = result * 26 + std::toupper(static_cast<unsigned char>(ch)) - 'A' + 1;
    }
    return result - 1;
}

std::string cellValue(const std::string& cellXml,
                      const std::vector<std::string>& sharedStrings) {
    const bool shared = cellXml.find(" t=\"s\"") != std::string::npos ||
                        cellXml.find(" t='s'") != std::string::npos;
    const bool inlineString = cellXml.find(" t=\"inlineStr\"") != std::string::npos ||
                              cellXml.find(" t='inlineStr'") != std::string::npos;

    if (inlineString) {
        const std::regex textRe("<t[^>]*>(.*?)</t>", std::regex::icase);
        std::string out;
        for (auto it = std::sregex_iterator(cellXml.begin(), cellXml.end(), textRe);
             it != std::sregex_iterator();
             ++it)
        {
            out += xmlUnescape((*it)[1].str());
        }
        return out;
    }

    const std::regex valueRe("<v[^>]*>(.*?)</v>", std::regex::icase);
    const auto it = std::sregex_iterator(cellXml.begin(), cellXml.end(), valueRe);
    if (it == std::sregex_iterator()) {
        return {};
    }

    const std::string raw = xmlUnescape((*it)[1].str());
    if (!shared) {
        return raw;
    }

    try {
        const int index = std::stoi(raw);
        if (index >= 0 && index < static_cast<int>(sharedStrings.size())) {
            return sharedStrings[static_cast<std::size_t>(index)];
        }
    } catch (...) {
    }
    return raw;
}

std::vector<std::vector<std::string>> readRows(const fs::path& path) {
    const auto sharedStrings = readSharedStrings(path);
    const std::string xml = unzipEntry(path, "xl/worksheets/sheet1.xml");
    const std::regex rowRe("<row[^>]*>(.*?)</row>", std::regex::icase);
    const std::regex cellRe("<c\\s+[^>]*r=\"([A-Z]+\\d+)\"[^>]*>(.*?)</c>",
                            std::regex::icase);

    std::vector<std::vector<std::string>> rows;
    for (auto rowIt = std::sregex_iterator(xml.begin(), xml.end(), rowRe);
         rowIt != std::sregex_iterator();
         ++rowIt)
    {
        const std::string rowXml = (*rowIt)[1].str();
        std::vector<std::string> row;
        for (auto cellIt = std::sregex_iterator(rowXml.begin(), rowXml.end(), cellRe);
             cellIt != std::sregex_iterator();
             ++cellIt)
        {
            const int index = columnIndex((*cellIt)[1].str());
            if (index < 0) {
                continue;
            }
            if (static_cast<int>(row.size()) <= index) {
                row.resize(static_cast<std::size_t>(index + 1));
            }
            row[static_cast<std::size_t>(index)] =
                cellValue((*cellIt)[0].str(), sharedStrings);
        }
        rows.push_back(std::move(row));
    }
    return rows;
}

std::string normalizeHeader(std::string value) {
    std::string out;
    for (unsigned char ch : value) {
        if (std::isalnum(ch)) {
            out.push_back(static_cast<char>(std::tolower(ch)));
        }
    }
    return out;
}

int designatorCount(const std::string& value) {
    int total = 0;
    const std::regex tokenRe("[^,;\\s]+");
    const std::regex rangeRe("([A-Za-z]+)(\\d+)-([A-Za-z]+)?(\\d+)");
    const std::regex singleRe("[A-Za-z]+\\d+");

    for (auto it = std::sregex_iterator(value.begin(), value.end(), tokenRe);
         it != std::sregex_iterator();
         ++it)
    {
        const std::string token = (*it).str();
        std::smatch match;
        if (std::regex_match(token, match, rangeRe)) {
            const std::string prefixA = match[1].str();
            const int start = std::stoi(match[2].str());
            const std::string prefixB = match[3].str();
            const int end = std::stoi(match[4].str());
            if (prefixB.empty() || prefixB == prefixA) {
                total += std::abs(end - start) + 1;
            } else {
                total += 1;
            }
        } else if (std::regex_match(token, singleRe)) {
            total += 1;
        } else {
            total += 1;
        }
    }
    return total;
}

std::map<std::string, int> assignPriorities(
    const std::vector<std::pair<std::string, int>>& orderedParts) {
    int total = 0;
    for (const auto& [_, quantity] : orderedParts) {
        total += std::max(quantity, 1);
    }

    std::map<std::string, int> priorityByPart;
    int cumulative = 0;
    for (const auto& [partNumber, quantity] : orderedParts) {
        const double shareBefore = total > 0
            ? static_cast<double>(cumulative) / static_cast<double>(total)
            : 1.0;
        if (shareBefore < 0.50) {
            priorityByPart[partNumber] = 1;
        } else if (shareBefore < 0.85) {
            priorityByPart[partNumber] = 2;
        } else {
            priorityByPart[partNumber] = 3;
        }
        cumulative += std::max(quantity, 1);
    }
    return priorityByPart;
}

json buildOrderJson(const fs::path& bomPath) {
    const auto rows = readRows(bomPath);
    if (rows.empty()) {
        throw std::runtime_error("BOM пустой");
    }

    std::map<std::string, int> headerByName;
    for (int i = 0; i < static_cast<int>(rows.front().size()); ++i) {
        headerByName[normalizeHeader(rows.front()[static_cast<std::size_t>(i)])] = i;
    }

    if (headerByName.find("partnumber") == headerByName.end()) {
        throw std::runtime_error("В BOM нет колонки PartNumber");
    }

    const int partIndex = headerByName.at("partnumber");
    const auto quantityIt = headerByName.find("quantity");
    const auto designatorIt = headerByName.find("designator");

    std::map<std::string, int> quantityByPart;
    for (std::size_t rowIndex = 1; rowIndex < rows.size(); ++rowIndex) {
        const auto& row = rows[rowIndex];
        if (partIndex >= static_cast<int>(row.size())) {
            continue;
        }
        const std::string partNumber = row[static_cast<std::size_t>(partIndex)];
        if (partNumber.empty()) {
            continue;
        }

        int quantity = 0;
        if (quantityIt != headerByName.end() &&
            quantityIt->second < static_cast<int>(row.size()) &&
            !row[static_cast<std::size_t>(quantityIt->second)].empty())
        {
            quantity = std::stoi(row[static_cast<std::size_t>(quantityIt->second)]);
        } else if (designatorIt != headerByName.end() &&
                   designatorIt->second < static_cast<int>(row.size()))
        {
            quantity = designatorCount(row[static_cast<std::size_t>(designatorIt->second)]);
        }

        quantityByPart[partNumber] += std::max(quantity, 1);
    }

    std::vector<std::pair<std::string, int>> orderedParts(
        quantityByPart.begin(),
        quantityByPart.end());
    std::sort(orderedParts.begin(), orderedParts.end(),
              [](const auto& lhs, const auto& rhs) {
                  if (lhs.second != rhs.second) {
                      return lhs.second > rhs.second;
                  }
                  return lhs.first < rhs.first;
              });

    const auto priorityByPart = assignPriorities(orderedParts);

    json order;
    const std::string stem = bomPath.stem().string();
    const auto timestamp = std::chrono::system_clock::now().time_since_epoch().count();
    order["order_id"] = stem + "-" + std::to_string(timestamp);
    order["title"] = stem;
    order["priority"] = "normal";
    order["duration_minutes"] = 0;
    order["deadline"] = "";   // no deadline by default; set manually if needed
    order["route"] = "";      // logistics route, optional
    order["items"] = json::array();

    int targetSlot = 1;
    std::sort(orderedParts.begin(), orderedParts.end(),
              [&priorityByPart](const auto& lhs, const auto& rhs) {
                  const int leftPriority = priorityByPart.at(lhs.first);
                  const int rightPriority = priorityByPart.at(rhs.first);
                  if (leftPriority != rightPriority) {
                      return leftPriority < rightPriority;
                  }
                  if (lhs.second != rhs.second) {
                      return lhs.second > rhs.second;
                  }
                  return lhs.first < rhs.first;
              });

    for (const auto& [partNumber, quantity] : orderedParts) {
        order["items"].push_back({
            {"barcode", partNumber},
            {"part_number", partNumber},
            {"material_type", "reel"},
            {"required_quantity", quantity},
            {"usage_priority", priorityByPart.at(partNumber)},
            {"target_slot", targetSlot++},
        });
    }

    return order;
}

fs::path writeTempOrderJson(const json& order) {
    const auto path = fs::temp_directory_path() /
        ("smartcart_bom_order_" +
         std::to_string(std::chrono::system_clock::now().time_since_epoch().count()) +
         ".json");
    std::ofstream out(path);
    if (!out.is_open()) {
        throw std::runtime_error("Не удалось создать временный JSON заказа");
    }
    out << order.dump(2);
    return path;
}

} // namespace

BomOrderImportService::BomOrderImportService(OrderImportService& orderImportSvc)
    : orderImportSvc_(orderImportSvc)
{}

OrderImportResult BomOrderImportService::importFromFile(
    const std::string& bomPath) {
    try {
        const auto orderJson = buildOrderJson(fs::path(bomPath));
        const auto tempPath = writeTempOrderJson(orderJson);
        return orderImportSvc_.importFromFile(tempPath.string());
    } catch (const std::exception& ex) {
        return OrderImportResult{
            false,
            0,
            domain::ErrorCode::InvalidConfig,
            ex.what()
        };
    }
}

} // namespace smartcart::application::services

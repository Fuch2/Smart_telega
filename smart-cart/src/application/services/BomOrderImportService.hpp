#pragma once

#include "application/services/OrderImportService.hpp"

#include <string>

namespace smartcart::application::services {

class BomOrderImportService {
public:
    explicit BomOrderImportService(OrderImportService& orderImportSvc);

    OrderImportResult importFromFile(const std::string& bomPath);

private:
    OrderImportService& orderImportSvc_;
};

} // namespace smartcart::application::services

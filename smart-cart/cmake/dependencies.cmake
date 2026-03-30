# ===== cmake/dependencies.cmake =====
# Централизованный поиск всех внешних зависимостей.
# Включается из корневого CMakeLists.txt до объявления таргетов.

find_package(SQLite3       REQUIRED)
find_package(spdlog        REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(GTest         REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Widgets Core)

message(STATUS "SQLite3   : ${SQLite3_VERSION}")
message(STATUS "spdlog    : ${spdlog_VERSION}")
message(STATUS "GTest     : ${GTest_VERSION}")
message(STATUS "Qt6       : ${Qt6_VERSION}")

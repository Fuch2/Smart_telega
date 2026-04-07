cmake_minimum_required(VERSION 3.22)
project(smartcart LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(cmake/warnings.cmake)
include(cmake/sanitizers.cmake)
include(cmake/dependencies.cmake)

set(SMARTCART_MIGRATIONS_DIR
    "${CMAKE_SOURCE_DIR}/src/infrastructure/db/migrations")
set(SMARTCART_CONFIG_DIR
    "${CMAKE_SOURCE_DIR}/config")

# ══════════════════════════════════════════════════════════════════════════════
# smartcart_infra — инфраструктура + application services (БЕЗ Qt)
# ══════════════════════════════════════════════════════════════════════════════
add_library(smartcart_infra OBJECT

    # ── infrastructure: config ────────────────────────────────────────────────
    src/infrastructure/config/ConfigLoader.cpp
    src/infrastructure/config/ModuleProfileLoader.cpp

    # ── infrastructure: db ────────────────────────────────────────────────────
    src/infrastructure/db/SqliteConnection.cpp
    src/infrastructure/db/repositories/ModuleRepositorySqlite.cpp
    src/infrastructure/db/repositories/ReelRepositorySqlite.cpp
    src/infrastructure/db/repositories/OperationRepositorySqlite.cpp
    src/infrastructure/db/repositories/OrderRepositorySqlite.cpp
    src/infrastructure/db/repositories/WorkflowRepositorySqlite.cpp

    # ── infrastructure: hw/stm32 ──────────────────────────────────────────────
    src/infrastructure/hw/stm32/FrameCodec.cpp
    src/infrastructure/hw/stm32/UartStm32Link.cpp
    src/infrastructure/hw/stm32/MockStm32Link.cpp

    # ── infrastructure: hw/scanner ────────────────────────────────────────────
    src/infrastructure/hw/scanner/HidScannerProvider.cpp
    src/infrastructure/hw/scanner/SerialScannerProvider.cpp
    src/infrastructure/hw/scanner/MockScannerProvider.cpp

    # ── infrastructure: hw/rfid ───────────────────────────────────────────────
    src/infrastructure/hw/rfid/MockRfidProvider.cpp
    src/infrastructure/hw/rfid/Rc522RfidProvider.cpp

    # ── infrastructure: logging ───────────────────────────────────────────────
    src/infrastructure/logging/LoggerFactory.cpp
    src/infrastructure/logging/SqliteEventLogger.cpp
    src/infrastructure/logging/SpdlogFileLogger.cpp

    # ── application: services (чистый C++, без Qt) ────────────────────────────
    src/application/services/StartupService.cpp
    src/application/services/AddReelService.cpp
    src/application/services/ReplaceReelService.cpp
    src/application/services/RecoveryService.cpp
    src/application/services/Stm32PollingService.cpp
    src/application/services/OrderImportService.cpp
)

target_include_directories(smartcart_infra PUBLIC
    ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(smartcart_infra PUBLIC
    SQLite::SQLite3
    spdlog::spdlog
    nlohmann_json::nlohmann_json
)

smartcart_set_warnings(smartcart_infra)

# ══════════════════════════════════════════════════════════════════════════════
# smartcart_app_qt — AppStateMachine + AppBootstrap (требует Qt6)
# Отдельная OBJECT-библиотека чтобы не тащить Qt в тесты
# ══════════════════════════════════════════════════════════════════════════════
add_library(smartcart_app_qt OBJECT
    src/application/state/AppStateMachine.cpp
    apps/smartcart_app/AppBootstrap.cpp
)

target_include_directories(smartcart_app_qt PUBLIC
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/apps
)

target_link_libraries(smartcart_app_qt PUBLIC
    smartcart_infra
    Qt6::Core
    Qt6::Widgets
)

# MOC для AppStateMachine (QObject)
set_target_properties(smartcart_app_qt PROPERTIES
    AUTOMOC ON
)

smartcart_set_warnings(smartcart_app_qt)

# ══════════════════════════════════════════════════════════════════════════════
# smartcart_app — headless CLI smoke-test (CI, без Qt)
# ══════════════════════════════════════════════════════════════════════════════
add_executable(smartcart_app
    apps/smartcart_app/main.cpp
)

target_compile_definitions(smartcart_app PRIVATE
    MIGRATIONS_DIR="${SMARTCART_MIGRATIONS_DIR}"
    CONFIG_DIR="${SMARTCART_CONFIG_DIR}"
)

target_include_directories(smartcart_app PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/apps
)

target_link_libraries(smartcart_app PRIVATE
    smartcart_infra
)

add_custom_command(TARGET smartcart_app POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/config
        $<TARGET_FILE_DIR:smartcart_app>/config
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${SMARTCART_MIGRATIONS_DIR}
        $<TARGET_FILE_DIR:smartcart_app>/migrations
    COMMENT "Copying config and migrations → smartcart_app output"
)

# ══════════════════════════════════════════════════════════════════════════════
# Tests
# ══════════════════════════════════════════════════════════════════════════════
enable_testing()

# ── Unit tests: FrameCodec (без БД, без Qt) ───────────────────────────────────
add_executable(frame_codec_tests
    tests/unit/infrastructure/frame_codec_test.cpp
    tests/unit/infrastructure/stm32_stream_parser_test.cpp
    src/infrastructure/hw/stm32/FrameCodec.cpp
)

target_include_directories(frame_codec_tests PRIVATE
    ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(frame_codec_tests PRIVATE
    GTest::gtest_main
)

smartcart_set_warnings(frame_codec_tests)
smartcart_enable_sanitizers(frame_codec_tests)

# ── Unit tests: Domain (без БД, без Qt) ───────────────────────────────────────
add_executable(domain_tests
    tests/unit/domain/cart_workflow_test.cpp
)

target_include_directories(domain_tests PRIVATE
    ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(domain_tests PRIVATE
    GTest::gtest_main
)

smartcart_set_warnings(domain_tests)
smartcart_enable_sanitizers(domain_tests)

# ── Integration tests (без Qt — только smartcart_infra) ───────────────────────
add_executable(integration_tests
    tests/integration/startup_flow_test.cpp
    tests/integration/add_reel_flow_test.cpp
    tests/integration/replace_reel_flow_test.cpp
    tests/integration/uart_recovery_test.cpp
    tests/integration/order_workflow_repository_test.cpp
    tests/integration/order_import_service_test.cpp
)

target_include_directories(integration_tests PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/apps
)

target_compile_definitions(integration_tests PRIVATE
    MIGRATIONS_DIR="${SMARTCART_MIGRATIONS_DIR}"
    CONFIG_DIR="${SMARTCART_CONFIG_DIR}"
)

target_link_libraries(integration_tests PRIVATE
    smartcart_infra
    GTest::gtest_main
)

smartcart_set_warnings(integration_tests)
smartcart_enable_sanitizers(integration_tests)

include(GoogleTest)
gtest_discover_tests(frame_codec_tests)
gtest_discover_tests(domain_tests)
gtest_discover_tests(integration_tests)

# ══════════════════════════════════════════════════════════════════════════════
# Qt UI — presentation слой
# main_qt.cpp теперь в apps/smartcart_app/ (П1.5)
# ══════════════════════════════════════════════════════════════════════════════
add_subdirectory(src/presentation/qt build_qt)

# ── Синхронизация config/ в build/ при любом cmake --build ───────────────────
add_custom_target(copy_config ALL
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/config
        ${CMAKE_BINARY_DIR}/config
    COMMENT "Syncing config/ → build/config/"
)
add_dependencies(smartcart_app     copy_config)
add_dependencies(integration_tests copy_config)

add_executable(smart_cart_ui
    # ── точка входа ──────────────────────────────────────────────────────────
    ${CMAKE_CURRENT_SOURCE_DIR}/main_qt.cpp

    # ── presentation ─────────────────────────────────────────────────────────
    ${CMAKE_CURRENT_SOURCE_DIR}/MainWindow.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MainWindow.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/views/WorkerView.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/views/WorkerView.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/views/AdminView.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/views/AdminView.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/widgets/StatusPanelWidget.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/widgets/StatusPanelWidget.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/widgets/SlotGridWidget.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/widgets/SlotGridWidget.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/viewmodels/WorkerViewModel.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/viewmodels/WorkerViewModel.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/viewmodels/AdminViewModel.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/viewmodels/AdminViewModel.hpp

    # ── bootstrap ─────────────────────────────────────────────────────────────
    ${CMAKE_SOURCE_DIR}/apps/smartcart_app/AppBootstrap.cpp

    # ── infrastructure: config ────────────────────────────────────────────────
    ${CMAKE_SOURCE_DIR}/src/infrastructure/config/ConfigLoader.cpp

    # ── infrastructure: db ────────────────────────────────────────────────────
    ${CMAKE_SOURCE_DIR}/src/infrastructure/db/SqliteConnection.cpp
    ${CMAKE_SOURCE_DIR}/src/infrastructure/db/repositories/ModuleRepositorySqlite.cpp
    ${CMAKE_SOURCE_DIR}/src/infrastructure/db/repositories/ReelRepositorySqlite.cpp
    ${CMAKE_SOURCE_DIR}/src/infrastructure/db/repositories/OperationRepositorySqlite.cpp

    # ── infrastructure: logging ───────────────────────────────────────────────
    ${CMAKE_SOURCE_DIR}/src/infrastructure/logging/LoggerFactory.cpp
    ${CMAKE_SOURCE_DIR}/src/infrastructure/logging/SqliteEventLogger.cpp

    # ── infrastructure: stm32 ─────────────────────────────────────────────────
    ${CMAKE_SOURCE_DIR}/src/infrastructure/hw/stm32/UartStm32Link.cpp
    ${CMAKE_SOURCE_DIR}/src/infrastructure/hw/stm32/FrameCodec.cpp
)
# ── Qt6 автоматическая обработка MOC/UIC/RCC ──────────────────────────────
set_target_properties(smart_cart_ui PROPERTIES
    AUTOMOC ON
    AUTOUIC ON
    AUTORCC ON
)

target_include_directories(smart_cart_ui BEFORE PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/apps
    ${CMAKE_CURRENT_SOURCE_DIR}
)


target_compile_definitions(smart_cart_ui PRIVATE
    MIGRATIONS_DIR="${SMARTCART_MIGRATIONS_DIR}"
    CONFIG_DIR="${SMARTCART_CONFIG_DIR}"
)

target_link_libraries(smart_cart_ui PRIVATE
    Qt6::Widgets
    Qt6::Core
    SQLite::SQLite3
    spdlog::spdlog
    nlohmann_json::nlohmann_json
)


# ===== src/presentation/qt/CMakeLists.txt =====
# Исправлено:
#   - используем smartcart_infra вместо дублирования .cpp
#   - добавлен AppBootstrap
#   - добавлены SpdlogFileLogger, scanner, rfid провайдеры

add_executable(smart_cart_ui
    # ── точка входа ──────────────────────────────────────────────────────────
    ${CMAKE_CURRENT_SOURCE_DIR}/main_qt.cpp

    # ── bootstrap ────────────────────────────────────────────────────────────
    ${CMAKE_SOURCE_DIR}/apps/smartcart_app/AppBootstrap.hpp

    # ── presentation: main window ─────────────────────────────────────────────
    ${CMAKE_CURRENT_SOURCE_DIR}/MainWindow.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MainWindow.hpp

    # ── presentation: views ───────────────────────────────────────────────────
    ${CMAKE_CURRENT_SOURCE_DIR}/views/WorkerView.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/views/WorkerView.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/views/AdminView.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/views/AdminView.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/views/DiagnosticsView.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/views/DiagnosticsView.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/views/ErrorDialog.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/views/ErrorDialog.hpp

    # ── presentation: widgets ─────────────────────────────────────────────────
    ${CMAKE_CURRENT_SOURCE_DIR}/widgets/StatusPanelWidget.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/widgets/StatusPanelWidget.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/widgets/SlotGridWidget.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/widgets/SlotGridWidget.hpp

    # ── presentation: viewmodels ──────────────────────────────────────────────
    ${CMAKE_CURRENT_SOURCE_DIR}/viewmodels/WorkerViewModel.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/viewmodels/WorkerViewModel.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/viewmodels/AdminViewModel.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/viewmodels/AdminViewModel.hpp

    # ── presentation: presenters ──────────────────────────────────────────────
    ${CMAKE_CURRENT_SOURCE_DIR}/presenters/AdminPresenter.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/presenters/AdminPresenter.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/presenters/WorkerPresenter.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/presenters/WorkerPresenter.hpp
)

set_target_properties(smart_cart_ui PROPERTIES
    AUTOMOC ON
    AUTOUIC ON
    AUTORCC ON
)

target_include_directories(smart_cart_ui BEFORE PRIVATE
    ${CMAKE_SOURCE_DIR}/src        # → #include "infrastructure/..."
    ${CMAKE_SOURCE_DIR}/apps       # → #include "smartcart_app/AppBootstrap.hpp"
    ${CMAKE_CURRENT_SOURCE_DIR}    # → #include "MainWindow.hpp" и т.д.
)

target_compile_definitions(smart_cart_ui PRIVATE
    MIGRATIONS_DIR="${SMARTCART_MIGRATIONS_DIR}"
    CONFIG_DIR="${SMARTCART_CONFIG_DIR}"
)

# smartcart_infra уже тянет SQLite3, spdlog, nlohmann_json
target_link_libraries(smart_cart_ui PRIVATE
    smartcart_infra
    smartcart_app_qt
    Qt6::Widgets
    Qt6::Core
)

add_custom_command(TARGET smart_cart_ui POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/config
        $<TARGET_FILE_DIR:smart_cart_ui>/config
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${SMARTCART_MIGRATIONS_DIR}
        $<TARGET_FILE_DIR:smart_cart_ui>/migrations
)

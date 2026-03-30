set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

add_executable(smart_cart_ui
    ${CMAKE_CURRENT_SOURCE_DIR}/main_qt.cpp
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

    ${CMAKE_SOURCE_DIR}/src/infrastructure/persistence/SqliteModuleRepository.cpp
)

# ↓ имя таргета исправлено: smartcart_qt → smart_cart_ui
target_compile_definitions(smart_cart_ui PRIVATE
    DB_PATH="$<TARGET_FILE_DIR:smart_cart_ui>/smartcart.db"
)

target_include_directories(smart_cart_ui PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(smart_cart_ui PRIVATE
    Qt6::Widgets
    Qt6::Core
    SQLite::SQLite3
)

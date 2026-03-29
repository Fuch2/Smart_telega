cmake_minimum_required(VERSION 3.21)

project(smart_cart_ui LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt6 REQUIRED COMPONENTS Widgets)

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
    ${CMAKE_SOURCE_DIR}/src/infrastructure/persistence/SqliteModuleRepository.hpp
    ${CMAKE_SOURCE_DIR}/src/domain/repositories/IModuleRepository.hpp
    ${CMAKE_SOURCE_DIR}/src/domain/entities/ModuleEntity.hpp
)


target_link_libraries(smart_cart_ui PRIVATE Qt6::Widgets)

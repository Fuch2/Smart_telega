cmake_minimum_required(VERSION 3.22)
project(smartcart LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(SQLite3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Widgets Core)

add_executable(smartcart_app
    apps/smartcart_app/main.cpp
    src/infrastructure/config/ConfigLoader.cpp
    src/infrastructure/db/SqliteConnection.cpp
    src/infrastructure/logging/LoggerFactory.cpp
    src/infrastructure/logging/SqliteEventLogger.cpp
)

add_custom_command(TARGET smartcart_app POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/config
        $<TARGET_FILE_DIR:smartcart_app>/config
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/src/infrastructure/db/migrations
        $<TARGET_FILE_DIR:smartcart_app>/migrations
)



target_include_directories(smartcart_app PRIVATE
    src
)

target_link_libraries(smartcart_app PRIVATE
    SQLite::SQLite3
    spdlog::spdlog
    nlohmann_json::nlohmann_json
)

# ---------------- TESTS ----------------
enable_testing()
find_package(GTest REQUIRED)

add_executable(frame_codec_tests
    tests/unit/infrastructure/frame_codec_test.cpp
    tests/unit/infrastructure/stm32_stream_parser_test.cpp
    src/infrastructure/hw/stm32/FrameCodec.cpp
)

target_include_directories(frame_codec_tests PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(frame_codec_tests PRIVATE
    GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(frame_codec_tests)

add_subdirectory(src/presentation/qt build_qt)

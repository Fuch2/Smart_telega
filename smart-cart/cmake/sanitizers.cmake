# ===== cmake/sanitizers.cmake =====
# Функция smartcart_enable_sanitizers(target) — подключает ASan + UBSan
# для тестовых таргетов при сборке в Debug или при явном флаге.
#
# Использование:
#   cmake -DSMARTCART_SANITIZE=ON -DCMAKE_BUILD_TYPE=Debug ..

option(SMARTCART_SANITIZE "Enable AddressSanitizer + UBSan for tests" OFF)

function(smartcart_enable_sanitizers target)
    if(NOT SMARTCART_SANITIZE)
        return()
    endif()

    if(MSVC)
        message(WARNING "Sanitizers not supported on MSVC — skipping ${target}")
        return()
    endif()

    target_compile_options(${target} PRIVATE
        -fsanitize=address,undefined
        -fno-omit-frame-pointer
        -g
    )
    target_link_options(${target} PRIVATE
        -fsanitize=address,undefined
    )
    message(STATUS "Sanitizers enabled for: ${target}")
endfunction()

# ===== cmake/warnings.cmake =====
# Функция smartcart_set_warnings(target) — единое место управления
# предупреждениями компилятора для всех таргетов проекта.

function(smartcart_set_warnings target)
    if(MSVC)
        target_compile_options(${target} PRIVATE
            /W4
            /WX             # warnings as errors
            /wd4251         # dll-interface warning (Qt)
            /wd4275
        )
    else()
        target_compile_options(${target} PRIVATE
            -Wall
            -Wextra
            -Wpedantic
            -Wshadow
            -Wnon-virtual-dtor
            -Wold-style-cast
            -Wcast-align
            -Woverloaded-virtual
            -Wnull-dereference
            -Wdouble-promotion
            -Wformat=2
            # Не делаем -Werror в продакшне — только в CI через опцию
            $<$<BOOL:${SMARTCART_WERROR}>:-Werror>
        )
    endif()
endfunction()

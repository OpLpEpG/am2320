# SPDX-License-Identifier: Apache-2.0
if(CONFIG_AM2320)

message("UMDOM: modul AM2320 Запуск сборки.")

zephyr_include_directories( . )


zephyr_library()

zephyr_library_sources(am2320.c)
    
endif(CONFIG_AM2320)


if (TARGET tinyusb_device)
    add_executable(hello_usb
            hello_usb.c
            ymodem.c
            )

    #add_compile_definitions(PICO_FLASH_SPI_CLKDIV=4)

    pico_generate_pio_header(hello_usb ${CMAKE_CURRENT_LIST_DIR}/pwmin.pio)
    pico_generate_pio_header(hello_usb ${CMAKE_CURRENT_LIST_DIR}/sm_to_buf.pio)
    # pull in common dependencies
    target_link_libraries(hello_usb pico_stdlib hardware_pio hardware_dma)

    # enable usb output, disable uart output
    pico_enable_stdio_usb(hello_usb 1)
    pico_enable_stdio_uart(hello_usb 0)

    # create map/bin/hex/uf2 file etc.
    pico_add_extra_outputs(hello_usb)

    # add url via pico_set_program_url
    example_auto_set_url(hello_usb)

    pico_define_boot_stage2(slower_boot2 ${PICO_DEFAULT_BOOT_STAGE2_FILE})
    target_compile_definitions(slower_boot2 PRIVATE PICO_FLASH_SPI_CLKDIV=4)

    pico_set_boot_stage2(hello_usb slower_boot2)

elseif(PICO_ON_DEVICE)
    message(WARNING "not building hello_usb because TinyUSB submodule is not initialized in the SDK")
endif()

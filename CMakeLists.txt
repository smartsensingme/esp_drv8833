set(pwm_source "src/pwm_ledc.c")
if(CONFIG_DRV8833_PWM_MCPWM)
    set(pwm_source "src/pwm_mcpwm.c")
endif()
idf_component_register(SRCS "src/esp_drv8833.c" ${pwm_source}
    INCLUDE_DIRS "include" PRIV_INCLUDE_DIRS "src"
    REQUIRES esp_driver_gpio
    PRIV_REQUIRES esp_driver_ledc esp_driver_mcpwm esp_rom esp_timer freertos)

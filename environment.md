# 硬件环境：
- stm32f103c8t6
- W25Q128：
    - CLK->PB13
    - CS->PB12
    - DO->PB14
    - DI->PB15
# 软件环境：
- stm32 标准库
- 目录结构：
```txt
environment.md
│  EventRecorderStub.scvd
│  keilkilll.bat
│  startup_stm32f10x_md.s
│  Template.uvguix.86157
│  Template.uvguix.Administrator
│  Template.uvguix.Chen
│  Template.uvguix.YB102
│  Template.uvoptx
│  Template.uvprojx
│
├─APP
│      Display.c
│      Display.h
│      start_task.c
│      start_task.h
│      State.c
│      State.h
│
├─DebugConfig
│      Target_1_STM32F103C8_1.0.0.dbgconf
│      Target_1_STM32F103ZE_1.0.0.dbgconf
│
├─freertos
│  │  FreeRTOSConfig.h
│  │
│  ├─inc
│  │      atomic.h
│  │      croutine.h
│  │      deprecated_definitions.h
│  │      event_groups.h
│  │      FreeRTOS.h
│  │      list.h
│  │      message_buffer.h
│  │      mpu_prototypes.h
│  │      mpu_wrappers.h
│  │      portable.h
│  │      projdefs.h
│  │      queue.h
│  │      semphr.h
│  │      StackMacros.h
│  │      stack_macros.h
│  │      stdint.readme
│  │      stream_buffer.h
│  │      task.h
│  │      timers.h
│  │
│  ├─port
│  │      port.c
│  │      portmacro.h
│  │
│  └─src
│          croutine.c
│          event_groups.c
│          heap_1.c
│          heap_2.c
│          heap_3.c
│          heap_4.c
│          heap_5.c
│          list.c
│          queue.c
│          stream_buffer.c
│          tasks.c
│          timers.c
│
├─HARDWARE
│  ├─beep
│  │      beep.c
│  │      beep.h
│  │
│  ├─exti
│  │      exti.c
│  │      exti.h
│  │
│  ├─key
│  │      key.c
│  │      key.h
│  │
│  ├─lcd
│  │      lcd.c
│  │      lcd.h
│  │      lcdfont.h
│  │      lcd_init.c
│  │      lcd_init.h
│  │      pic.h
│  │
│  ├─led
│  │      led.c
│  │      led.h
│  │
│  ├─pwm
│  │      pwm.c
│  │      pwm.h
│  │
│  ├─smg
│  │      smg.c
│  │      smg.h
│  │
│  ├─time
│  │      time.c
│  │      time.h
│  │
│  └─w25q128
│          driver_w25qxx.c
│          driver_w25qxx.h
│          driver_w25qxx_interface.h
│          driver_w25qxx_interface_template.c
│
├─Libraries
│  ├─CMSIS
│  │      core_cm3.c
│  │      core_cm3.h
│  │      startup_stm32f10x_hd.s
│  │      system_stm32f10x.c
│  │      system_stm32f10x.h
│  │
│  └─STM32F10x_StdPeriph_Driver
│      ├─inc
│      │      misc.h
│      │      stm32f10x_adc.h
│      │      stm32f10x_bkp.h
│      │      stm32f10x_can.h
│      │      stm32f10x_cec.h
│      │      stm32f10x_crc.h
│      │      stm32f10x_dac.h
│      │      stm32f10x_dbgmcu.h
│      │      stm32f10x_dma.h
│      │      stm32f10x_exti.h
│      │      stm32f10x_flash.h
│      │      stm32f10x_fsmc.h
│      │      stm32f10x_gpio.h
│      │      stm32f10x_i2c.h
│      │      stm32f10x_iwdg.h
│      │      stm32f10x_pwr.h
│      │      stm32f10x_rcc.h
│      │      stm32f10x_rtc.h
│      │      stm32f10x_sdio.h
│      │      stm32f10x_spi.h
│      │      stm32f10x_tim.h
│      │      stm32f10x_usart.h
│      │      stm32f10x_wwdg.h
│      │
│      └─src
│              misc.c
│              stm32f10x_adc.c
│              stm32f10x_bkp.c
│              stm32f10x_can.c
│              stm32f10x_cec.c
│              stm32f10x_crc.c
│              stm32f10x_dac.c
│              stm32f10x_dbgmcu.c
│              stm32f10x_dma.c
│              stm32f10x_exti.c
│              stm32f10x_flash.c
│              stm32f10x_fsmc.c
│              stm32f10x_gpio.c
│              stm32f10x_i2c.c
│              stm32f10x_iwdg.c
│              stm32f10x_pwr.c
│              stm32f10x_rcc.c
│              stm32f10x_rtc.c
│              stm32f10x_sdio.c
│              stm32f10x_spi.c
│              stm32f10x_tim.c
│              stm32f10x_usart.c
│              stm32f10x_wwdg.c
│
├─Obj
│  │  core_cm3.crf
│  │  core_cm3.d
│  │  core_cm3.o
│  │  croutine.crf
│  │  croutine.d
│  │  croutine.o
│  │  delay.crf
│  │  delay.d
│  │  delay.o
│  │  display.crf
│  │  display.d
│  │  display.o
│  │  event_groups.crf
│  │  event_groups.d
│  │  event_groups.o
│  │  heap_4.crf
│  │  heap_4.d
│  │  heap_4.o
│  │  lcd.crf
│  │  lcd.d
│  │  lcd.o
│  │  lcd_1.crf
│  │  lcd_1.d
│  │  lcd_1.o
│  │  lcd_init.crf
│  │  lcd_init.d
│  │  lcd_init.o
│  │  lcd_init_1.crf
│  │  lcd_init_1.d
│  │  lcd_init_1.o
│  │  led.crf
│  │  led.d
│  │  led.o
│  │  led_1.crf
│  │  led_1.d
│  │  led_1.o
│  │  list.crf
│  │  list.d
│  │  list.o
│  │  main.crf
│  │  main.d
│  │  main.o
│  │  misc.crf
│  │  misc.d
│  │  misc.o
│  │  port.crf
│  │  port.d
│  │  port.o
│  │  queue.crf
│  │  queue.d
│  │  queue.o
│  │  startup_stm32f10x_hd.d
│  │  startup_stm32f10x_hd.lst
│  │  startup_stm32f10x_hd.o
│  │  startup_stm32f10x_md.d
│  │  startup_stm32f10x_md.lst
│  │  startup_stm32f10x_md.o
│  │  start_task.crf
│  │  start_task.d
│  │  start_task.o
│  │  state.crf
│  │  state.d
│  │  state.o
│  │  stm32f10x_exti.crf
│  │  stm32f10x_exti.d
│  │  stm32f10x_exti.o
│  │  stm32f10x_gpio.crf
│  │  stm32f10x_gpio.d
│  │  stm32f10x_gpio.o
│  │  stm32f10x_it.crf
│  │  stm32f10x_it.d
│  │  stm32f10x_it.o
│  │  stm32f10x_rcc.crf
│  │  stm32f10x_rcc.d
│  │  stm32f10x_rcc.o
│  │  stm32f10x_tim.crf
│  │  stm32f10x_tim.d
│  │  stm32f10x_tim.o
│  │  stm32f10x_usart.crf
│  │  stm32f10x_usart.d
│  │  stm32f10x_usart.o
│  │  stream_buffer.crf
│  │  stream_buffer.d
│  │  stream_buffer.o
│  │  sys.crf
│  │  sys.d
│  │  sys.o
│  │  system.crf
│  │  system.d
│  │  system.o
│  │  system_stm32f10x.crf
│  │  system_stm32f10x.d
│  │  system_stm32f10x.o
│  │  systick.crf
│  │  systick.d
│  │  systick.o
│  │  tasks.crf
│  │  tasks.d
│  │  tasks.o
│  │  Template.axf
│  │  Template.bin
│  │  Template.build_log.htm
│  │  Template.hex
│  │  Template.htm
│  │  Template.lnp
│  │  Template.map
│  │  Template.sct
│  │  Template_sct.Bak
│  │  Template_Target 1.dep
│  │  timers.crf
│  │  timers.d
│  │  timers.o
│  │  usart.crf
│  │  usart.d
│  │  usart.o
│  │
│  ├─Listings
│  └─Objects
├─Public
│      system.c
│      system.h
│      SysTick.c
│      SysTick.h
│      usart.c
│      usart.h
│
└─User
        delay.c
        delay.h
        main.c
        stm32f10x.h
        stm32f10x_conf.h
        stm32f10x_it.c
        stm32f10x_it.h
        sys.c
        sys.h
```
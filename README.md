# microJect
MQL lubrication system using automotive injectors to lubricate cutting tools.

![]("Graphics/board_render.png")

Control is via modbus RTU on RS485 or via switches and pots (Two channels only in that mode). 
Power input is 24V and power delivered to the injector heads is 12V at up to about 3A total from an on board power converter. 

Firmware is written in Rust with Embassy as the framework and the processor on the board is an STM32F030k6. 

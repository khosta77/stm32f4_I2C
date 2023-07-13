# stm32f4 I2C пример

# Залитие на stm32f407vg discovery

```
$ make all; sudo openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c \
"init; reset halt; flash write_image erase main.hex; "\  
"reset; exit"
```

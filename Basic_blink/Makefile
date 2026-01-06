CC = arm-none-eabi-gcc
CFLAGS = -mcpu=cortex-m4 -mthumb -Wall -Wextra -O2
LDFLAGS = -T stm32f401RE.ld -Wl,--gc-sections

TARGET = blink.elf
SOURCES = main.c startup_stm32f401.s

$(TARGET): $(SOURCES)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $(TARGET) $(SOURCES)

clean:
	rm -f $(TARGET)

flash:
	openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
		-c "program $(TARGET) verify reset exit"

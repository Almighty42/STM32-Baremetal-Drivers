CC = arm-none-eabi-gcc
ARC = cortex-m4

CFLAGS = -mcpu=$(ARC) -mthumb -mfloat-abi=soft -std=gnu11 -Wall -O0
LDFLAGS = -nostartfiles -nodefaultlibs -mcpu=$(ARC) -mthumb -mfloat-abi=soft -T stm32_ls.ld -Wl,-Map=blink.map

TARGET = blink.elf
SOURCES = main.c startup.c
OBJECTS = $(SOURCES:.c=.o)

$(TARGET): $(OBJECTS)
	$(CC) $(LDFLAGS) -o $@ $(OBJECTS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

%.o: %.s
	$(CC) $(CFLAGS) -c $< -o $@

all: $(TARGET)  
	rm -f $(OBJECTS) *.map

clean:
	rm -f $(TARGET) $(OBJECTS) *.map

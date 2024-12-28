build: firmware.bin

main.o:
	arm-none-eabi-gcc -mcpu=cortex-m0plus main.c -c

firmware.elf: main.o
	arm-none-eabi-gcc -T link.ld -nostdlib main.o -o firmware.elf

firmware.bin: firmware.elf
	arm-none-eabi-objcopy -O binary firmware.elf firmware.bin

flash: firmware.bin
	st-flash --reset write firmware.bin 0x8000000

clean:
	rm -rf firmware.*
	rm -rf main.o

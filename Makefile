
TARGET = example2
LDFLAGS = -g -m32 -L ../../linux/Lib -L ../../../libusb/libusb-0.1.12/.libs/
LIBS = -lusb -lZestSC1
OBJS = Example2.o
CC = gcc
CFLAGS+= -Wall -pedantic -O2 -I../../Inc -m32
LD = gcc
BITFILE := FPGA-VHDL/Example2.bit

all:	console

clean:
	rm $(TARGET)
	rm $(OBJS)
	
$(TARGET): $(OBJS) ../../linux/Lib/libZestSC1.a
	$(LD) $(LDFLAGS) -o $(TARGET) $(OBJS) $(LIBS)

${BITFILE}:	FPGA-VHDL/Example2.vhd
	cd FPGA-VHDL/ && xst -intstyle ise -ifn "/home/ttsiod/Xilinx/ZestSC1/Examples/FPGA_to_UART/FPGA-VHDL/Example2.xst" -ofn "/home/ttsiod/Xilinx/ZestSC1/Examples/FPGA_to_UART/FPGA-VHDL/Example2.syr" || exit 1
	cd FPGA-VHDL/ && ngdbuild -intstyle ise -dd _ngo -sd ipcore_dir -aul -nt timestamp -uc /home/ttsiod/Xilinx/ZestSC1/UCF/ZestSC1.ucf -p xc3s1000-ft256-5 Example2.ngc Example2.ngd  || exit 1
	cd FPGA-VHDL/ && map -intstyle ise -p xc3s1000-ft256-5 -cm area -ir off -pr b -c 100 -o Example2_map.ncd Example2.ngd Example2.pcf || exit 1
	cd FPGA-VHDL/ && par -w -intstyle ise -ol high -t 1 Example2_map.ncd Example2.ncd Example2.pcf || exit 1
	cd FPGA-VHDL/ && trce -intstyle ise -e 3 -s 5 -n 3 -xml Example2.twx Example2.ncd -o Example2.twr Example2.pcf || exit 1
	cd FPGA-VHDL/ && bitgen -intstyle ise -f Example2.ut Example2.ncd || exit 1

console:	| ${BITFILE}
	rm -f ${OBJS} ${TARGET} 2>/dev/null
	$(MAKE) ${TARGET}
	sudo ./${TARGET} 1000000

.PHONY:	clean

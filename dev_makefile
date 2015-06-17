
srcdir = .
LDFLAGS =  -Wl,-elf2flt="-r -z -s 32768"
LIBS =  
CFLAGS = -O2 -fomit-frame-pointer -fno-builtin -fpic -msingle-pic-base


# Change these if necessary

CC = arm-elf-g++ 
CPP = arm-elf-g++ -E

FILES = MoxaGateway
 
all:	$(FILES)
	chmod a+x $(FILES)

MoxaGateway:	Utility.cpp  ModbusController.cpp ServerController.cpp LodamModbusProject.cpp
	$(CC) -o $@ $^ $(CFLAGS) $(LDFLAGS) $(LIBS) -D_STLP_UNIX -lpthread 

clean:
	rm -f *.o MoxaGateway *.gdb $(FILES)  




MODULE=run.x

WORK=.
SYSTEMC_LIB=$(firstword $(wildcard $(SYSTEMC)/lib*))

.PHONY: all clean 

all:
	@echo "+++++++++++++++++++++++++++++++++"
	@echo "Compiling a SystemC example"
	g++ -O0 -g -ggdb -Wno-deprecated\
            -DDEBUG_SYSTEMC\
            -I$(WORK)/ -I$(SYSTEMC)/include -I/include\
            -c $(WORK)/detector.cpp
	g++ -O0 -g -ggdb -Wno-deprecated\
            -DDEBUG_SYSTEMC\
            -I$(WORK)/ -I$(SYSTEMC)/include -I/include\
            -c $(WORK)/sensor_sim.cpp
	g++ -O0 -g -ggdb -Wno-deprecated\
            -DDEBUG_SYSTEMC\
            -I$(WORK)/ -I$(SYSTEMC)/include -I/include\
            -c $(WORK)/display.cpp
	g++ -O0 -g -ggdb -Wno-deprecated\
            -DDEBUG_SYSTEMC\
            -I$(WORK)/ -I$(SYSTEMC)/include -I/include\
            -c $(WORK)/main.cpp
	g++ -O0 -g -ggdb -Wno-deprecated\
            -DDEBUG_SYSTEMC\
            -I$(WORK)/ -I$(SYSTEMC)/include -I/include\
            -L$(WORK)/ -L$(SYSTEMC_LIB)\
            -o $(WORK)/$(MODULE) \
            $(WORK)/detector.o $(WORK)/display.o $(WORK)/main.o $(WORK)/sensor_sim.o\
            -lsystemc -lm   2>&1 | c++filt
	@echo "+++ Executing +++++++++++++++++++"
	$(WORK)/$(MODULE)

clean:
	rm -fr *.o *.x *.deps dependencies.mk *~


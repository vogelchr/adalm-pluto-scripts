OBJECTS = pluto_fdx_test.o

LIBS=-L/opt/pluto/lib -L/opt/pluto/lib64 -liio -lad9361 -lm
CPPFLAGS=-I/opt/pluto/include
CFLAGS=-Os -ggdb -g3 -march=native

all : pluto_fdx_test

pluto_fdx_test : $(OBJECTS)
	$(CC) $(LDFLAGS) -o $@ $^ $(LIBS)

ifneq ($(MAKECMDGOALS),clean)
include $(OBJECTS:.o=.d)
endif

%.d : %.c
	$(CC) $(CPPFLAGS) -o $@ -MM $^

.PHONY : clean
clean :
	rm -f *~ *.d *.o *.bak pluto_fdx_test

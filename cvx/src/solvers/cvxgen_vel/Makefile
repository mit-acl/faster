INPUT=vel
OPT = -Wall -O3  
# The option -03 will compile with all the optimizations available for max performance
# libmath is needed for sqrt, which is used only for reporting the gap. Can
# remove if desired for production solvers..
LDLIBS = -lm
CFLAGS = $(OPT) $(INCLUDES) -fPIC
LDFLAGS= -shared
CC = gcc
.PHONY: all
all: cvxgen_lib
cvxgen_lib: solver.o matrix_support.o ldl.o cvxgen_lib.o util.o 
	gcc -shared -o libcvx.so cvxgen_lib.o solver.o util.o matrix_support.o ldl.o -fPIC -Wl,--version-script=exportmap.map
	 mv libcvx.so ../cvxgen/libcvx_$(INPUT).so && cp interface_$(INPUT).h ../cvxgen    


# Include util.o for random functions and easy matrix printing.
#cvxgen_lib: solver.o matrix_support.o ldl.o util.o cvxgen_lib.o
solver.o: solver.h
matrix_support.o: solver.h
ldl.o: solver.h
util.o: solver.h
cvxgen_lib.o: solver.h

.PHONY : clean
clean :
	rm -f *.o cvxgen_lib libcvx.so

# Produced by CVXGEN, 2018-03-30 10:48:03 -0400.
# CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com.
# The code in this file is Copyright (C) 2006-2017 Jacob Mattingley.
# CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial
# applications without prior written permission from Jacob Mattingley.

# Filename: Makefile.
# Description: Basic Makefile.
INPUT=accel
OPT = -Wall -Os 
# libmath is needed for sqrt, which is used only for reporting the gap. Can
# remove if desired for production solvers..
LDLIBS = -lm
CFLAGS = $(OPT) $(INCLUDES) -fPIC
LDFLAGS= -shared
CC = gcc
.PHONY: all
all: cvxgen_lib
cvxgen_lib: solver.o matrix_support.o ldl.o cvxgen_lib.o util.o  
	gcc -shared -o libcvx.so cvxgen_lib.o solver.o util.o matrix_support.o ldl.o
	 mv libcvx.so ../cvxgen/libcvx_$(INPUT).so && mv solver.h solver_$(INPUT).h && cp solver_$(INPUT).h ../cvxgen  && mv solver_$(INPUT).h solver.h  
#the command objcopy above is needed to prevent conflict with other cvxgen generated files (in case you are using several cvxgen projects at the same time), see https://stackoverflow.com/questions/6940384/how-to-deal-with-symbol-collisions-between-statically-linked-libraries/6940389#6940389
# objcopy --prefix-symbols=$(INPUT)_ libcvx.so

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



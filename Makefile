
IDIR =./inc
CC=gcc
CFLAGS=-I$(IDIR)

ODIR=obj
LDIR =../lib

LIBS=-lm

_DEPS = easyspin.h easyspin_target_config.h easyspin_test.h eMotionControl.h gpio_lib.h
DEPS = $(patsubst %,$(IDIR)/%,$(_DEPS))

_OBJ = easyspin_test.o easyspin.o eMotionControl.o gpio_lib.o
OBJ = $(patsubst %,$(ODIR)/%,$(_OBJ))

$(ODIR)/%.o: %.c $(DEPS)
		$(CC) -c -o $@ $< $(CFLAGS)

easyspin_test: $(OBJ)
		gcc -o $@ $^ $(CFLAGS) $(LIBS)

.PHONY: clean

clean:
		rm -f $(ODIR)/*.o *~ core $(INCDIR)/*~

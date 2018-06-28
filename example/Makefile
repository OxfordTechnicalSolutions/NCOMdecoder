#############################################################
#                                                           #
# Make file for the NComToCsv example using the NCOMdecoder #
#                                                           #
#############################################################

#Directories
IDIR       =../nav
EXAMPLE_DIR=.

#Compiler settings
CC    =gcc
CFLAGS=-I$(IDIR)

#NCOMdecoder dependencies
_DEPS = NComRxC.h
DEPS  = $(patsubst %,$(IDIR)/%,$(_DEPS))

#NCOMdecoder object files required
_OBJ = NComRxC.o
OBJ  = $(patsubst %,$(IDIR)/%,$(_OBJ))

#Example object files required
_EXAMPLE = NcomToCsv.o
EXAMPLE  = $(patsubst %,$(EXAMPLE_DIR)/%,$(_EXAMPLE))

#Rule for *library* object files
$(IDIR)/%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

#Rule for the executable
NComToCsv: $(OBJ) $(EXAMPLE)
	gcc -o $@ $^ $(CFLAGS)

.PHONY: clean

clean:
	rm $(IDIR)/*.o *~ core $(INCDIR)/*~

#############################################################
#                                                           #
# Make file for the NComToCsv example using the NCOMdecoder #
#                                                           #
#############################################################

#Directory variables
EXAMPLES_DIR=examples
LIBRARY_DIR =nav
OBJECT_DIR  =obj

#Compiler settings
CC          =gcc
CFLAGS			=-I$(LIBRARY_DIR)

#Library files
_LIBRARY = NComRxC.h
LIBRARY  = $(patsubst %,$(LIBRARY_DIR)/%,$(_LIBRARY))

#Object files
_OBJ = NComRxC.o NComToCsv.o
OBJ = $(patsubst %,$(OBJECT_DIR)/%,$(_OBJ))


NComToCsv: $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS)

.PHONY: clean

#clean:
#	rm $(OBJECT_DIR)/*.o *~ core $(INCDIR)/*~

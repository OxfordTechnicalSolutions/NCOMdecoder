#======================================================================#
#                                                                      #
#      Make file for the NComToCsv NComToCsv using the NCOMdecoder       #
#                                                           		   #
#======================================================================#

# List of all targets
.PHONY: clean 

# Directories
NAV_INC_DIR=nav
EXAMPLE_INC_DIR=example
WRAPPER_INC_DIR=wrapper

# Compiler settings
CC=g++
# includes debug 
CFLAGS=-g \
		-I$(NAV_INC_DIR) \
		-I$(EXAMPLE_INC_DIR) \
		-I$(WRAPPER_INC_DIR)

all: wrapper_test

nav_objects:
	@echo "Building NComRxC nav objects"
	@$(CC) -c $(NAV_INC_DIR)/NComRxC.c -o $(NAV_INC_DIR)/NComRxC.o

example_objects: nav_objects
	@echo "Building example objects"
	@$(CC) -c $(CFLAGS) $(EXAMPLE_INC_DIR)/NComToCsv.c -o $(EXAMPLE_INC_DIR)/NComToCsv.o

example: nav_objects example_objects
	@echo "Building example NComToCSV executable"
	@mkdir -p $(EXAMPLE_INC_DIR)/bin
	@$(CC) $(CFLAGS) $(NAV_INC_DIR)/NComRxC.o \
					$(EXAMPLE_INC_DIR)/NComToCsv.o -o \
					$(EXAMPLE_INC_DIR)/bin/NComToCsv

wrapper_objects: nav_objects
	@echo "Building wrapper objects"
	@$(CC) -c $(CFLAGS) $(WRAPPER_INC_DIR)/NComRxCWrapper.cpp \
						-o $(WRAPPER_INC_DIR)/NComRxCWrapper.o

wrapper_test_objects: wrapper_objects
	@echo "Building the wrapper test objects"
	@$(CC) -c $(CFLAGS) $(WRAPPER_INC_DIR)/NComRxCWrapperTest.cpp \
						-o $(WRAPPER_INC_DIR)/NComRxCWrapperTest.o

wrapper_test: wrapper_test_objects wrapper_objects
	@echo "Building the wrapper test executable"
	@mkdir -p $(WRAPPER_INC_DIR)/bin
	@$(CC) $(CFLAGS) $(WRAPPER_INC_DIR)/NComRxCWrapperTest.o \
					$(WRAPPER_INC_DIR)/NComRxCWrapper.o \
					$(NAV_INC_DIR)/NComRxC.o \
					-o $(WRAPPER_INC_DIR)/bin/NComRxCWrapperTest


clean:
	@echo "Cleaning up..."
	@echo $(EXAMPLE_DIR)
	@rm \
	$(NAV_INC_DIR)/*.o \
	$(EXAMPLE_INC_DIR)/*.o \
	$(WRAPPER_INC_DIR)/*.o \
	$(EXAMPLE_INC_DIR)/*.exe \
	$(EXAMPLE_INC_DIR)/*.o \


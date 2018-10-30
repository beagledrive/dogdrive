#
# Sample makefile to compile template files

TARGET = tempexec
# source directory
SRC_DIR = ./src
# include directory
INC_DIR = ./header
# build directory
OBJ_DIR = ./obj

# -g debug information for the executable
# -Wall compiler warnings
CC = gcc
CFLAGS = -g -Wall -I$(INC_DIR)

# dependencies
_DEPS = template.h
DEPS = $(patsubst %,$(INC_DIR)/%,$(_DEPS))
_OBJS = template.o tempfunc.o
OBJS = $(patsubst %,$(OBJ_DIR)/%,$(_OBJS))

# build objects
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c  $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)
# build target
$(TARGET): $(OBJS)
	$(CC) -o $@ $^ $(CFLAGS)

.PHONY: clean

clean:
	rm -f $(OBJ_DIR)/*.o *~ core $(INC_DIR)/*~ $(SRC_DIR)/*~

#
# Sample makefile to compile template files

TARGET = build
# source directory
SRC_DIR = ./src
# include directory
INCLUDES = ./header

# -g debug information for the executable
# -Wall compiler warnings
CC = gcc
CFLAGS = -g -Wall -I$(INCLUDES) -lm -Werror
ifeq ($(CFG),debug)
CFLAGS += -D_DEBUG
else
CFLAGS += -O2
endif


# dependencies
SRCS = $(wildcard $(SRC_DIR)/*.c)
OBJS = $(SRCS:.c=.o)

# build objects
#$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c  $(DEPS)
#	$(CC) -c -o $@ $< $(CFLAGS)
# build target
#$(TARGET): $(OBJS)
#	$(CC) -o $@ $^ $(CFLAGS)

$(SRC_DIR)/%.o: $(SRC_DIR)/%.c
	$(CC) -c -o $@ $< $(CFLAGS)	

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^

.PHONY: clean

clean:
	rm -f $(OBJS) *~ core $(INCLUDES)/*~ $(SRC_DIR)/*~ $(TARGET)

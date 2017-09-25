
TARGET			:= battery-bq27x00
OBJS			:= main.o bq27x00.o

CC              := gcc
CXX             := g++
CFLAGS			:= -Wall -pipe

.PHONY:	clean

all:	$(TARGET)

clean:
	@rm -rf $(TARGET) $(OBJS)

$(TARGET):	$(OBJS)
	$(CC) -o $@ $^ $(LFALGS)


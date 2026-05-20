CC = gcc
CFLAGS = -Wall -g
LDFLAGS = 
LIBS = -lphidget22 -lzstd

TARGET = opcdecode
SRC = opcdecode.c

all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) $(SRC) -o $(TARGET) $(LDFLAGS) $(LIBS)

clean:
	rm -f $(TARGET)

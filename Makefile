CC     = riscv64-linux-gnu-gcc
CFLAGS = -Wall -O3 -g -I../ethercat/include/
LIBS   = -L../ethercat/etherlab/lib/ -lm -lethercat -lrt -lpthread

TARGET = solo_axis_igh
SRCS   = $(wildcard *.c)

OBJS   = $(SRCS:.c=.o)

$(TARGET):$(OBJS)
	$(CC) -L ../ethercat/lib/ -o $@ $^ $(LIBS)

clean:
	rm -rf $(TARGET) $(OBJS)

%.o:%.c
	$(CC) $(CFLAGS) -o $@ -c $<



LIB_NAME = libSUSI-3.02
LIB_LINK_NAME = $(LIB_NAME).so
LINKLIB = -Wl,-rpath,./ $(LIB_LINK_NAME)
CFLAGS = -Wall -O2

CC=gcc
all:jamplayer
jamplayer:*.c
#	$(CC) $^ -o $@
	$(CC) $^ -o $@ $(LINKLIB) 
#	$(CC) $(CFLAGS) $^ -o $(basename $< .c) $< $(LINKLIB) 

clean:
	rm -f jamplayer


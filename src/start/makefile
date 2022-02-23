CFLAGS += -W -Wall -std=c99
LIBS += -lm

masterprocess_executable: masterprocess.o
	$(CC) $(CFLAGS) masterprocess.o -o masterprocess $(LIBS)

masterprocess.o: masterprocess.c
	$(CC) $(CFLAGS) -c masterprocess.c

clean:
	rm *.o
	rm masterprocess_executable

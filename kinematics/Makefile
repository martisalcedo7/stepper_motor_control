CC = gcc
CFLAGS = -Wall

my_program: main.o kinematics.o
	$(CC) $(CFLAGS) -o my_program main.o kinematics.o -lm

main.o: main.c
	$(CC) $(CFLAGS) -c main.c

kinematics.o: kinematics.c
	$(CC) $(CFLAGS) -c kinematics.c

clean:
	rm -f *.o my_program

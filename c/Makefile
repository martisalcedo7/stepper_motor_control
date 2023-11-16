CC = gcc
CFLAGS = -Wall

my_program: test.o robot.o trajectories.o
	$(CC) $(CFLAGS) -o my_program test.o robot.o trajectories.o -lm

test.o: test.c
	$(CC) $(CFLAGS) -c test.c

robot.o: robot.c
	$(CC) $(CFLAGS) -c robot.c

trajectories.o: trajectories.c
	$(CC) $(CFLAGS) -c trajectories.c

clean:
	rm -f *.o my_program

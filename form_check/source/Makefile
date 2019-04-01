CC=g++
CXXFLAGS= -pedantic -Wall -Wextra
main: main.o application.o
	$(CC) -o form_check main.o application.o -L../deps -lKinect20
main.o: main.cpp application.o ../headers/application.h
	$(CC) $(CFLAGS) -c main.cpp

application.o: application.cpp ../headers/application.h ../headers/applicationconstants.h
	$(CC) $(CFLAGS) -c application.cpp

clean:
	rm -f *.o project

mur3 : mur3.c pilota3.c winsuport2.o winsuport2.h memoria.o memoria.h
	gcc -Wall mur3.c winsuport2.o memoria.o -o mur3 -lcurses -lpthread
	gcc -Wall pilota3.c winsuport2.o memoria.o -o pilota3 -lcurses

winsuport2.o : winsuport2.c winsuport2.h
	gcc -Wall -c winsuport2.c -o winsuport2.o

memoria.o : memoria.c memoria.h
	gcc -Wall -c memoria.c -o memoria.o

clean : 
	rm -f mur3 pilota3 winsuport2.o memoria.o


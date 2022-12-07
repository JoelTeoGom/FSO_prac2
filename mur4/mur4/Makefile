
mur4 : mur4.c pilota4.c winsuport2.o winsuport2.h memoria.o memoria.h semafor.o semafor.h missatge.o missatge.h
	gcc -Wall mur4.c winsuport2.o memoria.o semafor.o missatge.o -o mur4 -lcurses -lpthread
	gcc -Wall pilota4.c winsuport2.o memoria.o semafor.o missatge.o  -o pilota4 -lcurses  -lpthread

semafor.o : semafor.c semafor.h
	gcc -Wall -c semafor.c -o semafor.o

missatge.o : missatge.c missatge.h
	gcc -Wall -c missatge.c -o missatge.o

winsuport2.o : winsuport2.c winsuport2.h
	gcc -Wall -c winsuport2.c -o winsuport2.o

memoria.o : memoria.c memoria.h
	gcc -Wall -c memoria.c -o memoria.o

clean : 
	rm -f mur4 pilota4 winsuport2.o memoria.o missatge.o semafor.o 
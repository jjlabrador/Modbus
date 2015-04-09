#Fichero Makefile para compilar las distintas pruebas
# preparadas para verificar el correcto funcionamiento
# de la práctica
#
# basta con ejecutar en línea de comandos
#  make

pruebas:=$(wildcard prueba*.cpp)
ejecutables:=$(pruebas:.cpp=)

#all y clean son objetivos especiales
.PHONY : all clean zip
#Evitamos que se borren los fichero intermedios (%.o)
.SECONDARY :

all : $(ejecutables) CalculaCRC

CalculaCRC : CalculaCRC.cpp Makefile
	g++ -Wall -o $@ $<

#Reglas genéricas
prueba%.o : prueba%.cpp Makefile
	g++ -Wall -c -o $@ $<

%.o : %.cpp %.hpp Makefile
	g++ -Wall -c -o $@ $<

prueba% : prueba%.o ModbusServer.o
	g++ -Wall -g -o $@ $^ -lrt

#reglas para borrar los fichero generados
# y dejar sólo los fuente
clean :
	rm -f *.o $(ejecutables) CalculaCRC

#Generación de zip
.PHONY : zip

zip : FicherosPracticaModbus.zip

FicherosPracticaModbus.zip : Makefile prueba1.cpp CalculaCRC.cpp prueba2.cpp prueba3.cpp
	rm -f $@
	zip $@ $^

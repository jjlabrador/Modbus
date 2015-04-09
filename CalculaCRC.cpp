/**
 * Programa que calcula CRC de cadenas de bytes
 * pasados como parámetro de entrada.
 * Se consideran bytes en hexadecimal.
 *
 * \author Alberto Hamilton Castro
 *
 * Ejemplos de invocación:
 *    ./CalculaCRC 4 1 0 b 0 5
 *    ./CalculaCRC 04 01 00 0b 00 05
*/

#include <vector>
#include <iostream>
#include <iomanip>
#include <stdlib.h>


typedef unsigned char byte;

using namespace std;

/*! Calcula el código CRC-16 de los primeros 'len' bytes
 * \param mensaje vector que contiene los bytes de trabajo
 * \param len numero de bytes considerados
 * \return vector con los dos bytes del CRC en el orden "correcto"
*/
vector<byte>  crc16( vector<byte> mensaje, int len) {
#define POLY 0xA001
  int i;
  unsigned int crc=0xFFFF;


  for(int ba=0; ba<len; ba++) {
    crc ^= mensaje[ba];
    for(i=0; i<8; i++) {
      if( crc & 0x0001 ) {
        crc = (crc>>1) ^ POLY;
      } else {
        crc >>= 1;
      }
    }
  }
  vector<byte> codr;
  codr.push_back(crc & 0xff);
  codr.push_back(crc >> 8);
  return codr;
}

/*! Calcula el código CRC-16 de TODOS los bytes del vector
 * \param mensaje vector de bytes de trabajo
 * \return vector con los dos bytes del CRC en el orden "correcto"
 */
vector<byte>  crc16( vector<byte> mensaje) {
  return crc16( mensaje, mensaje.size() );
}


int main (int argc, char *argv[]) {

  //parseamos la linea de comandos
  int numBytes = argc - 1; //el primer argumento es el nombre del programa

  if( numBytes < 1) {
    cerr << "Necesario al menos un parámetro de entrada" <<endl;
    return 1;
  }

  vector<byte> bytes;
  char* p;
  for(int pa = 0; pa < numBytes; pa++) {
    unsigned long int val = strtoul( argv[pa+1], &p, 16);
    if( *p != 0 ) {
      cout << "Argunmento '" << argv[pa+1] << "' no parece número hexadecimal" <<endl;
      return 2;
    }
    if ( val > 255 ) {
      cout << "Argunmento '" << argv[pa+1] << "' no cabe en un byte" <<endl;
      return 3;
    }
    bytes.push_back( (byte)val );
  }

  //invocamos la función
  vector<byte> crc = crc16( bytes );

  cout << "El CRC para " << setfill('0') << hex ;
  for ( int ba = 0; ba < numBytes; ba++)
    cout << setw(2) << (int)bytes[ba] << ' ';
  cout << "es " << setw(2) << (int)crc[0]
    << ' ' << setw(2) << (int)crc[1]
    << dec <<endl;

  return 0;
}

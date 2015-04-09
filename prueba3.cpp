/**
 * Programa de prueba de la parte avanzada 2 de la práctica 3
 *
 * Pedimos todas las entradas analogicas y mostramos lo respuesta
 *
 * \author Alberto Hamilton Castro
 */

#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include "ModbusServer.hpp"

#define ID_DEV  6

//si vale 0 se haran todas las pruebas aunque fallen
//si vale 1 se parará desde que falle la primera
int SalSiFallo = 1;

#define BVEC(v) vector<byte>( v, v + sizeof(v)/sizeof(byte) )

typedef unsigned char byte;

using namespace std;


ostream& operator<<(ostream &os, const vector<byte> &v) {
  os << "[("  << v.size() << ") " << setfill('0') << hex ;
  for ( unsigned int ba = 0; ba < v.size(); ba++)
    os << setw(2) << (int)v[ba] << ' ';
  os << "] " << dec;
  return os;
}


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

  string NombreRegistros[20] = { "Año", "Mes", "Día", "Hora", "Minuto"
    , "segundos"
    , "UID" , "GID", "PID", "PPID"
    , "Segundos cómputo", "Milisegundos cómputo"
    , "Num. Peticiones", "Bytes Recibidos", "Bytes enviados"
    , "Reg 16", "Reg 17", "Reg 18", "Reg 19", "Reg 20"
  };

  int pruebas[] = { 0, 20, 0, 6, 11, 5, 11, 5, 11, 5, 0, 20 };

  ModbusServer mbs(ID_DEV);

  for( unsigned int pa=0; pa < (sizeof(pruebas)/sizeof(int)/2); pa++) {
    cout << "Prueba " << pa << " =====================================" <<endl;

    byte regIni = pruebas[ pa*2 ];
    byte numReg = pruebas[ pa*2+1 ];
    byte peti[] = { ID_DEV, 0x04, 0x00, regIni, 0x00, numReg};
    vector<byte> peticion = BVEC(peti);
    vector<byte> crc = crc16( peticion );
    peticion.push_back( crc[0] );
    peticion.push_back( crc[1] );

    vector<byte> respuesta = mbs.peticion( peticion );
    cout << "La petición enviada es " << peticion << endl;
    cout << "La respuesta recbida es " << respuesta << endl;

    unsigned int len = respuesta.size();
    crc = crc16( respuesta, len-2 );
    if ( (crc[0] == respuesta[len-2]) && (crc[1] == respuesta[len-1]) )
      cout << "CRC correcto" <<endl;
    else {
      cout << "CRC INCORRECTO" <<endl;
      return 1;
    }

    if ( respuesta[0] != ID_DEV ) {
      cerr << "Respuesta con ID incorrecto" <<endl;
      return 1;
    } else {
      cout << "ID correcto"  <<endl;
    }

    if ( respuesta[1] != 0x04 ) {
      cerr << "Respuesta con Función incorrecta" <<endl;
      return 1;
    } else {
      cout << "Respuesta con Función correcta" <<endl;
    }

    if( (respuesta[2]/2) != numReg ) {
      cerr << "Respuesta con número de registros INCORRECTOS: "
      <<  (int)respuesta[2] << " != " << numReg <<endl;
      return 1;
    } else {
      cout << "Respuesta con número de registros correctos" <<endl;
    }

    unsigned int numBytes = respuesta[2];
    if( numBytes != (numReg*2) )  {
      cerr << "Respuesta con numero de datos incorrecto" <<endl;
      return 1;
    } else {
      cout << "Respuesta con numero de datos correcto" <<endl;
    }

    cout << "Los datos enviados son :" << endl;
    for(unsigned int i = 0; i<numReg; i++) {
      int dato = ((int)respuesta[i*2+3]<<8) | respuesta[i*2+4];
      cout << NombreRegistros[i+regIni] << ": " << dato << endl;
    }
    sleep(1);
  }
  cout << "Terminadas todas las pruebas" << endl;
}

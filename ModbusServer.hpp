/**
 * \file ModbusServer.hpp
 * \class ModbusServer
 * \brief Clase en C++ que implementa un servidor Modbus-RTU
 * \author Juan José Labrador González
 * \subject Computación Ubicua, Sistemas Empotrados y Sistemas Industriales
 * \date 02-03-15
 */

#if !defined (_MODBUSSERVER_H)
  #define _MODBUSSERVER_H
  #include <iostream>
  #include <vector>
  #include <time.h>
  #include <string.h>
  #include <unistd.h>
  #include <stdlib.h>
  #include <bitset>
  #include <sstream>
  #include <iomanip>
  #include <math.h>
  #define TAM_IODIGITAL 20
  #define TAM_OANALOG 10
  #define TAM_IANALOG 20
  #define TAM_F123456 8
  #define TAM_F1516 9

  using namespace std;

  typedef unsigned char byte;
  typedef unsigned short tByte;

  class ModbusServer {

    public:
      byte _id;
      vector<byte> oDigital;
      vector<byte> iDigital;
      vector<tByte> oAnalog;
      vector<tByte> iAnalog;

    private:
      bool comprobarID(byte id);
      bool comprobarCRC(vector<byte> recibido);

    public:
      ModbusServer(byte id);
      ~ModbusServer();
      vector<unsigned char> peticion(vector<byte> recibido);
      bool par(int n);
      char* getTime();
      vector<unsigned char> crc16(vector<unsigned char> mensaje, int len);
      vector<unsigned char> crc16(vector<unsigned char> mensaje);
      void imprimirRespuesta(vector<unsigned char> respuesta);
      void imprimirRegistros(unsigned int opc, unsigned int tipo);
      vector<unsigned char> funcion_01(vector<byte> recibido);
      vector<unsigned char> funcion_05(vector<byte> recibido);
      vector<unsigned char> funcion_15(vector<byte> recibido);
      vector<unsigned char> funcion_03(vector<byte> recibido);
      vector<unsigned char> funcion_06(vector<byte> recibido);
      vector<unsigned char> funcion_16(vector<byte> recibido);
      vector<unsigned char> funcion_02(vector<byte> recibido);
      vector<unsigned char> funcion_04(vector<byte> recibido);
      vector<unsigned char> error(vector<byte> recibido, unsigned char tipo);
      bool comprobarTamanio(vector<byte> recibido);
      bool comprobarRango(unsigned int nPos, unsigned int off, unsigned int limite);
      bool comprobarValor(unsigned int v);
      bool comprobarNRegistros(unsigned int nPos, unsigned int nReg);
  };
#endif

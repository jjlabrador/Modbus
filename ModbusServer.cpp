/**
 * \file ModbusServer.cpp
 * \class ModbusServer
 * \brief Fichero de implementación del servidor Modbus-RTU
 * \author Juan José Labrador González
 * \subject Computación Ubicua, Sistemas Empotrados y Sistemas Industriales
 * \date 02-03-15
 */

#include "ModbusServer.hpp"

ModbusServer::ModbusServer(byte id) {
  _id = id;
  oDigital.resize(TAM_IODIGITAL + 1);
  iDigital.resize(TAM_IODIGITAL + 1);
  oAnalog.resize(TAM_OANALOG + 1);
  iAnalog.resize(TAM_IANALOG + 1);
  
  // 20 primeras salidas digitales
  for (int i = 0; i < 21; i++) {
    if (par(i))
      oDigital[i] = 0;
    else
      oDigital[i] = 1;
  }

  // 10 primeras salidas analogicas
  unsigned int contador = 0;
  for (int i = 1; i < 11; i++) {
    oAnalog[i] = contador * 4;
   contador++;
  }

  // 20 primeras entradas analogicas
  char *time = getTime();
  char *ptr = strtok(time, "-");
  contador = 1;

  while (ptr != NULL) {
    iAnalog[contador] = atoi(ptr);
    ptr = strtok (NULL, "-");
    contador++;
  }

  iAnalog[contador] = getuid();
  contador++;
  iAnalog[contador] = getgid();
  contador++;
  iAnalog[contador] = getpid();
  contador++;
  iAnalog[contador] = getppid();
  contador++;

  clock_t t = clock();
  float seg = ((float)t) / CLOCKS_PER_SEC;
  iAnalog[contador] = seg;                      // Segundos
  contador++;
  iAnalog[contador] = seg * 1000;               // Milisegundos
  contador++;

  iAnalog[contador] = 0;        // Peticiones recibidas
  contador++;
  iAnalog[contador] = 0;        // Bytes recibidos
  contador++;
  iAnalog[contador] = 0;        // Bytes enviados
  contador++;

  for (; contador < 21; contador++) {
    if (par(contador))
      iAnalog[contador] = 0x0000;
    else
      iAnalog[contador] = 0x1111;
  }

  // 20 primeras entradas digitales
  for (int i = 1; i < 21; i++) {
    if (i < 15) {
      if (par(iAnalog[i]))
        iDigital[i] = 1;
      else
        iDigital[i] = 0;
    }
    else
      iDigital[i] = 0;
  }
}

ModbusServer::~ModbusServer() {
  oDigital.clear();
  iDigital.clear();
  oAnalog.clear();
  iAnalog.clear();
}

vector<unsigned char> ModbusServer::peticion(vector<byte> recibido) {
  vector<unsigned char> respuesta;

  if (comprobarID(recibido[0])) {
    if (comprobarCRC(recibido)) {
      switch(recibido[1]) {
        case 01: {
          respuesta = funcion_01(recibido);
          break;
        }
        case 05: {
          respuesta = funcion_05(recibido);
          break;
        }
        case 15: {
          respuesta = funcion_15(recibido);
          break;
        }
        case 03: {
          respuesta = funcion_03(recibido);
          break;
        }
        case 06: {
          respuesta = funcion_06(recibido);
          break;
        }
        case 16: {
          respuesta = funcion_16(recibido);
          break;
        }
        case 02: {
          respuesta = funcion_02(recibido);
          break;
        }
        case 04: {
          respuesta = funcion_04(recibido);
          break;
        }
        default:
          respuesta = error(recibido, 0x01);
      }
    }
  }

  if (respuesta.size() != 0) {
    vector<unsigned char> crc = crc16(respuesta, respuesta.size() - 2);
    respuesta[respuesta.size() - 2] = crc[0];
    respuesta[respuesta.size() - 1] = crc[1];
  }

  iAnalog[13]++;                        // Peticiones recibidas
  iAnalog[14] += recibido.size();       // Bytes recibidos
  iAnalog[15] += respuesta.size();      // Bytes enviados
  
  return (respuesta);
}

bool ModbusServer::comprobarID(byte id) {
  return (this -> _id == id) ? true : false;
}

bool ModbusServer::comprobarCRC(vector<byte> recibido) {
  vector<byte> crc_recibido(2);
  unsigned int tam = recibido.size();

  crc_recibido[0] = recibido[tam - 2];
  crc_recibido[1] = recibido[tam - 1];

  vector<unsigned char> crc_nuevo = crc16(recibido, tam - 2);

  if ((crc_nuevo[0] == crc_recibido[0]) && (crc_nuevo[1] == crc_recibido[1]))
    return true;
  else
    return false;
}

bool ModbusServer::par(int n) {
  return (n % 2 == 0) ? true : false;
}

char* ModbusServer::getTime() {
  time_t rawtime;
  struct tm *timeinfo;
  char *buffer = new char[80];

  time (&rawtime);
  timeinfo = localtime (&rawtime);

  strftime (buffer, 80, "%Y-%m-%d-%H-%M-%S", timeinfo);

  return buffer;
}

vector<unsigned char>  ModbusServer::crc16( vector<unsigned char> mensaje, int len) {
  #define POLY 0xA001
  int i;
  unsigned int crc = 0xFFFF;


  for (int ba = 0; ba < len; ba++) {
    crc ^= mensaje[ba];
    for (i = 0; i < 8; i++) {
      if (crc & 0x0001) {
        crc = (crc >> 1) ^ POLY;
      }
      else {
        crc >>= 1;
      }
    }
  }

  vector<unsigned char> codr;
  codr.push_back(crc & 0xff);
  codr.push_back(crc >> 8);

  return (codr);
}

/*! Calcula el código CRC-16 de TODOS los bytes del vector
 * \param mensaje vector de bytes de trabajo
 * \return vector con los dos bytes del CRC en el orden "correcto"
 */
vector<unsigned char>  ModbusServer::crc16( vector<unsigned char> mensaje) {
  return crc16(mensaje, mensaje.size());
}

void ModbusServer::imprimirRespuesta(vector<unsigned char> respuesta) {
  cout << showbase << internal << setfill('0');
  for (unsigned int i = 0; i < respuesta.size(); i++)
    cout << hex << setw(4) << respuesta[i] << " ";
  cout << endl;
}

void ModbusServer::imprimirRegistros(unsigned int opc, unsigned int tipo) {
  unsigned int ind = 1;
  unsigned int desp = 0;

  if (opc == 0) {               // Registros digitales
    if (tipo == 1) {            // Entradas
      ind = 10001;
      desp = 10000;
    }
    for (unsigned int i = ind; i < desp + 21; i++)
      cout << hex << setw(4) << iDigital[i] << " ";
  }
  else {                        // Registros analógicos
    unsigned int limite = 21;
    if (tipo == 0) {            // Salidas
      ind = 10001;
      desp = 10000;
      limite = 11;
    }
    for (unsigned int i = ind; i < desp + limite; i++)
      cout << hex << setw(4) << iAnalog[i] << " ";
  }
  cout << endl;
}

vector<unsigned char> ModbusServer::funcion_01(vector<byte> recibido) {

  vector<unsigned char> respuesta;
  unsigned int nPosiciones = (recibido[4] << 8) | recibido[5];
  unsigned int offset = (recibido[2] << 8) | recibido[3];

  if (comprobarRango(nPosiciones, offset, 20)) {
    vector<byte> datos;

    for (int i = nPosiciones; i > 0; i--)
      datos.push_back(this -> oDigital[offset + i]); // Little Endian

    stringstream numeroEntero;
    for (unsigned int i = 0; i < datos.size(); i++)
      numeroEntero << (int)datos[i];

    string stringEntero = numeroEntero.str();

    int aux = nPosiciones;
    unsigned char hexa;
    vector<unsigned char> vv;

    if (nPosiciones > 8) {
      while ((aux / 8) != 0) {
        bitset<8> numeroBinario(stringEntero.substr(aux - 8, aux - 1));
        hexa = numeroBinario.to_ulong();
        vv.push_back(hexa);
        aux -= 8;
      }

      bitset<8> numeroBinario(stringEntero.substr(0, aux));
      hexa = numeroBinario.to_ulong();
      vv.push_back(hexa);
    }
    else {
      bitset<8> numeroBinario(stringEntero);
      hexa = numeroBinario.to_ulong();
      vv.push_back(hexa);
    }

    unsigned int nBytes = ceil((float)nPosiciones / 8);
    if (nBytes == 0)
      nBytes++;

    unsigned int tam = 1 + 1 + 1 + nBytes + 2;
    //                 d + f + bytes + nBytes + + crc
    respuesta.resize(tam);
    respuesta[0] = recibido[0];
    respuesta[1] = recibido[1];
    respuesta[2] = nBytes;

    int cont = 3;
    for (unsigned int i = 0; i < vv.size(); i++) {
      respuesta[cont] = vv[i];
      cont++;
    }
  }
  else
    respuesta = error(recibido, 0x02);

  //imprimirRegistros(0, 0);
  return (respuesta);
}

vector<unsigned char> ModbusServer::funcion_05(vector<byte> recibido) {

  vector<unsigned char> respuesta;
  unsigned int offset = (recibido[2] << 8) | recibido[3];
  unsigned int valor = (recibido[4] << 8) | recibido[5];

  if (comprobarValor(valor)) {
    if (comprobarRango(0, offset, 20)) {
      if (valor == 0xff00)
        valor = 1;
      oDigital[offset + 1] = valor;

      respuesta.resize(recibido.size());

      respuesta[0] = recibido[0];
      respuesta[1] = recibido[1];
      respuesta[2] = recibido[2];
      respuesta[3] = recibido[3];
      respuesta[4] = recibido[4];
      respuesta[5] = recibido[5];
    }
    else
      respuesta = error(recibido, 0x02);
  }
  else
    respuesta = error(recibido, 0x03);

  //imprimirRegistros(0, 0);
  return (respuesta);
}

vector<unsigned char> ModbusServer::funcion_15(vector<byte> recibido) {

  vector<unsigned char> respuesta;
  unsigned int nPosiciones = (recibido[4] << 8) | recibido[5];
  unsigned int offset = (recibido[2] << 8) | recibido[3];

  if (comprobarRango(nPosiciones, offset, 20)) {
    int nBytes = recibido[6];
    int cont = 7;
    typedef bitset<8> bi;

    vector<bi> valores;

    for (int i = 0; i < nBytes; i++) {
      bitset<8> binario(recibido[cont]);
      valores.push_back(binario);
      cont++;
    }

    int aux = 0;
    int cont2 = 0;

    if (nPosiciones > 8) {
      for (unsigned int i = offset; i < nPosiciones + offset; i++) {
        oDigital[i + 1] = valores[cont2][aux];
        aux++;
        if (aux == 8) {
          cont2++;
          aux = 0;
        }
      }
    }
    else {
      int aux2 = 0;
      for (unsigned int i = offset; i < nPosiciones + offset; i++) {
        oDigital[i + 1] = valores[0][aux2];
        aux2++;
      }
    }

    respuesta.resize(8);

    respuesta[0] = recibido[0];
    respuesta[1] = recibido[1];
    respuesta[2] = recibido[2];
    respuesta[3] = recibido[3];
    respuesta[4] = recibido[4];
    respuesta[5] = recibido[5];
  }
  else
    respuesta = error(recibido, 0x02);

  ////imprimirRegistros(0, 0);
  return (respuesta);
}

bool ModbusServer::comprobarTamanio(vector<byte> recibido) {
  if ((recibido[1] == 15) || (recibido[1] == 16))
    return ((recibido.size() > TAM_F1516) ? true : false);
  else
    return ((recibido.size() == TAM_F123456) ? true : false);
}

bool ModbusServer::comprobarRango(unsigned int nPos, unsigned int off, unsigned int limite) {
  return ((nPos + off) <= limite) ? true : false;
}

bool ModbusServer::comprobarValor(unsigned int v) {
  return ((v == 0x0000) || (v == 0xff00)) ? true : false;
}

bool ModbusServer::comprobarNRegistros(unsigned int nPos, unsigned int nReg) {
  if ((0x0001 <= nPos) && (nPos <= 0x007B))
    if (nPos * 2 == nReg)
      return true;
  return false;
}

vector<unsigned char> ModbusServer::funcion_03(vector<byte> recibido) {

  vector<unsigned char> respuesta;
  unsigned int nPosiciones = (recibido[4] << 8) | recibido[5];
  unsigned int offset = (recibido[2] << 8) | recibido[3];

  if (comprobarRango(nPosiciones, offset, 10)) {
    unsigned int nBytes = nPosiciones * 2;
    unsigned int tam = 1 + 1 + 1 + nBytes + 2;
                    //dir+f+bytes+nBytes + crc
    respuesta.resize(tam);

    respuesta[0] = recibido[0];
    respuesta[1] = recibido[1];
    respuesta[2] = nBytes;
    int cont = 3;

    for (unsigned int i = offset + 1; i < offset + nPosiciones + 1; i++) {
      respuesta[cont] = (oAnalog[i] >> 8) & 0xff;
      cont++;
      respuesta[cont] = (oAnalog[i] >> 0) & 0xff;
      cont++;
    }
  }
  else
    respuesta = error(recibido, 0x02);

  //imprimirRegistros(1, 0);
  return (respuesta);
}

vector<unsigned char> ModbusServer::funcion_06(vector<byte> recibido) {

  vector<unsigned char> respuesta;
  unsigned int offset = recibido[2] + recibido[3];

  if (comprobarRango(0, offset, 10)) {
    unsigned int valor = (recibido[4] << 8) | recibido[5];
    respuesta.resize(recibido.size());

    respuesta[0] = recibido[0];
    respuesta[1] = recibido[1];
    respuesta[2] = recibido[2];
    respuesta[3] = recibido[3];
    oAnalog[offset + 1] = valor;
    respuesta[4] = recibido[4];
    respuesta[5] = recibido[5];
  }
  else
    respuesta = error(recibido, 0x02);

  //imprimirRegistros(1, 0);
  return (respuesta);
}

vector<unsigned char> ModbusServer::funcion_16(vector<byte> recibido) {

  vector<unsigned char> respuesta;
  unsigned int nPosiciones = (recibido[4] << 8) | recibido[5];
  unsigned int offset = (recibido[2] << 8) | recibido[3];

  if (comprobarNRegistros(nPosiciones, recibido[6])) {
    if (comprobarRango(nPosiciones, offset, 10)) {
      respuesta.resize(8);

      respuesta[0] = recibido[0];
      respuesta[1] = recibido[1];
      respuesta[2] = recibido[2];
      respuesta[3] = recibido[3];
      respuesta[4] = recibido[4];
      respuesta[5] = recibido[5];
      int cont = 7;

      for (unsigned int i = offset; i < offset + nPosiciones; i++) {
        oAnalog[i + 1] = (recibido[cont] << 8) | recibido[cont + 1];
        cont += 2;
      }
    }
    else
      respuesta = error(recibido, 0x02);
  }
  else
    respuesta = error(recibido, 0x03);

  //imprimirRegistros(1, 0);
  return (respuesta);
}

vector<unsigned char> ModbusServer::funcion_02(vector<byte> recibido) {

  unsigned int nPosiciones = (recibido[4] << 8) | recibido[5];
  unsigned int offset = (recibido[2] << 8) | recibido[3];

  //if (comprobarRango(nPosiciones, offset, 20)) {
    vector<byte> datos;

    for (int i = nPosiciones; i > 0; i--)
      datos.push_back(this -> iDigital[offset + i]); // Little Endian

    stringstream numeroEntero;
    for (unsigned int i = 0; i < datos.size(); i++)
      numeroEntero << (int)datos[i];

    string stringEntero = numeroEntero.str();

    int aux = nPosiciones;
    unsigned char hexa;
    vector<unsigned char> vv;
    vv.resize(0);

    if (nPosiciones > 8) {
      while ((aux / 8) != 0) {
        bitset<8> numeroBinario(stringEntero.substr(aux - 8, aux - 1));
        hexa = numeroBinario.to_ulong();
        vv.push_back(hexa);
        aux -= 8;
      }

      bitset<8> numeroBinario(stringEntero.substr(0, aux));
      hexa = numeroBinario.to_ulong();
      vv.push_back(hexa);
    }
    else {
      bitset<8> numeroBinario(stringEntero);
      hexa = numeroBinario.to_ulong();
      vv.push_back(hexa);
    }

    unsigned int nBytes = ceil((float)nPosiciones / 8);
    if (nBytes == 0)
      nBytes++;

    unsigned int tam = 1 + 1 + 1 + nBytes + 2;
    //                 d + f + bytes + nBytes + + crc
    vector<unsigned char> respuesta(tam);
    respuesta[0] = recibido[0];
    respuesta[1] = recibido[1];
    respuesta[2] = nBytes;

    int cont = 3;
    for (unsigned int i = 0; i < vv.size(); i++) {
      respuesta[cont] = vv[i];
      cont++;
    }
  //}
  //else {
  //  respuesta = error(recibido, 0x02);
  //}

  //imprimirRegistros(0, 1);
  return (respuesta);
}

vector<unsigned char> ModbusServer::funcion_04(vector<byte> recibido) {

  vector<unsigned char> respuesta;
  unsigned int nPosiciones = (recibido[4] << 8) | recibido[5];
  unsigned int offset = (recibido[2] << 8) | recibido[3];
  unsigned int nBytes = nPosiciones * 2;
  unsigned int tam = 1 + 1 + 1 + nBytes + 2;

  if (comprobarNRegistros(nPosiciones, nPosiciones * 2)) {
    if (comprobarRango(nPosiciones, offset, 20)) {
      respuesta.resize(tam);

      respuesta[0] = recibido[0];
      respuesta[1] = recibido[1];
      respuesta[2] = nBytes;
      int cont = 3;

      for (unsigned int i = offset + 1; i < offset + nPosiciones + 1; i++) {
        respuesta[cont] = (iAnalog[i] >> 8) & 0xff;
        cont++;
        respuesta[cont] = (iAnalog[i] >> 0) & 0xff;
        cont++;
      }
    }
    else
      respuesta = error(recibido, 0x02);
  }
  else
    respuesta = error(recibido, 0x03);

  //imprimirRespuesta(respuesta);
  return (respuesta);
}

vector<unsigned char> ModbusServer::error(vector<byte> recibido, unsigned char tipo) {

  vector<unsigned char> respuesta(5);

  respuesta[0] = recibido[0];
  respuesta[1] = recibido[1] | 0x80;
  respuesta[2] = tipo;

  //imprimirRegistros(1, 1);
  return (respuesta);
}
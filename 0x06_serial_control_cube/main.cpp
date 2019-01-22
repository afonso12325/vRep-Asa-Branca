/*
    Client of V-REP simulation server (remoteApi)
    Copyright (C) 2015  Rafael Alceste Berri rafaelberri@gmail.com

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// Habilite o server antes na simulação V-REP com o comando lua:
// simExtRemoteApiStart(portNumber) -- inicia servidor remoteAPI do V-REP

extern "C" {
  #include "remoteApi/extApi.h"
}
#include "serial.h"
#include <iostream>
#include <stdio.h> 
#include <string> 
#include <errno.h> 
#include <fcntl.h> 
#include <stdlib.h> 
#include <string.h>
#include <termios.h> 
#include <unistd.h>
using namespace std;

int main(int argc, char ** argv) {
  char * portname = "/dev/ttyACM0";
  int fd;
  int wlen;
  string serverIP = "127.0.0.1";
  int serverPort = 19999;
  int motorHandle = 0;
  int tree_handle = 0;
  int corpo_handle;
  float euler[3];
  string angle;
  float vel;
  // inicialização serial
  fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0) {
    printf("Error opening %s: %s\n", portname, strerror(errno));
    return -1;
  }
  /*baudrate 9600, 8 bits, no parity, 1 stop bit */
  set_interface_attribs(fd, B9600);
  //set_mincount(fd, 0);                /* set to pure timed read */

  /* simple noncanonical input */
  // variaveis de cena e movimentação do pioneer
  char buf[50];
  int clientID = simxStart((simxChar * ) serverIP.c_str(), serverPort, true, true, 2000, 5);
  bool ft = true;
  int rdlen = 0;
  unsigned char bufserial[80];
  unsigned char v[4];
  if (clientID != -1) {
    cout << "Servidor conectado!" << std::endl;
    // inicialização dos motores
    if (simxGetObjectHandle(clientID, (const simxChar * )
        "Revolute_joint", (simxInt * ) & motorHandle, (simxInt) simx_opmode_oneshot_wait) != simx_return_ok)
      cout << "Handle do motor esquerdo nao encontrado!" << std::endl;
    else
      cout << "Conectado ao motor " << std::endl;

    if (simxGetObjectHandle(clientID, (const simxChar * )
        "corpo", (simxInt * ) & corpo_handle, (simxInt) simx_opmode_oneshot_wait) != simx_return_ok)
      cout << "Handle do corpo nao encontrado!" << std::endl;
    else
      cout << "Conectado ao corpo " << std::endl;
    
    if (simxGetObjectHandle(clientID, (const simxChar * )
        "Tree", (simxInt * ) & tree_handle, (simxInt) simx_opmode_oneshot_wait) != simx_return_ok)
      cout << "Handle do tree nao encontrado!" << std::endl;
    else
      cout << "Conectado ao tree " << std::endl;

  

    // desvio e velocidade do robô
    while (simxGetConnectionId(clientID) != -1) // enquanto a simulação estiver ativa
    {
      // atualiza velocidades dos motores
      simxGetObjectOrientation(clientID, corpo_handle, tree_handle, euler, (simxInt) simx_opmode_oneshot_wait);
      // euler[2] = 11.7;
      // unsigned char e[sizeof euler[2]];
      // memcpy(e,&(euler[2]), sizeof euler[2]);
      // for(int ind = 0; ind < sizeof euler[2]; ind++){
      //   printf("%x\n", e[ind]);
      // } 
      // printf("----------\n");
      angle = to_string(euler[2]);
      if(angle[0] == '-'){
        if(angle.length() > 6){
          for(int j = 0; j<angle.length() - 6;j++) angle.pop_back();
        }
          
      }
      else{
        if(angle.length() > 5){
          for(int j = 0; j<angle.length() - 5;j++) angle.pop_back();
        }
      }

      //std::cout << angle << endl;
      // vS = vRightS + "," + vLeftS+"e";
      cout<< "sending: "<<angle<<endl;
      write(fd,angle.c_str(),angle.length());
      // canwrite = true;

      // cout<<vS<<endl;
      // printf("lê\n");
      rdlen = read(fd, bufserial, 4);
      // printf("leu!!!!!!!!!!\n");

      if (rdlen > 0) {
        unsigned char * p;
        int ind = 0;
        for (p = bufserial; rdlen-- > 0; p++){
          v[ind] = *p;
          ind++;
        }
        memcpy(&vel,v,4);
        printf("received :  %f\n", vel);
        // cout<<vel<<endl;
        // cout << "oi"<<endl;
      
      
      }
   
        
      

      simxSetJointTargetVelocity(clientID, motorHandle, (simxFloat) vel, simx_opmode_streaming);
  
      

    }
  
    simxFinish(clientID); // fechando conexao com o servidor
    cout << "Conexao fechada!" << std::endl;
  }
   else
    cout << "Problemas para conectar o servidor!" << std::endl;
  return 0;
}
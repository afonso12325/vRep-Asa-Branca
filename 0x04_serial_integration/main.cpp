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

int main(int argc, char **argv) 
{
  char *portname = "/dev/ttyACM0";
  int fd;
  int wlen;
  string serverIP = "127.0.0.1";
  int serverPort = 19999;
  int leftMotorHandle = 0;
  float vLeft = 0;
  int rightMotorHandle = 0;
  float vRight = 0;
  string sensorNome[16];
  int sensorHandle[16];
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
  float noDetectionDist=0.5;
  float maxDetectionDist=0.2;
  float detect[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  float braitenbergL[16]={-0.2,-0.4,-0.6,-0.8,-1,-1.2,-1.4,-1.6, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
  float braitenbergR[16]={-1.6,-1.4,-1.2,-1,-0.8,-0.6,-0.4,-0.2, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
  float v0=2;
  char buf[50];
  int clientID=simxStart((simxChar*)serverIP.c_str(),serverPort,true,true,2000,5);
  
  if (clientID!=-1)
  {
    cout << "Servidor conectado!" << std::endl;
    
    // inicialização dos motores
    if(simxGetObjectHandle(clientID,(const simxChar*) "Pioneer_p3dx_leftMotor",(simxInt *) &leftMotorHandle, (simxInt) simx_opmode_oneshot_wait) != simx_return_ok)
      cout << "Handle do motor esquerdo nao encontrado!" << std::endl;  
    else
      cout << "Conectado ao motor esquerdo!" << std::endl;
    
    if(simxGetObjectHandle(clientID,(const simxChar*) "Pioneer_p3dx_rightMotor",(simxInt *) &rightMotorHandle, (simxInt) simx_opmode_oneshot_wait) != simx_return_ok)
      cout << "Handle do motor direito nao encontrado!" << std::endl;  
    else
      cout << "Conectado ao motor direito!" << std::endl;
    
    // inicialização dos sensores (remoteApi)
    for(int i = 0; i < 16; i++)
    {
      sprintf(buf, "Pioneer_p3dx_ultrasonicSensor%d",i+1);
      
      if(simxGetObjectHandle(clientID,(const simxChar*) buf,(simxInt *) &sensorHandle[i], (simxInt) simx_opmode_oneshot_wait) != simx_return_ok)
	cout << "Handle do sensor " << sensorNome[i] << " nao encontrado!" << std::endl;
      else
      {
        cout << "Conectado ao sensor " << sensorNome[i] << std::endl;
	simxReadProximitySensor(clientID,sensorHandle[i],NULL,NULL,NULL,NULL,simx_opmode_streaming);
      }
    }
    
    // desvio e velocidade do robô
    while(simxGetConnectionId(clientID)!=-1) // enquanto a simulação estiver ativa
    {
      // handle serial
        unsigned char buf[80];
        int rdlen;

        rdlen = read(fd, buf, sizeof(buf) - 1);
        if (rdlen > 0) {
            unsigned char   *p;
            for (p = buf; rdlen-- > 0; p++)
              if('0'<=*p && *p<='9'){  
                 for(int i = 0; i < 16; i++)
      {
  //handle sim
	simxUChar state;
	simxFloat coord[3];
	
	if (simxReadProximitySensor(clientID,sensorHandle[i],&state,coord,NULL,NULL,simx_opmode_buffer)==simx_return_ok)
	{ 
	  float dist = coord[2];
	  if(state > 0 && (dist<noDetectionDist))
	  {
	    if(dist<maxDetectionDist)
	    {
	      dist=maxDetectionDist;
	    }
	    
	    detect[i]=1-((dist-maxDetectionDist)/(noDetectionDist-maxDetectionDist));
	  }
	  else
	    detect[i] = 0;
	}
	else
	  detect[i] = 0;
      }
      
      vLeft = v0;
      vRight = v0;
      
      for(int i = 0; i < 16; i++)
      {
	vLeft=vLeft+braitenbergL[i]*detect[i];
        vRight=vRight+braitenbergR[i]*detect[i];
      }
      
      // atualiza velocidades dos motores
      if(*p == '0'){
        simxSetJointTargetVelocity(clientID, leftMotorHandle, (simxFloat) vLeft, simx_opmode_streaming);
        simxSetJointTargetVelocity(clientID, rightMotorHandle, (simxFloat) vRight, simx_opmode_streaming);
      }
      else{
        simxSetJointTargetVelocity(clientID, leftMotorHandle, (simxFloat) 0, simx_opmode_streaming);
        simxSetJointTargetVelocity(clientID, rightMotorHandle, (simxFloat) 0, simx_opmode_streaming);
      }
      // espera um pouco antes de reiniciar a leitura dos sensores
      extApi_sleepMs(5);


              }
	    
        } else if (rdlen < 0) {
            printf("Error from read: %d: %s\n", rdlen, strerror(errno));
        }
     
    }
      
    simxFinish(clientID); // fechando conexao com o servidor
    cout << "Conexao fechada!" << std::endl;
  }
  else
    cout << "Problemas para conectar o servidor!" << std::endl;
  return 0;
}

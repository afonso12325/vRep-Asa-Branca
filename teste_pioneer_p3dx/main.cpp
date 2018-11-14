/* 
 * File:   main.cpp
 * Author: Daniel de Filgueiras Gomes ( daniel.fgomes@ufpe.br )
 * This source code is licensed under the MLP2 terms (MLP2.html or https://www.mozilla.org/en-US/MPL/2.0/ )
 * This code is provided as an illustrative example of communication with vRep.
 * Department of Electronics and Systems/UFPE. ( https://www.ufpe.br/des/o-des )
 * Created on 1 de Outubro de 2018, 11:18
 */

        
#include <stdio.h>
#include <stdlib.h>

extern "C" {
    #include "remoteApi/extApi.h"
}
/*
 *  scene: apiRemoto_BUMPER
    leftJointHandle=sim.getObjectHandle("dr12_leftJoint_")
    rightJointHandle=sim.getObjectHandle("dr12_rightJoint_")
    bumperSensorHandle=sim.getObjectHandle("dr12_bumperForceSensor_")
    backwardModeUntilTime=0
    currentTime=sim.getSimulationTime()
    result,f,t=sim.readForceSensor(bumperSensorHandle)
    if (result>0) then
        if (math.abs(f[2])>1) or (math.abs(f[3])>1) then
            backwardModeUntilTime=currentTime+2 -- 2 seconds backwards
        end
    end
    
    
    if (currentTime<backwardModeUntilTime) then
        sim.setJointTargetVelocity(leftJointHandle,-100*math.pi/180)
        sim.setJointTargetVelocity(rightJointHandle,-50*math.pi/180)
    else
        sim.setJointTargetVelocity(leftJointHandle,200*math.pi/180)
        sim.setJointTargetVelocity(rightJointHandle,200*math.pi/180)
    end
*/

int main(int argc,char* argv[])
{
    int usensors[16] = {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};
    float noDetectionDist=0.5;
    float maxDetectionDist=0.2;
    float detect[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    float braitenbergL[16] = {-0.2,-0.4,-0.6,-0.8,-1,-1.2,-1.4,-1.6, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    float braitenbergR[16] = {-1.6,-1.4,-1.2,-1,-0.8,-0.6,-0.4,-0.2, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    float v0 = 2;
    float vLeft, vRight;
    char buf[50];
    float* detected;
    float dist;
    int clientID=simxStart((simxChar*)"127.0.0.1",19999,true,true,2000,5);
    if (clientID!=-1)
    {
        
        printf("Connected to remote API server\n");

        // Now try to retrieve data in a blocking fashion (i.e. a service call):
        int objectCount;
        int* objectHandles;
        int ret=simxGetObjects(clientID,sim_handle_all,&objectCount,&objectHandles,simx_opmode_blocking);
        if (ret==simx_return_ok)
            printf("Number of objects in the scene: %d\n",objectCount);
        else
            printf("Remote API function call returned with error code: %d\n",ret);
    
        
        //Now retrieve streaming data (i.e. in a non-blocking fashion):
        int startTime=extApi_getTimeInMs();
        int mouseX;
        unsigned char statusBumper=0;
        
        simxGetIntegerParameter(clientID,sim_intparam_mouse_x,&mouseX,simx_opmode_streaming); // Initialize streaming
        
        int motorL,motorR,bumper;
        int i;
        
        //simxGetObjectHandle(clientID,"dr12_bumperForceSensor_",&bumper,simx_opmode_blocking);
        simxGetObjectHandle(clientID,"Pioneer_p3dx_leftMotor",&motorL,simx_opmode_blocking);
        simxGetObjectHandle(clientID,"Pioneer_p3dx_rightMotor",&motorR,simx_opmode_blocking);
        for(int i = 0;i<16;i++){
            sprintf(buf, "Pioneer_p3dx_ultrasonicSensor%d",i+1);
            simxGetObjectHandle(clientID,buf,&(usensors[i]),simx_opmode_blocking);
        }
        for(i = 0;i<16;i++){
            simxReadProximitySensor(clientID, usensors[i],NULL, NULL,NULL,NULL,simx_opmode_streaming); 
        }
        simxFloat coord[3];
        while (extApi_getTimeDiffInMs(startTime) < 7000)
        {
            //ret=simxGetIntegerParameter(clientID,sim_intparam_mouse_x,&mouseX,simx_opmode_buffer); // Try to retrieve the streamed data
            //if (ret==simx_return_ok) // After initialization of streaming, it will take a few ms before the first value arrives, so check the return code
            //    printf("Mouse position x: %d\n",mouseX); // Mouse position x is actualized when the cursor is over V-REP's window
            for(int i = 0;i<16;i++){
                simxReadProximitySensor(clientID, usensors[3],NULL, coord,NULL,NULL,simx_opmode_buffer);
                if(coord == NULL){
                    printf("NULO");
                }
                else
                printf("x= %f, y=%f, z=%f \n", coord[0],coord[1],coord[0]);
                dist = coord[2];
                if(dist < noDetectionDist){
                    if(dist<maxDetectionDist){
                        dist = maxDetectionDist;
                    }
                    detect[i] = 1-((dist-maxDetectionDist)/(noDetectionDist-maxDetectionDist));
                }
                else detect[i] = 0;
            }
            
            vLeft = v0;
            vRight = v0;
            for(int i =0;i<16;i++){
                        vLeft=vLeft+braitenbergL[i]*detect[i];
                        vRight=vRight+braitenbergR[i]*detect[i];
            }
            //simxReadForceSensor(clientID,bumper,&statusBumper,vf,vt,simx_opmode_buffer);
            //printf("%f %f %f\n",vf[0],vf[1],vf[2]);
            //if((vf[2])<-4.95) {
                //simxSetJointTargetVelocity(clientID,motorL,0,simx_opmode_oneshot);
                //simxSetJointTargetVelocity(clientID,motorR,0,simx_opmode_oneshot);
            //}
            //else {
                simxSetJointTargetVelocity(clientID,motorL,vLeft,simx_opmode_oneshot);
                simxSetJointTargetVelocity(clientID,motorR,vRight,simx_opmode_oneshot);
            //}
            
        }
        printf("saiu");
        // Now send some data to V-REP in a non-blocking fashion:
        simxAddStatusbarMessage(clientID,"Hello V-REP!",simx_opmode_oneshot);

        // Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
        int pingTime;
        simxGetPingTime(clientID,&pingTime);

        // Now close the connection to V-REP:   
        simxFinish(clientID);
    }
    return(0);
}
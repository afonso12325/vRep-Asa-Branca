/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   TMotor.h
 * Author: daniel
 *
 * Created on 25 de Outubro de 2018, 10:41
 */

#ifndef TMOTOR_H
#define TMOTOR_H

class TMotor {
public:
    enum {FULL_RESET,STD_RESET };
    
    TMotor();
    TMotor(const TMotor& orig);
    virtual ~TMotor();
    void setTau(float tau);
    float getTau() const;
    void setDNoise(float dNoise);
    float getDNoise() const;
    void setMNoise(float mNoise);
    float getMNoise() const;
    void setOn(bool state);
    bool isOn() const;
    void setDir(bool dir);
    bool isDir() const;
    void setW(float w);
    float getW() const;
    
    float run(const float &w0);

    TMotor& reset(const int resetType=TMotor::STD_RESET);
    
private:
    float w0;//velocidade atual
    float w;
    bool dir;
    bool state;
    
    float mNoise;
    float dNoise;
    float tau;
    
    void init();
    

};

#endif /* TMOTOR_H */


/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   TMotor.cpp
 * Author: daniel
 * 
 * Created on 25 de Outubro de 2018, 10:41
 */

#include "TMotor.h"

TMotor::TMotor() {
    init();
}

TMotor::TMotor(const TMotor& orig) {
    //init();
    this->w0=orig.w0;
    this->dNoise=orig.dNoise;
    
    
}

TMotor::~TMotor() {
}

void TMotor::setTau(float tau) {
    this->tau = tau;
}

float TMotor::getTau() const {
    return tau;
}

void TMotor::setDNoise(float dNoise) {
    this->dNoise = dNoise;
}

float TMotor::getDNoise() const {
    return dNoise;
}

void TMotor::setMNoise(float mNoise) {
    this->mNoise = mNoise;
}

float TMotor::getMNoise() const {
    return mNoise;
}

void TMotor::setOn(bool state) {
    this->state = state;
}

bool TMotor::isOn() const {
    return state;
}

void TMotor::setDir(bool dir) {
    this->dir = dir;
}

bool TMotor::isDir() const {
    return dir;
}

void TMotor::setW(float w) {
    this->w = w;
}

float TMotor::getW() const {
    return w;
}

/*
 * Modelo de primeira orde
 * @param w0 - velocidade atual dada pelo simulador
 * @return   - Nova velocidade 1dt a mais no tempo
 */
float TMotor::run(const float &w0) {
    
    
    return w0;
}

void TMotor::reset(const int resetType) {
    if (resetType == TMotor::STD_RESET) {
        w0 = 0;
        
    } else
        if (resetType == TMotor::FULL_RESET) {

        init();
        w0 = 0;
        
    }
    
    return *this;

}
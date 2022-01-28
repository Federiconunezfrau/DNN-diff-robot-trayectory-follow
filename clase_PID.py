#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Sep  7 17:51:39 2021

@author: federico
"""

import numpy as np

class PID():
    def __init__(self,Kp,Ti,Td,T):
        
        # parámetros del PID
        self.Kp = Kp
        self.Ti = Ti
        self.Td = Td
        
        # Período de muestreo: el PID controla cada "T" segundos
        self.T = T
        
        # constante para usar con el derivador
        self.N = 10
        
        # valor acumulado del integrador
        self.I = 0
        
        # valor acumulado del derivador
        self.D = 0
        
        # referencia del controlador
        # self.reference = 0
        
        # salida del sistema
        # self.current_state = 0
        
        
    def control(self,e):
        # delta_y: salida de la planta - salida anterior
        
        # se calcula el error entre referencia y estado actual
        
        # Se calcula la acción de control para theta
        P = self.Kp * e
        
        #I = self.I + self.Kp * (self.T/self.Ti) * e
        I = 0
    
        #D = self.Td * self.D/(self.Td + self.N * self.T) - self.Kp * self.Td * self.N * e / (self.Td + self.N * self.T)
        
        D = 0
        
        u = P + I + D
        
        self.I = I
        self.D = D
        
        return u

def calculate_auxiliar_reference(desired, current):
    # Recibe la pose actual y la pose deseada. Calcula una referencia para 
    # utilizar con el controlador.
    
    delta_x = desired[0] - current[0]
    delta_y = desired[1] - current[1]
    
    delta_d = np.sqrt((delta_x)**2 + (delta_y)**2)
    beta = np.arctan2((delta_y),(delta_x))
    
    # if delta_x == 0:
        
    #     if delta_y > 0:
    #         beta = np.pi/2
    
    #     elif delta_y < 0:
    #         beta = -np.pi/2
        
    #     else:
    #         beta = 0
    
    # else:
    #     beta = np.arctan(delta_y/delta_x)
    
    gamma = beta - desired[2]
    
    # se normaliza gamma para obtener un número entre -pi y pi
    
    while (gamma > np.pi):
        gamma = gamma - 2*np.pi
    
    while(gamma < -np.pi):
        gamma = gamma + 2*np.pi
    
    if (gamma > np.pi/2):
        gamma = np.pi/2
        # gamma = -1 * gamma + np.pi

    if (gamma < -np.pi/2):
        gamma = -np.pi/2
        #gamma = -1 * gamma - np.pi
    
    x_ref = current[0] + delta_d * np.cos(beta-gamma)
    y_ref = current[1] + delta_d * np.sin(beta-gamma)
    
    return (x_ref,y_ref)
    

def calculate_references(reference,current_state):
    # A partir de una pose y de la pose de referencia, se calculan las
    # variables auxiliares "s" y "theta"
    
    
    epsilon = 0.01
    
    x_ref = reference[0]
    y_ref = reference[1]
    
    x = current_state[0]
    y = current_state[1]
    theta = current_state[2]
    
    #delta_y = y_ref - y
    #delta_x = x_ref - x
    
    # se calcula e_theta
    # epsilon = 0.001
    theta_ref = np.arctan2((y_ref - y),(x_ref - x))
    # theta_ref = reference[2]
    # if delta_y == 0:
    
    #     if delta_y > 0:
    #         theta_ref = np.pi/2

    #     elif delta_y < 0:
    #         theta_ref = -np.pi/2
    
    #     else:
    #         theta_ref = 0

    # else:
    #     theta_ref = np.arctan(delta_y/delta_x)
    
    
    e_theta = theta_ref - theta
    
    # se normaliza e_theta para obtener un número entre -pi y pi
    while(e_theta> np.pi):
        e_theta = e_theta - 2 * np.pi

    while(e_theta < -np.pi):
        e_theta = e_theta + 2 * np.pi
    
    # se calcula e_s
    delta_l = np.sqrt((x_ref - x)**2 + (y_ref - y)**2) 
    
    if delta_l < epsilon:
        e_theta = 0
        e_s = 0
    
    else:
        e_s = delta_l * np.cos(theta_ref - theta)
    
    
    
    return e_theta,e_s

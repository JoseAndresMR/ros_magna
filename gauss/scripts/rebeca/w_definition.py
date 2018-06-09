# -*- coding: utf-8 -*-
"""
Created on Fri Jun  1 19:07:35 2018

@author: rebeca

CODIGO ACCESO A RED NEURONAL EN FUNCON DE LAS ENTRADAS UAV Y OBS
ADEMAS SE ESTRUCTURAN LOS VECTORES OBSTACULOS QUE LUEGO SE USARAN EN MATRIZ_ENTRADA
"""

import os
import pandas as pd
import numpy as np

dir = os.getcwd()
# Lista de archivos uav. Simulaciones de los distintos UAV
pos_obs = []
type_world = []
archivo = []
num_sim=0

position_obs_x = []
position_obs_y = []
position_obs_z = []
pos_obs_x = []
pos_obs_y = []
pos_obs_z = []

"""
PODEMOS VARIAR ESTOS VALORES PARA USAR UNA RED NEURONAL U OTRA,SEGUN NUMERO DE UAV Y OBS

POSTERIORMENTE, ESTOS VALORES SERAN DECLARADOS EN FUNCION DE LO QUE ESCRIBIMOS EN EL SIMULADOR

"""
uav = 1
obs = 0

"""
FIN DE VARIABLES DEPENDIENTES DEL CODIGO
"""

for simulaciones in os.listdir(dir+ "/type1_Nuav%s"  % uav + "_Nobs%s"  % obs + "/dataset_1" ):
    if simulaciones.startswith("simulation"):        
        type_world.append(simulaciones)   
        num_sim = len(type_world)
        dir_type = dir+ "/type1_Nuav%s" % uav + "_Nobs%s"  % obs+ "/dataset_1"

for sim in range (1, len(type_world)+1):
    dir_world = dir_type + "/simulation_%s" % sim
    for archivo in os.listdir(dir_world):
        input_world=[]
        if archivo.startswith("world"):   
            input_world = pd.read_csv(dir_world+ "/world_definition.csv", sep=',')
            if (obs > 0):
                pos_obs= np.matrix(input_world['obs_pose_list_simple'][0])
    if (obs > 0):    
        for obstacle in range (0,obs):
            offset = 3*obstacle
            pos_obs_x.append(float(pos_obs.T[0+offset]))
            pos_obs_y.append(float(pos_obs.T[1+offset]))
            pos_obs_z.append(float(pos_obs.T[2+offset])) 
if (obs > 0):        
    position_obs_x = np.reshape(pos_obs_x, (num_sim,obs))
    position_obs_y = np.reshape(pos_obs_x, (num_sim,obs))
    position_obs_z = np.reshape(pos_obs_x, (num_sim,obs))
    

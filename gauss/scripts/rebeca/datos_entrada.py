# -*- coding: utf-8 -*-
"""
Created on Mon Apr 23 14:15:53 2018

@author: rebeca fernandez niederacher

    CREACION DE MATRIZ DE ENTRADA Y SALIDA DE LA RED, APOYANDONOS EN LOS DATOS EXTRAIDOS EN DATOS.PY
"""

import datos
import w_definition
import pandas as pd


"""
DECLARAMOS MATRICES DE ENTRADA Y SALIDA A LA RED
"""
input_matrix = []
label = []

"""
DECLARAMOS UNA SERIE DE VARIABLES DE LAS QUE DEPENDE LA MATRIZ DE ENTRADA
"""

position_x = []
position_y = []
position_z = []
orientation_x = []
orientation_y = []
orientation_z = [] 
orientation_w = [] 
lin_x = [] 
lin_y = []
lin_z = []
ang_x = []
ang_y = []
ang_z = []
pos_uav_x = []
pos_uav_y= []
pos_uav_z= []
orien_uav_x = []
orien_uav_y = []
orien_uav_z = []
orien_uav_w = []
new_lin_x = []
new_lin_y = []
new_lin_z = []
new_ang_x = []
new_ang_y = []
new_ang_z = []
goal_pos_x = []
goal_pos_y = []
goal_pos_z = []
goal_orien_x = []
goal_orien_y = []
goal_orien_z = []
goal_orien_w = []
lin_uav_x = []
lin_uav_y = []
lin_uav_z = []
ang_uav_x = []
ang_uav_y = []
ang_uav_z = []

"""
ACCEDEMOS A LOS VALORES EXTRAIDOS EN WORLD_DEFINITION.PY
"""
obs_x = w_definition.position_obs_x
obs_y =  w_definition.position_obs_y
obs_z =  w_definition.position_obs_z

obstacle_x = []
obstacle_y = []
obstacle_z = []

n_uav = w_definition.uav
n_obs = w_definition.obs
dir_input = w_definition.dir_type
num_sim = w_definition.num_sim
valid_simulation = w_definition.valid_simulation
"""
LLAMAMOS AL ARCHIVO DATOS.PY DONDE EXTRAIMOS LOS DATOS DE LOS ARCHIVOS .CSV
"""

for sim in range (0, num_sim):
    for UAV in range (1, n_uav+1):
        input_data = []
        input_data = pd.read_csv(dir_input + "/simulation_%s" % valid_simulation[sim] + "/uav_%s.csv" % UAV, sep=',')
        datos.pos_orien_vel(input_data, n_uav,  UAV,lin_uav_x, lin_uav_y, lin_uav_z, ang_uav_x, ang_uav_y, ang_uav_z, goal_pos_x, goal_pos_y, goal_pos_z, goal_orien_x, goal_orien_y, goal_orien_z, goal_orien_w,new_lin_x, new_lin_y, new_lin_z, new_ang_x, new_ang_y, new_ang_z,pos_uav_x,pos_uav_y,pos_uav_z ,orien_uav_x,orien_uav_y,orien_uav_z,orien_uav_w, position_x, position_y, position_z, orientation_x, orientation_y, orientation_z, orientation_w, lin_x, lin_y, lin_z, ang_x, ang_y, ang_z)
        for n_obstacle in range (0, n_obs):
            obstacle_x.append([])
            obstacle_y.append([])
            obstacle_z.append([])
            for len_obs in range (0, len(input_data['actual_UAV_%s_pose' % UAV])):
                obstacle_x[n_obstacle].append(obs_x[sim-1][n_obstacle])
                obstacle_y[n_obstacle].append(obs_y[sim-1][n_obstacle])
                obstacle_z[n_obstacle].append(obs_z[sim-1][n_obstacle])


"""
CREACION DE LA MATRIZ DE ENTRADA
"""

for j in range(0, len(pos_uav_x)):
    input_matrix.append([])
    label.append([])
    input_matrix[j].append(float(lin_uav_x[j]))
    input_matrix[j].append(float(lin_uav_y[j]))
    for i in range (0, n_uav):  
              
        #DIF POS
        while(float(pos_uav_x[j]) != float(position_x[i][j])):
            input_matrix[j].append(float(position_x[i][j]) -float(pos_uav_x[j]))
            input_matrix[j].append(float(position_y[i][j]) -float(pos_uav_y[j]))
            """
            input_matrix[j].append(float(position_z[i][j]) -float(pos_uav_z[j]))
            input_matrix[j].append(float(orientation_x[i][j]) - float(orien_uav_x[j]))
            input_matrix[j].append(float(orientation_y[i][j]) - float(orien_uav_y[j]))
            input_matrix[j].append(float(orientation_z[i][j]) - float(orien_uav_z[j]))
            input_matrix[j].append(float(orientation_w[i][j]) - float(orien_uav_w[j])) #7*n_uav
            """
        # DIF VEL

        while(float(lin_uav_x[j]) != float(lin_x[i][j])):
            input_matrix[j].append(float(lin_x[i][j]))
            input_matrix[j].append(float(lin_y[i][j]))
            """
            #input_matrix[j].append(float(lin_z[i][j]))
            #input_matrix[j].append(float(ang_x[i][j]))
            #input_matrix[j].append(float(ang_y[i][j]))
            #input_matrix[j].append(float(ang_z[i][j])) #6*n_uav
            """
    #obstaculos
    if (n_obs > 0):
        for num_obstacle in range (0,n_obs): 
            input_matrix[j].append(float(obstacle_x[num_obstacle][j]) - float(pos_uav_x[j]))
            input_matrix[j].append(float(obstacle_y[num_obstacle][j]) - float(pos_uav_y[j]))
            """
            input_matrix[j].append(float(obstacle_z[num_obstacle][j]) - float(pos_uav_z[j]))
            input_matrix[j].append(0.0 - float(pos_uav_y[j]))
            input_matrix[j].append(0.0 - float(pos_uav_y[j]))
            input_matrix[j].append(0.0 - float(pos_uav_y[j]))
            input_matrix[j].append(0.0 - float(pos_uav_y[j])) #n_uav*7
            """
    #DIF POS Y OBJETIVO
    input_matrix[j].append(float(goal_pos_x[j]) - float(pos_uav_x[j]))
    input_matrix[j].append(float(goal_pos_y[j]) - float(pos_uav_y[j]))
    """
    #input_matrix[j].append(float(goal_pos_z[j]) - float(pos_uav_z[j]))
    #input_matrix[j].append(float(goal_orien_x[j]) - float(orien_uav_x[j]))
    #input_matrix[j].append(float(goal_orien_y[j]) - float(orien_uav_y[j]))
    #input_matrix[j].append(float(goal_orien_z[j]) - float(orien_uav_z[j]))
    #input_matrix[j].append(float(goal_orien_w[j]) - float(orien_uav_w[j])) #7
    """
    #NEW VELOCITY
    label[j].append(float(new_lin_x[j]))       
    label[j].append(float(new_lin_y[j]))
    """
    label[j].append(float(new_lin_z[j]))  
    label[j].append(float(new_ang_x[j]))  
    label[j].append(float(new_ang_y[j]))  
    label[j].append(float(new_ang_z[j]))
    """
"""
VALORES NECESARIOS PARA LA RED NEURONAL, NOS INFORMAN DEL NUMERO D ENTRADAS DE LA MATRIZ Y EL NUMERO DE SIMULACIONES TOTALES
"""
features_len = len(input_matrix[0])
simulation_len =  len(input_matrix)
label_len = len(label[0])
print ("simulation_len:")
print (simulation_len)
print ("number of input features:")
print (features_len)
print("label len")
print (label_len)


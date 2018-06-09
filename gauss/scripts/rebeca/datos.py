# -*- coding: utf-8 -*-
"""
Created on Thu Apr 26 12:13:29 2018

@author: rebeca fernandez niederacher
"""
# -*- coding: utf-8 -*-
"""
Created on Thu Apr 19 10:00:41 2018

@author: rebeca fernandez niederacher
"""
""" AQUÍ EXTRAEMOS LOS DATOS DEL CSV Y LOS ALMACENAMOS EN DIFERENTES VECTORES, DE MODO QUE TENGAMOS TODOS LOS 
    DATOS DE LOS ARCHIVOS DISPONIBLES.
    
"""


"""
FUNCIONES PARA EXTRAER LOS DIFERENTES VALORES SEGUN EJES COORDENADOS
"""
def find_pos(uav, first, last):
    try:
        start = uav.index(first) + len( first )
        end = uav.index(last, start )
        return uav[start:end]
    except ValueError:
        return ""
        
def find_orient(uav, first, last):
    try:
        start = uav.rindex(first) + len(first)
        end = uav.rindex(last,start)
        return uav[start:end]
    except ValueError:
        return ""

"""
FUNCION QUE ES LLAMADA POR DATOS_ENTRADA
"""

def pos_orien_vel(input_data, n_uav, UAV,lin_uav_x, lin_uav_y, lin_uav_z, ang_uav_x, ang_uav_y, ang_uav_z, goal_pos_x, goal_pos_y, goal_pos_z, goal_orien_x, goal_orien_y, goal_orien_z, goal_orien_w,new_lin_x, new_lin_y, new_lin_z, new_ang_x, new_ang_y, new_ang_z,pos_uav_x,pos_uav_y,pos_uav_z ,orien_uav_x,orien_uav_y,orien_uav_z,orien_uav_w,position_x,position_y,position_z, orientation_x,orientation_y,orientation_z,orientation_w,lin_x, lin_y, lin_z, ang_x, ang_y, ang_z):  
    for uav in range (1, n_uav+1):
        uav_pos = []
        uav_vel = []
        new_vel = []
        goal_position = []
        goal = False
        if uav == UAV:
            goal = True
        else :
            goal = False
        position_x.append([])
        position_y.append([]) 
        position_z.append([]) 
        orientation_x.append([])
        orientation_y.append([])
        orientation_z.append([])
        orientation_w.append([])
        
        lin_x.append([])
        lin_y.append([])
        lin_z.append([])
        ang_x.append([])
        ang_y.append([])
        ang_z.append([])
        for  j in range (0, len(input_data['actual_UAV_%s_pose' % uav])): 
            
            uav_pos.append(input_data['actual_UAV_%s_pose' % uav][j])
            uav_vel.append(input_data['actual_UAV_%s_vel' % uav][j])

            position_x[uav-1].append(find_pos(uav_pos[j], 'x:','\n'))
            position_y[uav-1].append(find_pos(uav_pos[j], 'y:', '\n'))
            position_z[uav-1].append(find_pos(uav_pos[j], 'z:', '\n'))
            orientation_x[uav-1].append(find_orient(uav_pos[j] ,'x:' , '\n  y:' ))
            orientation_y[uav-1].append(find_orient(uav_pos[j], 'y:' ,  '\n  z:' ))
            orientation_z[uav-1].append(find_orient(uav_pos[j], 'z:' , '\n  w:'))
            orientation_w[uav-1].append(find_orient(uav_pos[j], 'w: ' , ''))
            
            lin_x[uav-1].append(find_pos(uav_vel[j], 'x:' , '\n'))
            lin_y[uav-1].append(find_pos(uav_vel[j], 'y:' , '\n'))
            lin_z[uav-1].append(find_pos(uav_vel[j], 'z:' , '\n'))
            ang_x[uav-1].append(find_orient(uav_vel[j], 'x:' ,'\n  y:'))
            ang_y[uav-1].append(find_orient(uav_vel[j], 'y:' , '\n'))
            ang_z[uav-1].append(find_orient(uav_vel[j], 'z:' ,  ''))
            
        
            if goal == True:
                pos_uav_x.append(find_pos(uav_pos[j], 'x:','\n'))
                pos_uav_y.append(find_pos(uav_pos[j], 'y:', '\n'))
                pos_uav_z.append(find_pos(uav_pos[j], 'z:', '\n'))
                orien_uav_x.append(find_orient(uav_pos[j] ,'x:' , '\n  y:' ))
                orien_uav_y.append(find_orient(uav_pos[j], 'y:' ,  '\n  z:' ))
                orien_uav_z.append(find_orient(uav_pos[j], 'z:' , '\n  w:'))
                orien_uav_w.append(find_orient(uav_pos[j], 'w: ' , ''))
                
                lin_uav_x.append(find_pos(uav_vel[j], 'x:' , '\n'))
                lin_uav_y.append(find_pos(uav_vel[j], 'y:' , '\n'))
                lin_uav_z.append(find_pos(uav_vel[j], 'z:' , '\n'))
                ang_uav_x.append(find_orient(uav_vel[j], 'x:' ,'\n  y:'))
                ang_uav_y.append(find_orient(uav_vel[j], 'y:' , '\n'))
                ang_uav_z.append(find_orient(uav_vel[j], 'z:' ,  ''))
                
                goal_position.append(input_data['goal_UAV_%s_pose' % uav][j])
                new_vel.append(input_data['UAV_%s_new_velocity_twist' % uav][j])
                
                goal_pos_x.append(find_pos(goal_position[j], 'x:','\n'))
                goal_pos_y.append(find_pos(goal_position[j], 'y:', '\n'))
                goal_pos_z.append(find_pos(goal_position[j], 'z:', '\n'))
                goal_orien_x.append(find_orient(goal_position[j] ,'x:' ,'\n  y:' ))
                goal_orien_y.append(find_orient(goal_position[j], 'y:' , '\n  z:' ))
                goal_orien_z.append(find_orient(goal_position[j], 'z:' ,'\n  w:'))
                goal_orien_w.append(find_orient(goal_position[j], 'w:' ,''))
                
                new_lin_x.append(find_pos(new_vel[j], 'x:' , '\n'))
                new_lin_y.append(find_pos(new_vel[j], 'y:' , '\n'))
                new_lin_z.append(find_pos(new_vel[j], 'z:' , '\n'))
                new_ang_x.append(find_orient(new_vel[j], 'x:' ,'\n  y:'))
                new_ang_y.append(find_orient(new_vel[j], 'y:' , '\n'))
                new_ang_z.append(find_orient(new_vel[j], 'z:' ,  ''))
              
    

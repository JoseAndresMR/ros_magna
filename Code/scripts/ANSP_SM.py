from smach import StateMachine, State, CBState, cb_interface
import smach_ros
import time
from smach_ros import SimpleActionState
import numpy as np

from Worlds import *
from pydag.msg import *

class ANSP_SM(object):
    def __init__(self,heritage):
        self.ansp_sm = StateMachine(outcomes=['completed', 'failed'])
        self.ansp_sm.userdata.id = 1
        self.ansp_sm.userdata.n_uav = 0

        with self.ansp_sm:
            StateMachine.add('new_world', CBState(self.new_world_stcb,cb_kwargs={'heritage':heritage}),
                                    {'completed':'spawn_uav_world'})

            def follow_path_goal_cb(ud, goal, heritage):
                goal.goal_path_poses_list = np.array(heritage.uavs_goal_paths_list[ud.n_uav])
                # print type(goal.goal_path_poses_list[0])
                return goal

            StateMachine.add('spawn_uav_world',
                       CBState(self.spawn_uav_stcb,
                                        input_keys=['id','n_uav'],
                                        cb_kwargs={'heritage':heritage}),
                       transitions={'completed':'follow_paths'},
                       remapping={'id':'id'})

            StateMachine.add('follow_paths',
                      SimpleActionState('/pydag/ANSP_UAV_{}/follow_path_command'.format(self.ansp_sm.userdata.id),
                                        FollowPathAction,
                                        goal_cb=follow_path_goal_cb,
                                        result_cb=self.follow_path_result_cb,
                                        input_keys=['id',"n_uav"],
                                        goal_cb_kwargs={'heritage':heritage}),
                      transitions={'succeeded':'completed', 'preempted':'completed','aborted':'failed'},
                      remapping={'id':'id'})

    @cb_interface(outcomes=['completed','failed'])
    def new_world_stcb(ud,heritage):
        # Local parameters inizialization
        heritage.CreatingSimulationDataStorage()
        heritage.world = Worlds(1)
        heritage.GettingWorldDefinition()
        heritage.uavs_goal_paths_list = []

        return "completed"


    @cb_interface(outcomes=['completed','failed'])
    def spawn_uav_stcb(ud,heritage):  ## iterar entre entradas para probar
        uav_goal_path = heritage.world.PathGenerator()
        heritage.uavs_goal_paths_list.append(uav_goal_path)
        first_pose = uav_goal_path[0]
        heritage.SetUavSpawnFeatures(ud.n_uav+1,heritage.uav_models[ud.n_uav],[first_pose.position.x,first_pose.position.y,0])      
        heritage.UAVSpawner1(ud.n_uav)  
        # time.sleep(10000)

        return "completed"

    # # @cb_interface(outcomes=['completed','failed'])
    # def follow_path_goal_cb(ud, goal, heritage):
    #    return heritage.uavs_goal_paths_list(ud.n_uav)

    def follow_path_result_cb(ud, heritage):
       return

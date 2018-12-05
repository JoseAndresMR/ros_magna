from smach import StateMachine, State, CBState, cb_interface
import smach_ros
import rospy, time
from smach_ros import ActionServerWrapper

from pydag.msg import *

class Ground_Station_SM(object):
    # At init, the State Machine receives as "heriage" the hole "self" of the Ground Station
    def __init__(self,heritage):
        # Creation of State Machine and definition of its outcomes
        self.gs_sm = StateMachine(outcomes=['completed', 'failed'])

        with self.gs_sm:
            # Initialization of the dictionary containing every Action Service Wrapper
            self.asw_dicc = {}

            ### ACTION SERVER ADVERTISING ###

            # Add a state where drone does nothing but wait to receive a service order
            StateMachine.add('action_server_advertiser',
                            CBState(self.action_server_advertiser_stcb,
                                         cb_kwargs={'heritage':heritage,'asw_dicc':self.asw_dicc}),
                            {'completed':'completed'})

            # Every action service wrapper is similar following a structure defined by SMACH.
            # Each wrapper is defined and added to
            # To understand what they do, see callbacks below

            ### TAKE-OFF STATE MACHINE & WRAPPER ###

            self.take_off_sm = StateMachine(outcomes=['completed', 'failed'],
                                         input_keys=['action_goal'])

            self.asw_dicc['take_off'] = ActionServerWrapper(
                        '/pydag/ANSP_UAV_{}/take_off_command'.format(heritage.ID),
                        TakeOffAction,
                        self.take_off_sm,
                        ['completed'], ['failed'], ['preempted'],
                        goal_key = 'action_goal',
                        result_key = 'action_result' )

            with self.take_off_sm:

                StateMachine.add('take_off',
                                 CBState(self.take_off_stcb,
                                         cb_kwargs={'heritage':heritage}),
                                 {'completed':'completed'})

            ### LAND STATE MACHINE & WRAPPER ###

            self.land_sm = StateMachine(outcomes=['completed', 'failed'],
                                         input_keys=['action_goal'])

            self.asw_dicc['land'] = ActionServerWrapper(
                        '/pydag/ANSP_UAV_{}/land_command'.format(heritage.ID),
                        LandAction,
                        self.land_sm,
                        ['completed'], ['failed'], ['preempted'],
                        goal_key = 'action_goal',
                        result_key = 'action_result' )

            with self.land_sm:

                StateMachine.add('land',
                                 CBState(self.land_stcb,
                                         cb_kwargs={'heritage':heritage}),
                                 {'completed':'completed'})


            ### BASIC MOVE STATE MACHINE & WRAPPER ###

            self.basic_move_sm = StateMachine(outcomes=['completed', 'failed'],
                                         input_keys=['action_goal'])

            self.asw_dicc['basic_move'] = ActionServerWrapper(
                        '/pydag/ANSP_UAV_{}/basic_move_command'.format(heritage.ID),
                        BasicMoveAction,
                        self.basic_move_sm,
                        ['completed'], ['failed'], ['preempted'],
                        goal_key = 'action_goal',
                        result_key = 'action_result' )

            with self.basic_move_sm:

                StateMachine.add('basic_move',
                                 CBState(self.basic_move_stcb,
                                         input_keys=['action_goal'],
                                         cb_kwargs={'heritage':heritage}),
                                 {'completed':'completed'})

            ### SAVE CVS STATE MACHINE & WRAPPER ###

            self.save_csv_sm = StateMachine(outcomes=['completed', 'failed'],
                                         input_keys=['action_goal'])

            self.asw_dicc['save_csv'] = ActionServerWrapper(
                        '/pydag/ANSP_UAV_{}/save_csv_command'.format(heritage.ID),
                        SaveCSVAction,
                        self.save_csv_sm,
                        ['completed'], ['failed'], ['preempted'],
                        goal_key = 'action_goal',
                        result_key = 'action_result' )

            with self.save_csv_sm:

                StateMachine.add('save_csv',
                                 CBState(self.save_csv_stcb,
                                         cb_kwargs={'heritage':heritage}),
                                 {'completed':'completed'})

            ### FOLLOW PATH STATE MACHINE & WRAPPER ###
            self.follow_path_sm = StateMachine(outcomes=['completed', 'failed'],
                                         input_keys=['action_goal'])

            self.asw_dicc['follow_path'] = ActionServerWrapper(
                        '/pydag/ANSP_UAV_{}/follow_path_command'.format(heritage.ID),
                        FollowPathAction,
                        self.follow_path_sm,
                        ['completed'], ['failed'], ['preempted'],
                        goal_key = 'action_goal',
                        result_key = 'action_result' )

            with self.follow_path_sm:

                StateMachine.add('follow_path',
                                 CBState(self.follow_path_stcb,
                                         input_keys=['action_goal'],
                                         cb_kwargs={'heritage':heritage}),
                                 {'completed':'completed'})

            # StateMachine.add('to_wp', self.follow_path_sm,
            #                         {'completed':'action_server_advertiser'})

            ### FOLLOW UAV AD STATE MACHINE  & WRAPPER###
            self.follow_uav_ad_sm = StateMachine(outcomes=['completed', 'failed'],
                                         input_keys=['action_goal'])

            self.asw_dicc['follow_uav_ad'] = ActionServerWrapper(
                        '/pydag/ANSP_UAV_{}/follow_uav_ad_command'.format(heritage.ID),
                        FollowUAVADAction,
                        self.follow_uav_ad_sm,
                        ['completed'], ['failed'], ['preempted'],
                        goal_key = 'action_goal',
                        result_key = 'action_result' )

            with self.follow_uav_ad_sm:

                StateMachine.add('follow_uav_ad',
                                 CBState(self.follow_uav_ad_stcb,
                                         input_keys=['action_goal'],
                                         cb_kwargs={'heritage':heritage}),
                                 {'completed':'completed'})

            ### FOLLOW UAV AP STATE MACHINE  & WRAPPER###
            self.follow_uav_ap_sm = StateMachine(outcomes=['completed', 'failed'],
                                         input_keys=['action_goal'])

            self.asw_dicc['follow_uav_ap'] = ActionServerWrapper(
                        '/pydag/ANSP_UAV_{}/follow_uav_ap_command'.format(heritage.ID),
                        FollowUAVAPAction,
                        self.follow_uav_ap_sm,
                        ['completed'], ['failed'], ['preempted'],
                        goal_key = 'action_goal',
                        result_key = 'action_result' )

            with self.follow_uav_ap_sm:

                StateMachine.add('follow_uav_ap',
                                 CBState(self.follow_uav_ap_stcb,
                                         input_keys=['action_goal'],
                                         cb_kwargs={'heritage':heritage}),
                                 {'completed':'completed'})

            if heritage.smach_view == True:
                sis = smach_ros.IntrospectionServer('pydag/UAV_{}_introspection'.format(heritage.ID), self.gs_sm, '/UAV_{}'.format(heritage.ID))
                sis.start()


    @cb_interface(outcomes=['completed', 'failed'])
    def action_server_advertiser_stcb(ud, heritage, asw_dicc):
        heritage.SetVelocityCommand(True)       # Tell GS to hover whole no server request is received
        for key in asw_dicc.keys():     # Run every ASW stored
            asw_dicc[key].run_server()

        # Function to inform ANSP about actual UAV's state
        heritage.state = "waiting for action command"
        heritage.ANSPStateActualization()

        time.sleep(0.2)

        rospy.spin()

        return 'completed'

    @cb_interface(outcomes=['completed','failed'])
    def take_off_stcb(ud,heritage):
        print("TAKE OFF COMMAND")

        heritage.TakeOffCommand(5,True)     # Tell GS to take off. In the future altitude will be received

        # Function to inform ANSP about actual UAV's state
        heritage.state = "inizializating"
        heritage.ANSPStateActualization()

        return 'completed'

    @cb_interface(outcomes=['completed','failed'])
    def follow_path_stcb(ud,heritage):

        # Copy the goal path to follow into GS's variable
        heritage.goal_path_poses_list = ud.action_goal.goal_path_poses_list
        heritage.PathFollower()     # Tell the GS to execute that function

        return 'completed'

    @cb_interface(outcomes=['completed','failed'])
    def follow_uav_ad_stcb(ud,heritage):

        # Tell the GS the identity of its new target
        heritage.state = "following UAV {0}".format(ud.action_goal.target_ID)

        heritage.ANSPStateActualization()       # Function to inform ANSP about actual UAV's state

        # Tell the GS to execute UAVFollowerAD role with at the required distance
        heritage.UAVFollowerAtDistance(ud.action_goal.target_ID,ud.action_goal.distance)

        return 'completed'

    @cb_interface(outcomes=['completed','failed'])
    def follow_uav_ap_stcb(ud,heritage):

        # Tell the GS the identity of its new target
        heritage.state = "following UAV {0}".format(ud.action_goal.target_ID)

        heritage.ANSPStateActualization()       # Function to inform ANSP about actual UAV's state

        # Tell the GS to execute UAVFollowerAP role with the required bias
        heritage.UAVFollowerAtPosition(ud.action_goal.target_ID,ud.action_goal.pos)

        return 'completed'

    @cb_interface(outcomes=['completed','failed'])
    def save_csv_stcb(ud,heritage):
        heritage.StoreData()        # Tell the GS to store the data

        return "completed"


    @cb_interface(outcomes=['completed','failed'])
    def land_stcb(ud,heritage):
        print "landing"
        heritage.LandCommand(True)        # Tell the GS to land

        # Function to inform ANSP about actual UAV's state
        heritage.state = "landed"
        heritage.ANSPStateActualization()

        return 'completed'

    @cb_interface(outcomes=['completed','failed'])
    def basic_move_stcb(ud,heritage):

        # Parse received basic move information
        move_type = ud.action_goal.move_type
        dynamic = ud.action_goal.dynamic
        direction = ud.action_goal.direction
        value = ud.action_goal.value

        # Tell the GS to execute basic move role with parsed information
        heritage.basic_move(move_type,dynamic,direction,value)


        return 'completed'

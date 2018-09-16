from smach import StateMachine, State, CBState, cb_interface
import smach_ros
import rospy, time
from smach_ros import ActionServerWrapper

from pydag.msg import *

class Ground_Station_SM(object):
    def __init__(self,heritage):
        self.heritage = heritage

        self.gs_sm = StateMachine(outcomes=['completed', 'failed'])

        with self.gs_sm:

            # StateMachine.add('take_off',
            #                 CBState(self.take_off_stcb,
            #                              cb_kwargs={'heritage':heritage}),
            #                 {'completed':'action_server_advertiser'})

            self.asw_dicc = {}

            ### ACTION SERVER ADVERTISING ###
            StateMachine.add('action_server_advertiser',
                            CBState(self.action_server_advertiser_stcb,
                                         cb_kwargs={'heritage':heritage,'asw_dicc':self.asw_dicc}),
                            {'completed':'completed'})

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

            ### SAVE CVS STATE MACHINE & WRAPPER ###

            self.save_csv_sm = StateMachine(outcomes=['completed', 'failed'],
                                         input_keys=['action_goal'])

            self.asw_dicc['save_csv'] = ActionServerWrapper(
                        '/pydag/ANSP_UAV_{}/land_command'.format(heritage.ID),
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

            ### FOLLOW UAV STATE MACHINE  & WRAPPER###
            self.follow_uav_sm = StateMachine(outcomes=['completed', 'failed'],
                                         input_keys=['action_goal'])

            self.asw_dicc['follow_uav'] = ActionServerWrapper(
                        '/pydag/ANSP_UAV_{}/follow_uav_command'.format(heritage.ID),
                        FollowUAVAction,
                        self.follow_uav_sm,
                        ['completed'], ['failed'], ['preempted'],
                        goal_key = 'action_goal',
                        result_key = 'action_result' )

            with self.follow_uav_sm:

                StateMachine.add('follow_uav',
                                 CBState(self.follow_uav_stcb,
                                         input_keys=['action_goal'],
                                         cb_kwargs={'heritage':heritage}),
                                 {'completed':'completed'})

    @cb_interface(outcomes=['completed','failed'])
    def take_off_stcb(ud,heritage):
        print("TAKE OF COMMAND")
        heritage.TakeOffCommand(5,True)
        heritage.state = "inizializating"
        heritage.ANSPStateActualization()

        return 'completed'

    @cb_interface(outcomes=['completed','failed'])
    def action_server_advertiser_stcb(ud,heritage,asw_dicc):
        heritage.SetVelocityCommand(True)
        # [asw_dicc[key].run_server() for key in asw_dicc.keys()]
        asw_dicc['take_off'].run_server()
        asw_dicc['land'].run_server()
        asw_dicc['save_csv'].run_server()
        asw_dicc['follow_path'].run_server()
        asw_dicc['follow_uav'].run_server()
        heritage.state = "waiting for action command"
        heritage.ANSPStateActualization()

        time.sleep(0.2)

        rospy.spin()

        return 'completed'

    @cb_interface(outcomes=['completed','failed'])
    def follow_path_stcb(ud,heritage):
        heritage.new_path_incoming = False
        heritage.goal_path_poses_list = ud.action_goal.goal_path_poses_list
        heritage.PathFollowerLegacy()

        return 'completed'

    @cb_interface(outcomes=['completed','failed'])
    def follow_uav_stcb(ud,heritage):
        heritage.new_path_inceoming = False
        heritage.state = "following UAV {0}".format(ud.action_goal.target_ID)
        heritage.ANSPStateActualization()
        heritage.UAVFollower(ud.action_goal.target_ID)

        return 'completed'

    @cb_interface(outcomes=['completed','failed'])
    def save_csv_stcb(ud,heritage):
        heritage.CreatingCSV()

        return "completed"


    @cb_interface(outcomes=['completed','failed'])
    def land_stcb(ud,heritage):
        print "landing"
        heritage.LandCommand(True)
        heritage.state = "landed"
        heritage.ANSPStateActualization()

        return 'completed'
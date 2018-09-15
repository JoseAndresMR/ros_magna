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

            StateMachine.add('take_off',
                            CBState(self.take_off_stcb,
                                         cb_kwargs={'heritage':heritage}),
                            {'completed':'action_server_advertiser'})

            self.asw_dicc = {}

            ### FOLLOW PATH STATE MACHINE ###
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

            ### FOLLOW UAV STATE MACHINE ###
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

            # StateMachine.add('to_wp', self.follow_path_sm,
            #                         {'completed':'action_server_advertiser'})


            ### ACTION SERVER ADVERTISING ###
            StateMachine.add('action_server_advertiser',
                            CBState(self.action_server_advertiser_stcb,
                                         cb_kwargs={'heritage':heritage,'asw_dicc':self.asw_dicc}),
                            {'completed':'land'})

            StateMachine.add('land', CBState(self.landing_stcb,cb_kwargs={'heritage':heritage}),
                                    {'completed':'completed'})

    @cb_interface(outcomes=['completed','failed'])
    def take_off_stcb(ud,heritage):
        heritage.TakeOffCommand(5,True)
        heritage.state = "inizializating"
        heritage.ANSPStateActualization()

        return 'completed'

    @cb_interface(outcomes=['completed','failed'])
    def action_server_advertiser_stcb(ud,heritage,asw_dicc):
        heritage.SetVelocityCommand(True)
        asw_dicc['follow_path'].run_server()
        asw_dicc['follow_uav'].run_server()
        heritage.state = "waiting for action command"
        heritage.ANSPStateActualization()

        rospy.spin()

        return 'completed'

    @cb_interface(outcomes=['completed','failed'])
    def follow_path_stcb(ud,heritage):
        heritage.new_path_incoming = False
        heritage.goal_path_poses_list = ud.action_goal.goal_path_poses_list
        heritage.PathFollowerLegacy()
        # if heritage.new_path_incoming == False:
        #     heritage.state = "stabilizing"
        #     heritage.ANSPStateActualization()
        # else:
        #     pass
        
        return 'completed'

    @cb_interface(outcomes=['completed','failed'])
    def follow_uav_stcb(ud,heritage):
        heritage.new_path_inceoming = False
        heritage.state = "following UAV {0}".format(ud.action_goal.target_ID)
        heritage.ANSPStateActualization()
        heritage.UAVFollower(ud.action_goal.target_ID)
        # if heritage.new_path_incoming == False:
        #     heritage.state = "stabilizing"
        #     heritage.ANSPStateActualization()
        # else:
        #     pass
        
        return 'completed'

    @cb_interface(outcomes=['completed','failed'])
    def landing_stcb(ud,heritage):
        print "landing"
        heritage.CreatingCSV()
        heritage.LandCommand(True)
        heritage.state = "landed"
        heritage.ANSPStateActualization()

        return 'completed'
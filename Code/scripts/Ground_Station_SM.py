from smach import StateMachine, State, CBState, cb_interface
import smach_ros
import time
from smach_ros import ActionServerWrapper

from pydag.msg import *

class Ground_Station_SM(object):
    def __init__(self,heritage):
        self.heritage = heritage

        # @smach.cb_interface(input_keys=['q'],
        #                     output_keys=['xyz'],
        #                     outcomes=['foo'])
        # def my_cb(ud, x, y, z):
        #     ud.xyz = ud.q + x + y + z
        #     return 'foo'

        self.follow_path_sm = StateMachine(outcomes=['completed', 'failed'])
        # self.gs_sm.userdata.heritage = heritage

        self.asw = ActionServerWrapper(
                        '/pydag/ANSP_UAV_{}/follow_path_command'.format(heritage.ID),
                        FollowPathAction,
                        self.follow_path_sm,
                        ['completed'], ['failed'], ['preempted'],)
                        # goal_key = 'my_awesome_goal',
                        # result_key = 'egad_its_a_result' )

        with self.follow_path_sm:

            StateMachine.add('take_off', CBState(self.take_off_stcb,cb_kwargs={'heritage':heritage}),
                                    {'completed':'to_wp'})

            self.to_wp_sm = StateMachine(outcomes=['completed', 'failed'])

            with self.to_wp_sm:

                StateMachine.add('going_to_wp', CBState(self.going_to_wp_stcb,cb_kwargs={'heritage':heritage}),
                                        {'completed':'hovering'})

                StateMachine.add('hovering', CBState(self.hovering_stcb,cb_kwargs={'heritage':heritage}),
                                        {'completed':'completed'})

            StateMachine.add('to_wp', self.to_wp_sm,
                                    {'completed':'landing'})

            StateMachine.add('landing', CBState(self.landing_stcb,cb_kwargs={'heritage':heritage}),
                                    {'completed':'completed'})




    @cb_interface(outcomes=['completed','failed'])
    def take_off_stcb(ud,heritage):
        time.sleep(10)
        time.sleep((heritage.ID-1) * 8)
        print "flag 1"
        heritage.TakeOffCommand(5,True)
        print "flag 2"
        heritage.state = "inizializating"
        heritage.ANSPStateActualization()

        clock_start = time.time()
        clock = clock_start - time.time()

        return 'completed'

    @cb_interface(outcomes=['completed','failed'])
    def going_to_wp_stcb(ud,heritage):
        heritage.new_path_incoming = False
        heritage.PathFollowerLegacy()
        if heritage.new_path_incoming == False:
            heritage.state = "stabilizing"
            heritage.ANSPStateActualization()
        else:
            pass
        
        return 'completed'

    @cb_interface(outcomes=['completed','failed'])
    def hovering_stcb(ud,heritage):
        heritage.SetVelocityCommand(True)
        time.sleep(0.1)
        heritage.state = "hovering"
        heritage.ANSPStateActualization()

        return 'completed'

    @cb_interface(outcomes=['completed','failed'])
    def landing_stcb(ud,heritage):
        print "landing"
        heritage.LandCommand(True)
        heritage.state = "landed"
        heritage.ANSPStateActualization()

        return 'completed'
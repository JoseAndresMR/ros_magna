from smach_ros import SimpleActionState
from smach import StateMachine, State, CBState, cb_interface
from actionlib_msgs.msg import GripperAction

sm = StateMachine(['succeeded','aborted','preempted'])
with sm:
    def gripper_goal_cb(userdata, goal):
       gripper_goal = GripperGoal()
       gripper_goal.position.x = 2.0
       gripper_goal.max_effort = userdata.gripper_input
       return gripper_goal

    StateMachine.add('TRIGGER_GRIPPER',
                      SimpleActionState('action_server_namespace',
                                        GripperAction,
                                        goal_cb=gripper_goal_cb,
                                        input_keys=['gripper_input']),
                      transitions={'succeeded':'APPROACH_PLUG'},
                      remapping={'gripper_input':'userdata_input'})

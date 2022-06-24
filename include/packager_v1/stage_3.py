from attr import s
import rospy
from std_msgs.msg import String, Float64
from .AuxFuctions import ParseBitfield


class Stage_3:
    class Tray:
        POS_RELEASE = 0
        POS_SEAL = 1
        POS_DISPENSE = 2
        POS_ROUTER = 3
        POS_NONE = 4

        STATE_IDLE = 0
        STATE_HOMING = 1
        STATE_RUNNING = 2
        STATE_ERROR = 3
        STATE_MAPPING = 4
        
        def __init__(self):
            self.ls_min = 0
            self.enc_seal = 0
            self.enc_disp = 0
            self.homed = 0
            self.ls_max = 0
            self.current_pos = self.POS_NONE
            self.state = self.STATE_IDLE
            self.x_pos = 0.0

            # ROS framework
            self.subs_tray_status = rospy.Subscriber('/tray/status', String, self.StatusCallback)
            self.subs_tray_axis = rospy.Subscriber('/tray/axis', Float64, self.AxisCallback)
            
        def __del__(self):
            self.subs_tray_status.unregister()
            self.subs_tray_axis.unregister()

        # Callback for ROS subscriber
        def StatusCallback(self, data):
            s = data.data   # data is bitfield as string
            self.bit_mapping = {'ls_min': 1, 'enc_seal': 1, 'enc_disp': 1, 'ls_max': 1, 'homed': 1, 'current_pos': 3, 'state': 3}
            vals = ParseBitfield(s, self.bit_mapping)
            self.ls_min = int(vals['ls_min'])
            self.enc_seal = int(vals['enc_seal'])
            self.enc_disp = int(vals['enc_disp'])
            self.ls_max = int(vals['ls_max'])
            self.homed = int(vals['homed'])
            self.current_pos = int(vals['current_pos'])
            self.state = int(vals['state'])

        def AxisCallback(self, data):
            self.x_pos = float(data.data)

        def Print(self):
            s = 'State:' + str(self.state)
            s += '|Homed:' + str(self.homed)
            s += '|(' + str(self.ls_min) + ',' + str(self.enc_seal) + ',' + str(self.enc_disp) + ',' + str(self.ls_max) + ')'
            s += '|Pos:' + str(self.current_pos)
            s += '|X:' + str(self.x_pos)
            return s


    STATE_IDLE = 0
    STATE_RUNNING = 1


    def __init__(self):
        self.homed = False
        self.currentState = self.STATE_IDLE

        self.tray = self.Tray()
        # ROS framework
        self.subs_dsp_status = rospy.Subscriber('/dispenser/status', String, queue_size=10)
        self.subs_seal_status = rospy.Subscriber('/seal/status', String, queue_size=10)
        self.subs_ws_status = rospy.Subscriber('/weight/value', Float64, queue_size=10)

        self.pub_cmd = rospy.Publisher('/stage3/cmd', String, queue_size=10)

    def PrintState(self):
        return self.tray.Print()

    def __del__(self):
        self.subs_dsp_status.unregister()
        self.subs_seal_status.unregister()
        self.subs_ws_status.unregister()
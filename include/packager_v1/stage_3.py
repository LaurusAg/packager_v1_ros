from attr import s
import rospy
from std_msgs.msg import String, Float64
from std_srvs.srv import Trigger, SetBool
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
            self.subs_tray_status = rospy.Subscriber('/stage3/tray/status', String, self.StatusCallback)
            self.subs_tray_axis = rospy.Subscriber('/stage3/tray/axis', Float64, self.AxisCallback)
            
        def __del__(self):
            self.subs_tray_status.unregister()
            self.subs_tray_axis.unregister()

        # Callback for ROS subscriber
        def StatusCallback(self, data):
            s = data.data   # data is bitfield as string
            bit_mapping = {'ls_min': 1, 'enc_seal': 1, 'enc_disp': 1, 'ls_max': 1, 'homed': 1, 'current_pos': 3, 'state': 3}
            vals = ParseBitfield(s, bit_mapping)
            self.ls_min = int(vals['ls_min'])
            self.enc_seal = int(vals['enc_seal'])
            self.enc_disp = int(vals['enc_disp'])
            self.ls_max = int(vals['ls_max'])
            self.homed = int(vals['homed'])
            self.current_pos = int(vals['current_pos'], 2)
            self.state = int(vals['state'])

        def AxisCallback(self, data):
            self.x_pos = float(data.data)

        def Print(self):
            s = 'State:' + str(self.state)
            s += '|Homed:' + str(self.homed)
            s += '|LS:(' + str(self.ls_min) + ',' + str(self.enc_seal) + ',' + str(self.enc_disp) + ',' + str(self.ls_max) + ')'
            s += '|Pos:' + str(self.current_pos)
            s += '|X:' + str(self.x_pos)
            return s

        # Returns a string to publish to the CMD topic
        def Cmd_GoTo(self, pos):
            if( pos == self.POS_RELEASE or 
                pos == self.POS_SEAL or 
                pos == self.POS_DISPENSE or 
                pos == self.POS_ROUTER ):
                return ('M'+str(pos))
                
        def Cmd_MoveX(self, x):
            return ('X'+str(int(x)))

        def Cmd_Home(self):
            return 'H5'

    class Dispenser:
        STATE_IDLE = 0
        STATE_EXIT_LS = 1
        STATE_WAIT_LS = 2
        STATE_ERROR = 3

        def __init__(self):
            self.state = self.STATE_IDLE
            self.ls1 = 1
            self.ls2 = 2
            self.ls3 = 3
            self.ls4 = 4

            # ROS framework
            self.subs_dsp_status = rospy.Subscriber('/stage3/dispenser/status', String, self.StatusCallback)
            
        def __del__(self):
            self.subs_dsp_status.unregister()

        # Callback for ROS subscriber
        def StatusCallback(self, data):
            s = data.data   # data is bitfield as string
            bit_mapping = {'ls1': 1, 'ls2': 1, 'ls3': 1, 'ls4': 1, 'state': 3}
            vals = ParseBitfield(s, bit_mapping)
            self.ls1 = int(vals['ls1'])
            self.ls2 = int(vals['ls2'])
            self.ls3 = int(vals['ls3'])
            self.ls4 = int(vals['ls4'])
            self.state = int(vals['state'], 2)

        def Print(self):
            s = 'State:' + str(self.state)
            s += '|LS:' + '(' + str(self.ls1) + ',' + str(self.ls2) + ',' + str(self.ls3) + ',' + str(self.ls4) + ')'
            return s

        def Cmd_Home(self):
            return 'H3'

        def Cmd_RunOnce(self):
            return 'A3'

    class Seal:
        def __init__(self):
            pass

        # @param latch_state If true, latch servo.
        def Cmd_Latch(self, latch_state):
            return 'S' + ('1' if latch_state==True else '0')

        def Cmd_Press(self, press_state):
            return 'C' + ('1' if press_state==True else '0')

    STATE_IDLE = 0
    STATE_HOMING = 1
    STATE_MOVING = 2
    STATE_DISPENSING = 3
    STATE_SEALING = 4
    STATE_ERROR = 5

    def __init__(self):
        self.state = self.STATE_IDLE
        self.step = 0 # Aux var for performing sequential processes

        self.tray = self.Tray()
        self.dsp = self.Dispenser()
        self.seal = self.Seal()
        # ROS framework
        self.subs_ws_status = rospy.Subscriber('/stage3/weight/value', Float64, queue_size=10)

        self.pub_cmd = rospy.Publisher('/stage3/cmd', String, queue_size=10)

        # ROS services
        self.srv_home = rospy.Service('/stage_3/0_home', Trigger, self.Srv_Home)
        self.srv_dispense = rospy.Service('/stage_3/1_dispense', Trigger, self.Srv_Dispense)
        self.srv_toRouter = rospy.Service('/stage_3/2_toRouter', Trigger, self.Srv_ToRouter)
        self.srv_seal = rospy.Service('/stage_3/3_seal', Trigger, self.Srv_Seal)

    def PrintState(self):
        return self.tray.Print()

    def Srv_Home(self, data):
        self.PublishCmd(self.tray.Cmd_Home())
        rospy.sleep(1)
        self.PublishCmd(self.dsp.Cmd_Home())
        self.state = self.STATE_HOMING
        return [True, ""]

    def Srv_Dispense(self, data):
        if(self.state == self.STATE_IDLE):
            self.state = self.STATE_DISPENSING
            self.flag_dispensing = False
            if(self.tray.current_pos != self.tray.POS_DISPENSE):
                self.PublishCmd(self.tray.Cmd_GoTo(self.tray.POS_DISPENSE))
            return [True, ""]
        else:
            return [False, "Stage 3 is not idle"]

    def Srv_ToRouter(self, data):
        if(self.state == self.STATE_IDLE):
            self.PublishCmd(self.tray.Cmd_GoTo(self.tray.POS_ROUTER))
            self.state = self.STATE_MOVING
            return [True, ""]
        else:
            return [False, "Stage 3 is not idle"]

    def Srv_Seal(self, data):
        if(self.state == self.STATE_IDLE):
            self.state = self.STATE_SEALING
            self.step = 0
            if(self.tray.current_pos != self.tray.POS_RELEASE):
                self.PublishCmd(self.tray.Cmd_GoTo(self.tray.POS_RELEASE))
            return [True, ""]
        else:
            return [False, "Stage 3 is not idle"]

    def Update(self):
        if(self.state == self.STATE_IDLE):
            pass
        
        elif(self.state == self.STATE_HOMING):
            if(self.tray.state == self.tray.STATE_IDLE and self.dsp.state == self.dsp.STATE_IDLE):
                self.state = self.STATE_IDLE    # Homing done
        
        elif(self.state == self.STATE_MOVING):
            if not(self.tray.state == self.tray.STATE_RUNNING):
                self.state = self.STATE_IDLE

        elif(self.state == self.STATE_DISPENSING):
            if(self.flag_dispensing == False and self.tray.current_pos == self.tray.POS_DISPENSE):  # If tray idle, then just arrived to position
                self.PublishCmd(self.dsp.Cmd_RunOnce())
                self.flag_dispensing = True
                rospy.sleep(1)  # Wait for dsp state to change
            elif(self.flag_dispensing and self.dsp.state == self.dsp.STATE_IDLE):
                self.state = self.STATE_IDLE

        elif(self.state == self.STATE_SEALING):
            if(self.step == 0):         # going to release pos
                if(self.tray.state == self.tray.STATE_IDLE and self.tray.current_pos == self.tray.POS_RELEASE):  # If tray idle, then just arrived to position
                    self.PublishCmd(self.seal.Cmd_Latch(True))
                    rospy.sleep(1)  # Wait for servo to move
                    self.PublishCmd(self.tray.Cmd_GoTo(self.tray.POS_SEAL))
                    rospy.sleep(1)  # Wait for servo to move
                    self.step = 1
            elif(self.step == 1):       # going to seal pos
                if(self.tray.x_pos > 140):
                    self.PublishCmd(self.seal.Cmd_Latch(False))
                if(self.tray.state == self.tray.STATE_IDLE and self.tray.current_pos == self.tray.POS_SEAL):
                    # Press twice to ensure seal
                    self.PublishCmd(self.seal.Cmd_Press(True))
                    rospy.sleep(1.5)
                    self.PublishCmd(self.seal.Cmd_Press(False))
                    rospy.sleep(1.5)
                    self.PublishCmd(self.seal.Cmd_Press(True))
                    rospy.sleep(1.5)
                    self.PublishCmd(self.seal.Cmd_Press(False))
                    rospy.sleep(1.5)
                    self.PublishCmd(self.tray.Cmd_GoTo(self.tray.POS_RELEASE))
                    self.step = 2
            elif(self.step == 2):       # going to release pos
                if(self.tray.state == self.tray.STATE_IDLE and self.tray.current_pos == self.tray.POS_RELEASE):
                    self.state = self.STATE_IDLE

            

    def PublishCmd(self, cmd):
        try:
            self.pub_cmd.publish(cmd)
        except:
            rospy.logwarn('Stage3 -> Error sending command to stage 3 edge node.')
            self.state = self.STATE_ERROR

    def __del__(self):
        self.subs_dsp_status.unregister()
        self.subs_seal_status.unregister()
        self.subs_ws_status.unregister()
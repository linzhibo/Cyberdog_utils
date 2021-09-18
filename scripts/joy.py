import sys
sys.path.insert(0, "../")
import pygame
import xbox_keymap 
import grpc
import proto.athena_common_athena_grpc_protos_cyberdog_app_pb2 as cyberdog_app_pb2
import proto.athena_common_athena_grpc_protos_cyberdog_app_pb2_grpc as cyberdog_app_pb2_grpc

xbox =xbox_keymap.xbox()

def print_dict(dictonary):
    print("---"*10)
    for key, value in dictonary.items():
        print(key, value)

class joy_control():
    def __init__(self):

        pygame.init()
        self.clock = pygame.time.Clock()
        pygame.joystick.init()

        self.stub = None
        self.cyberdog_ip = "192.168.3.86"  # to be modified 
        self.deadzone = 0.3
        self.speed = 0.2

        """
        GAIT_TRANS     = 0;  EMERGENCY STOP DON't USE
        GAIT_PASSIVE   = 1;  EMERGENCY STOP DON't USE
        GAIT_KNEEL     = 2;  = get down
        GAIT_STAND_R   = 3;  ??
        GAIT_STAND_B   = 4;  ??
        GAIT_AMBLE     = 5;  ??
        GAIT_WALK      = 6;
        GAIT_SLOW_TROT = 7;
        GAIT_TROT      = 8;
        GAIT_FLYTROT   = 9;
        GAIT_BOUND     = 10;
        GAIT_PRONK     = 11;
        GAIT_DEFAULT   = 99;
        
        Command {
        DEFAULT = 0;
        LOCK = 1;
        CONFIG = 2;
        MANUAL = 3;
        SEMI = 13;
        EXPLOR = 14;
        TRACK = 15;
        }

        """
        # order
        self.available_order = {"TURN_AROUND" : 13,
                                "HI_FIVE"     : 14,
                                "DANCE"       : 15,
                                "WELCOME"     : 16,
                                "TURN_OVER"   : 17,
                                "SIT"         : 18,
                                "BOW"         : 19,
                                "MAX"         : 20,
                                }
        # gait
        self.available_gait = {"GAIT_WALK" :        6,
                               "GAIT_SLOW_TROT" :   7,
                               "GAIT_TROT" :        8,
                               "GAIT_FLYTROT" :     9,
                               "GAIT_BOUND" :       10,
                               "GAIT_PRONK" :       11
                              }
        self.gait_list = list(self.available_gait.keys())
        self.gait_iter = 0
        self.gait_len = len(self.available_gait)
        self.gait_key = {xbox.LB:-1,xbox.RB:1} 
        self.dog_init = 0

        # hidden command  ↑↑↓↓←←→→ no BABA to simplify, [doge] 
        self.secret_command = [xbox.HU]*2 + [xbox.HD]*2 + [xbox.HL]*2 + [xbox.HR]*2 
        self.sc_iter = 0
        self.unlock_high_level = 0 
        self.mad_dog_mode = 0

    def check_sc(self, key):
        ## use secrete key to enter/leave another control mode
        if key == self.secret_command[self.sc_iter]:
            self.sc_iter+=1
            if self.sc_iter == len(self.secret_command):
                self.unlock_high_level = (self.unlock_high_level + 1) % 2
                self.sc_iter = 0
                self.set_cmd_level(self.unlock_high_level )
        elif key == xbox.HU:
            self.sc_iter = 2 if self.sc_iter==2 else 1 ## to deal with the combination like ↑↑↑↓↓←←→→
        else:
            self.sc_iter = 0 

    def check_gait(self, key):
        ## user L R button to iterate cyclically through gait list 
        if key in self.gait_key:
            self.gait_iter = (self.gait_iter + self.gait_key[key]) % self.gait_len
            self.set_gait(self.gait_list[self.gait_iter])

    def send_cmd(self, cmd_dict):
        # print_dict(cmd_dict)
        twist = cyberdog_app_pb2.Twist()
        
        if self.mad_dog_mode:
            twist.linear.y = cmd_dict["axis_0"] * self.speed
            twist.angular.x = cmd_dict["axis_4"] * self.speed 
            twist.angular.y = cmd_dict["axis_1"] * self.speed
            twist.angular.z = cmd_dict["axis_3"] * self.speed
        else:
            twist.linear.x = cmd_dict["axis_1"] * self.speed
            twist.linear.y = cmd_dict["axis_3"] * self.speed 
            twist.angular.z = cmd_dict["axis_0"] * self.speed

        try:
            self.stub.sendAppDecision(cyberdog_app_pb2.Decissage(twist=twist))
        except:
            pass

    def set_gait(self, gait_id = 8):
        request = cyberdog_app_pb2.CheckoutPattern_request()
        request.patternstamped.pattern.gait_pattern = gait_id
        response = self.stub.setPattern(request)
        # print("Execute gait_id " +str(gait_id) +" result:" + str(response.succeed))
        print("Execute gait_id ", list(self.available_gait.keys())[list(self.available_gait.values()).index(gait_id)])
        pygame.time.wait(1000)

    def set_current_speed(self, speed):
        if (speed < 0):
            self.speed = max(0.2, self.speed + speed)
        if (speed > 0):
            self.speed = min(2.0, self.speed + speed)
        print("set speed to: ", speed, " m/s")
    
    def set_cmd_level(self, level):
        print("secret command "+ ["locked, unlocked"][level])
        self.mad_dog_mode = level
        [self.trot(), self.set_gait(gait_id=5)][level]
        
    def set_ai_token(self, vol = 5):
        tr = cyberdog_app_pb2.TokenPass_Request()
        tr.ask = cyberdog_app_pb2.TokenPass_Request.ASK_SET_VOLUME
        tr.vol = vol
        response = self.stub.sendAiToken(tr)
        print("set volume to: ", vol)

    def set_mode(self, mode=0):
        request = cyberdog_app_pb2.CheckoutMode_request()
        request.next_mode.mode.control_mode = mode
        request.timeout = 10
        response = self.stub.setMode(request)
        # print("Execute mode " +str(mode) +" result:" + str(response.succeed))
    
    def set_param(self, cmd_dict):
        param = cyberdog_app_pb2.Parameters()
        param.body_height = cmd_dict["axis_1"] * self.max_speed
        param.gait_height = cmd_dict["axis_3"] * self.max_speed
        result = self.stub.setBodyPara(param)
        # print(result)

    def send_order(self, order_id):
        request = cyberdog_app_pb2.ExtMonOrder_Request()
        request.order.id = order_id
        request.timeput = 10
        print("send order: ", list(self.available_order.keys())[list(self.available_order.values()).index(order_id)])

    def trot(self):
        self.set_gait()

    def stand_up(self):
        self.set_mode(mode=cyberdog_app_pb2.CheckoutMode_request.MANUAL)
        print("Doggo getting up")

    def get_down(self):
        self.set_mode(mode=cyberdog_app_pb2.CheckoutMode_request.DEFAULT)
        print("Doggo going down")

    def dog_start(self):
        if not self.dog_init:
            channel = grpc.insecure_channel(self.cyberdog_ip + ':50051')
            self.stub = cyberdog_app_pb2_grpc.CyberdogAppStub(channel)
            self.set_ai_token()
            self.stand_up()
            self.trot()
            self.dog_init = 1
            print("doggo started with: ", self.cyberdog_ip + ':50051')

    def run(self):
        running = True
        while running:
            self.dog_start()
            joystick_count = pygame.joystick.get_count()
            cmd = {}
            for i in range(joystick_count):
                joystick = pygame.joystick.Joystick(i)
                joystick.init()
                for event in pygame.event.get(): 
                    if event.type == pygame.QUIT: 
                        running = False 
                    if event.type == pygame.JOYBUTTONDOWN:
                        print("Button Pressed")
                        if joystick.get_button(xbox.LB):
                            self.check_gait(xbox.LB)

                        elif joystick.get_button(xbox.RB):
                            self.check_gait(xbox.RB)

                        elif joystick.get_button(xbox.A):
                            self.send_order(self.available_order["WELCOME"]) 

                        elif joystick.get_button(xbox.B):
                            self.send_order(self.available_order["HI_FIVE"]) 

                        elif joystick.get_button(xbox.X):
                            self.send_order(self.available_order["SIT"]) 

                        elif joystick.get_button(xbox.Y):
                            self.send_order(self.available_order["BOW"]) 

                        elif joystick.get_button(xbox.LSI):
                            self.set_current_speed(-0.2)
                            # self.send_order(self.available_order["DANCE"]) 

                        elif joystick.get_button(xbox.RSI):
                            self.set_current_speed(0.2)
                            # self.send_order(self.available_order["TURN_AROUND"]) 

                        elif joystick.get_button(xbox.GB):
                            self.send_order(self.available_order["MAX"]) 

                        elif joystick.get_button(xbox.SB):
                            self.stand_up()

                        elif joystick.get_button(xbox.BB):
                            self.get_down()

                        hats = joystick.get_numhats()
                        for i in range(hats):
                            hat = joystick.get_hat(i)
                            if hat !=[0,0]:
                                self.check_sc(hat)

                    axes = joystick.get_numaxes()
                    for i in range(axes):
                        axis = joystick.get_axis(i)
                        cmd.update({"axis_"+str(i): (-axis if abs(axis) > self.deadzone else 0)})
            self.send_cmd(cmd)
            self.clock.tick(30)
        pygame.quit()

if __name__ == '__main__':
    joy = joy_control()
    joy.run()
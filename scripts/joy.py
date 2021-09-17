import sys
sys.path.insert(0, "../")
import pygame
import grpc
import proto.athena_common_athena_grpc_protos_cyberdog_app_pb2 as cyberdog_app_pb2
import proto.athena_common_athena_grpc_protos_cyberdog_app_pb2_grpc as cyberdog_app_pb2_grpc

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
        self.speed = 2.0

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

        self.gait_iter = 8
        self.gait_len = 12
        self.gait_key = {"button_4":-1,"button5":1} #TODO modify with joystick L and R buttons
        self.dog_init = 0

        # hidden command  ↑↑↓↓←←→→ no BABA to simplify, [doge] 
        self.secret_command = [pygame.K_UP]*2 + [pygame.K_DOWN]*2 + [pygame.K_LEFT]*2 + [pygame.K_RIGHT]*2  #TODO modify with joystick hats: hat_0 (0, 0)
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
        elif key == pygame.K_UP:
            self.sc_iter = 2 if self.sc_iter==2 else 1 #@ to deal with the combination like ↑↑↑↓↓←←→→
        else:
            self.sc_iter = 0 

    def check_gait(self, key):
        ## user L R button to iterate cyclically through gait list 
        if key in self.gait_key:
            self.gait_iter = (self.gait_iter + self.gait_key[key]) % self.gait_len
            self.set_gait(self.gait_iter)

    def send_cmd(self, cmd_dict):
        ## to work on it, still have some problem, some mode cannot reveice twist cmd
        print_dict(cmd_dict)
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

        if(cmd_dict["button_7"]):
            self.stand_up()
        if(cmd_dict["button_6"]):
            self.get_down()
        if(cmd_dict["button_5"]):
            self.set_gait(gait_id = 7)

    def set_gait(self, gait_id = 8):
        request = cyberdog_app_pb2.CheckoutPattern_request()
        request.patternstamped.pattern.gait_pattern = gait_id
        response = self.stub.setPattern(request)
        # print("Execute gait_id " +str(gait_id) +" result:" + str(response.succeed))
        print("Execute gait_id " +str(gait_id))

    def set_current_speed(self, speed):
        self.speed = speed
    
    def set_cmd_level(self, level):
        print("secret command "+ ["locked, unlocked"][level])
        if level:
            self.set_gait(gait_id=5)
            self.mad_dog_mode = 1
        else:
            self.mad_dog_mode = 0
            self.trot()
        
    def set_ai_token(self):
        tr = cyberdog_app_pb2.TokenPass_Request()
        tr.ask = cyberdog_app_pb2.TokenPass_Request.ASK_SET_VOLUME
        tr.vol = 5
        response = self.stub.sendAiToken(tr)
        # print(response)

    def set_mode(self, mode=0):
        request = cyberdog_app_pb2.CheckoutMode_request()
        request.next_mode.mode.control_mode = mode
        request.timeout = 10
        response = self.stub.setMode(request)
        # print("Execute mode " +str(mode) +" result:" + str(response.succeed))
    
    def set_param(self, cmd_dict)
        param = cyberdog_app_pb2.Parameters()
        param.body_height = cmd_dict["axis_1"] * self.max_speed
        param.gait_height = cmd_dict["axis_3"] * self.max_speed
        result = self.stub.setBodyPara(param)
        # print(result)

    def trot(self):
        self.set_gait(gait_id=8)

    def stand_up(self):
        self.set_mode(mode=cyberdog_app_pb2.CheckoutMode_request.MANUAL)

    def get_down(self):
        self.set_mode(mode=cyberdog_app_pb2.CheckoutMode_request.DEFAULT)

    def dog_start(self):
        if not self.dog_init:
            channel = grpc.insecure_channel(self.cyberdog_ip + ':50051')
            self.stub = cyberdog_app_pb2_grpc.CyberdogAppStub(channel)
            self.set_ai_token()
            self.stand_up()
            self.trot()
            self.dog_init = 1

    def run(self):
        running = True
        while running:
            self.dog_start()
            for event in pygame.event.get(): 
                if event.type == pygame.QUIT: 
                    running = False 
            joystick_count = pygame.joystick.get_count()
            cmd = {}
            for i in range(joystick_count):
                joystick = pygame.joystick.Joystick(i)
                joystick.init()
                axes = joystick.get_numaxes()
                for i in range(axes):
                    axis = joystick.get_axis(i)
                    cmd.update({"axis_"+str(i): (-axis if abs(axis) > self.deadzone else 0)})
                buttons = joystick.get_numbuttons()
                for i in range(buttons):
                    button = joystick.get_button(i)
                    cmd.update({"button_"+str(i):button})
                hats = joystick.get_numhats()
                for i in range(hats):
                    hat = joystick.get_hat(i)
                    cmd.update({"hat_"+str(i):hat})
            self.send_cmd(cmd)
            self.clock.tick(30)
        pygame.quit()

if __name__ == '__main__':
    joy = joy_control()
    joy.run()
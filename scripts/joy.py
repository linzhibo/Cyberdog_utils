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
        self.max_speed = 0.2
        self.init_dog = 0

    def send_cmd_vel(self, cmd_dict):
        # print_dict(cmd_dict)
        twist = cyberdog_app_pb2.Twist()
        twist.linear.x = cmd_dict["axis_1"] * self.max_speed
        twist.linear.y = cmd_dict["axis_3"] * self.max_speed 
        twist.angular.z = cmd_dict["axis_0"] * self.max_speed
        self.stub.sendAppDecision(cyberdog_app_pb2.Decissage(twist=twist))

    def set_ai_token(self):
        tr = cyberdog_app_pb2.TokenPass_Request()
        tr.ask = 1
        tr.vol = 1
        response = self.stub.sendAiToken(tr)
        print(response)

    def stand_up(self):
        next_mode = cyberdog_app_pb2.ModeStamped()
        # next_mode.mode = 
        response = self.stub.setMode(
            cyberdog_app_pb2.CheckoutMode_request(
                next_mode=cyberdog_app_pb2.ModeStamped(
                    header=cyberdog_app_pb2.Header(
                        stamp=cyberdog_app_pb2.Timestamp(
                            sec=0,      # seem not need
                            nanosec=0   # seem not need
                        ),
                        frame_id=""     # seem not need
                    ),
                    mode=cyberdog_app_pb2.Mode(
                        control_mode=cyberdog_app_pb2.CheckoutMode_request.MANUAL,
                        mode_type=0     # seem not need
                    )),
                timeout=10))
        succeed_state = False
        for resp in response:
            succeed_state = resp.succeed
            print('Execute Stand up, result:' + str(succeed_state))

    def gait_walk(self):
        # Change gait to walk
        response = self.stub.setPattern(
            cyberdog_app_pb2.CheckoutPattern_request(
                patternstamped=cyberdog_app_pb2.PatternStamped(
                    header=cyberdog_app_pb2.Header(
                        stamp=cyberdog_app_pb2.Timestamp(
                            sec=0,      # seem not need
                            nanosec=0   # seem not need
                        ),
                        frame_id=""     # seem not need
                    ),
                    pattern=cyberdog_app_pb2.Pattern(
                        gait_pattern=cyberdog_app_pb2.Pattern.GAIT_TROT
                    )
                ),
                timeout=10
            )
        )
        for resp in response:
            succeed_state = resp.succeed
            print('Change gait to walk, result:' + str(succeed_state))

    def run(self):
        while True:
            with grpc.insecure_channel(str(self.cyberdog_ip) + ':50051') as channel:
                self.stub = cyberdog_app_pb2_grpc.CyberdogAppStub(channel)
                if not self.init_dog:
                    # self.set_ai_token()
                    self.stand_up()
                    self.gait_walk()
                    self.init_dog = 1
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
                        cmd.update({"hats_"+str(i):hats})
                self.send_cmd_vel(cmd)
                self.clock.tick(20)
        pygame.quit()

if __name__ == '__main__':
    joy = joy_control()
    joy.run()
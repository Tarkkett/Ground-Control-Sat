import pygame
import threading
from time import sleep

class GamepadCtrl:


    def __init__(self):
        pygame.init()
        self.joysticks = {}
        print(self.joysticks)
        threading.Thread(target=self.run, daemon=True, name="GamepadThread").start()
        self.isControlMode = False

    def run(self):
        
        self.threading = True
        while self.threading:
            try:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        self.threading = False

                    if event.type == pygame.JOYBUTTONDOWN:
                        print("Joystick button pressed.")
                        self.isControlMode = not self.isControlMode
                        if event.button == 0:
                            joystick =self.joysticks[event.instance_id]
                            if joystick.rumble(0, 0.7, 500):
                                print(f"Rumble effect played on joystick {event.instance_id}")

                    if event.type == pygame.JOYDEVICEADDED:

                        joy = pygame.joystick.Joystick(event.device_index)
                        self.joysticks[joy.get_instance_id()] = joy
                        if self.threading == True:
                            print(f"Joystick {joy.get_instance_id()} connencted")
                            print(joy.get_instance_id())

                    if event.type == pygame.JOYDEVICEREMOVED:
                        del self.joysticks[event.instance_id]
                        #print(f"Joystick {event.instance_id} disconnected")

                # For each joystick:
                for joystick in self.joysticks.values():

                    #axes = joystick.get_numaxes()
                    if self.threading:
                        self.x = self.map_value(joystick.get_axis(0))     
                        self.y = self.map_value(-joystick.get_axis(1))
                        self.lockLX = self.map_value(joystick.get_axis(0))
                        self.lockLY = self.map_value(-joystick.get_axis(1))
                        self.lockRX = self.map_value(joystick.get_axis(2))
                        self.lockRY = self.map_value(-joystick.get_axis(3))
                        
                        sleep(0.1)
                            #print(f"Axis {i} value: {axis:>6.3f}")
            except Exception as e:
                pass
                #print(str(e) + " lol")
    def map_value(self, value):
        return (value + 1) * 50



if __name__ == "__main__":
    GamepadCtrl()
    pygame.quit()
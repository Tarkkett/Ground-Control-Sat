import pygame
import threading




class GamepadCtrl:

    def __init__(self):
        pygame.init()
        self.joysticks = {}
        self.threading = True
        threading.Thread(target=self.run, daemon=True, name="GamepadThread").start()

    def run(self):
        self.threading = True
        
        while self.threading:
            try:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        self.threading = False

                    if event.type == pygame.JOYDEVICEADDED:
                        # This event will be generated when the program starts for every
                        # joystick, filling up the list without needing to create them manually.
                        joy = pygame.joystick.Joystick(event.device_index)
                        self.joysticks[joy.get_instance_id()] = joy
                        print(f"Joystick {joy.get_instance_id()} connencted")
                        print(joy.get_instance_id())

                    if event.type == pygame.JOYDEVICEREMOVED:
                        del self.joysticks[event.instance_id]
                        print(f"Joystick {event.instance_id} disconnected")

                # For each joystick:
                for joystick in self.joysticks.values():

                    axes = joystick.get_numaxes()

                    for i in range(axes):
                        axis = joystick.get_axis(i)
                        print(f"Axis {i} value: {axis:>6.3f}")
            except Exception as e:
                print(e + " lol")



if __name__ == "__main__":
    GamepadCtrl()
    pygame.quit()
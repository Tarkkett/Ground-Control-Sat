from GUI_Master import RootGUI, ComGUI
from Serial_Com_ctrl import SerialCtrl
from Data_com_ctrl import DataMaster
from Gamepad import GamepadCtrl

MySerial = SerialCtrl()
MyData = DataMaster()
Gamepad = GamepadCtrl()
RootMaster = RootGUI(MySerial, MyData, Gamepad)
ComMaster = ComGUI(RootMaster.root, MySerial, MyData, Gamepad)


RootMaster.root.mainloop()

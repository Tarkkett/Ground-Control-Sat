from GUI_Master import RootGUI, ComGUI
from Serial_Com_ctrl import SerialCtrl
from Data_com_ctrl import DataMaster

MySerial = SerialCtrl()
MyData = DataMaster()
RootMaster = RootGUI()
ComMaster = ComGUI(RootMaster.root, MySerial, MyData)

RootMaster.root.mainloop()

class DataMaster():
    def __init__(self):
        self.sync = "#?#\n"
        self.sync_ok = "!"
        self.StartStream = "#A#\n"
        self.StopStream = "#S#\n"
        self.StartController = "#C#\n"
        self.StopController = "#P#\n"
        self.SyncChannel = 0
        self.control_x = 0
        self.control_y = 0
        self.msg = []

        self.data_ok = False
        print("Set to false!")

        self.XData = []
        self.YData = []
        self.FunctionMaster = ["RawData",
                               "Voltage"]

    def DecodeMsg(self):
        print("Decode 1")
        print(self.RowMsg)
        temp = self.RowMsg.decode('utf-8')
        if len(temp) > 0:
            if "#" in temp:
                self.msg = temp.split("#")
                print(f"Before removing index: {self.msg}")
                del self.msg[0]
                #del self.msg[1]
                print(f"After removing index: {self.msg}")
                if self.msg[0] in "D":
                    self.data_ok = True
                    print("OK!")
                    self.messageLength = 0
                    self.messageLengthCheck = 0
                    
                    del self.msg[0]
                    del self.msg[len(self.msg)-1]
                    
                    self.messageLength = float(self.msg[len(self.msg)-1])
                    print(F"Length: {self.messageLength}")
                    del self.msg[len(self.msg)-1]
                else:
                    self.data_ok = False
                    print("Changed to false!")

                
                    

    def GenChannels(self):
        self.Channels = [f"Ch{ch}" for ch in range(self.SyncChannel)]
    
    def BuildYData(self):
        for _ in range(self.SyncChannel):
            self.YData.append([])

    def ClearData(self):
        self.RowMsg=""
        self.msg = []
        self.YData = []

if __name__ == "__main__":
    DataMaster()
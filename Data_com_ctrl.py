class DataMaster():
    def __init__(self):
        self.sync = "#?#\n"
        self.sync_ok = "!"
        self.StartStream = "#A#\n"
        self.StopStream = "#S#\n"
        self.SyncChannel = 0
        self.msg = []

        self.XData = []
        self.YData = []

    def DecodeMsg(self):
        temp = self.RowMsg.decode('utf-8')
        if len(temp) > 0:
            if "#" in temp:
                self.msg = temp.split("#")
                print(f"Before removing index: {self.msg}")
                del self.msg[0]
                print(f"After removing index: {self.msg}")

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
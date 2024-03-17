class DataMaster():
    def __init__(self):
        self.sync = "#?#\n"
        self.sync_ok = "!"
        self.StartStream = "#A#\n"
        self.StopStream = "#S#\n"
        self.SyncChannel = 0
    def DecodeMsg(self):
        temp = self.RowMsg.decode('utf-8')
        if len(temp) > 0:
            if "#" in temp:
                self.msg = temp.split("#")
                print(self.msg)
                del self.msg[0]
import serial.tools.list_ports
import time

class SerialCtrl():
    def __init__(self):
        self.com_list = []
        self.sync_cnt = 200

    def getCOMList(self):
        ports = serial.tools.list_ports.comports()
        self.com_list = [com[0] for com in ports]
        self.com_list.insert(0, "-")

    def SerialOpen(self, gui):
        try:
            self.ser.is_open
        except:
            PORT = gui.clicked_com.get()
            BAUD = gui.clicked_baudr.get()
            self.ser = serial.Serial()
            self.ser.baudrate = BAUD
            self.ser.port = PORT
            self.ser.timeout = 0.1
        try:
            if self.ser.is_open:
                print("COM Already Open")
                self.ser.status = True
            else:
                PORT = gui.clicked_com.get()
                BAUD = gui.clicked_baudr.get()
                self.ser = serial.Serial()
                self.ser.baudrate = BAUD
                self.ser.port = PORT
                self.ser.timeout = 0.1
                self.ser.open()
                self.ser.status = True
        except:
            self.ser.status = False

    def SerialClose(self, gui):
        try:
            self.ser.is_open
            self.ser.close()
            self.ser.status = False
        except:
            self.ser.status = False

    def SerialSync(self, gui):
        self.threading = True
        cnt=0
        while self.threading:
            try:
                self.ser.write(gui.data.sync.encode())
                #print("Sent Start MSG!")
                gui.conn.sync_status["text"] = "..Sync.."
                gui.conn.sync_status["fg"] = "orange"
                gui.data.RowMsg = self.ser.readline()
                #print(f"RowMsg: {gui.data.RowMsg}")
                gui.data.DecodeMsg()

                if gui.data.sync_ok in gui.data.msg[0]:
                    
                    if int(gui.data.msg[1]) > 0:
                        gui.conn.btn_start_stream["state"] = "active"
                        gui.conn.btn_add_chart["state"] = "active"
                        gui.conn.btn_remove_chart["state"] = "active"
                        gui.conn.save_check["state"] = "active"
                        gui.conn.sync_status["text"] = "OK"
                        gui.conn.sync_status["fg"] = "green"
                        gui.conn.ch_status["text"] = gui.data.msg[1]
                        gui.conn.ch_status["fg"] = "green"
                        gui.data.SyncChannel = int(gui.data.msg[1])
                        gui.data.GenChannels()
                        gui.data.BuildYData()
                        print(gui.data.Channels, gui.data.YData)
                        self.threading = False
                        break
                if self.threading == False:
                    break
            except Exception as e:
                print(e)
                #print("Common")

            cnt += 1
            if self.threading == False:
                break

            if cnt > self.sync_cnt:
                cnt = 0
                gui.conn.sync_status["text"] = "Failed"
                gui.conn.sync_status["fg"] = "red"
                if self.threading == False:
                    break
                time.sleep(0.5)

    def SerialDataStream(self, gui):
        self.threading = True
        while self.threading:
            try:
                self.ser.write(gui.data.StartStream.encode())
                gui.data.RowMsg = self.ser.readline()
                gui.data.DecodeMsg()
            except Exception as e:
                print(e)
            
    def StopStream(self, gui):
        try:
            self.ser.write(gui.data.StopStream.encode())
            gui.data.data_ok = False
        except Exception as e:
            gui.data.data_ok = False
            print(e)

if __name__ == "__main__":
    SerialCtrl()
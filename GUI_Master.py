from tkinter import *
from tkinter import messagebox
from tkinter import ttk
from tkinter.ttk import Progressbar, Style
import tkinter.font as TkFont
import threading
from time import sleep
import matplotlib.pyplot as plt
import tkintermapview
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import socket


from functools import partial

HOST = "127.0.0.1"
PORT = 65432

class RootGUI:
    def __init__(self, serial, data, gamepad):
        self.root = Tk()

        self.root.title("MagnifiCanSatGround Station Comms!")
        self.root.geometry("360x120")
        self.root.config(bg="grey")
        self.root.iconbitmap("Assets/icon.ico")
        self.gamepad = gamepad

        self.serial = serial
        self.data = data

        self.root.protocol("WM_DELETE_WINDOW", self.close_window)

    def close_window(self):
        
        print("Closing the main window now!")
        
        self.gamepad.theading = False
        self.serial.threading = False
        self.root.destroy()
        self.serial.SerialClose(self)
        
        sleep(1)
        
        
        

class ComGUI():
    def __init__(self, root, serial, data, gamepad):

        self.root = root
        self.serial = serial
        self.data = data
        self.gamepad = gamepad
        self.mainFont = TkFont.Font(family="Calibri",size=10,weight="bold")

        
        self.frame = LabelFrame(root, text="Comms Manager", padx=5, pady=5, bg="grey")
        self.label_com = Label(self.frame, text="Available port(s): ", bg="grey", width=15, anchor="w")
        self.label_baudr = Label(self.frame, text="Baud Rate: ", bg="grey", width=15, anchor="w")


        self.ComOptionMenu()
        self.BaudOptionMenu()

        self.btn_refresh = Button(self.frame, text="Refresh", width=10, command=self.com_refresh)
        self.btn_connect = Button(self.frame, text="Connect!", state=DISABLED, width=10, command=self.serial_connect)

        self.publish()

    def ComOptionMenu(self):
        self.serial.getCOMList()
        self.clicked_com = StringVar()
        self.clicked_com.set(self.serial.com_list[0])
        self.drop_com = OptionMenu(self.frame, self.clicked_com, *self.serial.com_list, command=self.connect_ctrl)
        self.drop_com.config(width=10)
    
    def BaudOptionMenu(self):
        self.clicked_baudr = StringVar()
        bds = ["-", "300", "9600", "57600", "115200"]
        self.clicked_baudr.set(bds[0])
        self.drop_baudr = OptionMenu(self.frame, self.clicked_baudr, *bds, command=self.connect_ctrl)
        self.drop_baudr.config(width=10)

    def publish(self):
        #Con frame
        self.frame.grid(row=0, column=0, rowspan=3, columnspan=3, padx=5, pady=5, ipady=3, sticky=NW)

        self.label_com.grid(column=1, row=2)
        self.drop_com.grid(column=2, row=2)
        self.label_baudr.grid(column=1, row=3)
        self.drop_baudr.grid(column=2, row=3)
        self.btn_refresh.grid(column=3, row=2, padx=5)
        self.btn_connect.grid(column=3, row=3, padx=5)

    def connect_ctrl(self, widget):

        print("Connect")
        if "-" in self.clicked_com.get() or "-" in self.clicked_baudr.get():
            self.btn_connect["state"] = "disable"
        else:
            self.btn_connect["state"] = "active"

    def serial_connect(self):
        if self.btn_connect["text"] in "Connect!":
            self.serial.SerialOpen(self)
            if self.serial.ser.status:
                self.btn_connect["text"] = "Disconnect"
                self.btn_refresh["state"] = "disable"
                self.drop_baudr["state"] = "disable"
                self.drop_com["state"] = "disable"
                InfoMsg = f"Connection success!"
                messagebox.showinfo("showinfo", InfoMsg)
                
                #Start Comms, controller, map and logger
                
                self.controller = GamepadGUI(self.root, self.gamepad, self.data, self.serial)
                self.logger = LoggerGUI(self.root, self.data, self.serial)
                self.map = MapGUI(self.root, self.mainFont, self.data)
                self.conn = ConnGUI(self.root, self.serial, self.data, self.mainFont, self.logger)
                self.controls = ControlsGUI(self.root, self.serial, self.data)

                self.serial.t1 = threading.Thread(
                    target = self.serial.SerialSync, args = (self,), daemon=True
                )
                self.serial.t1.start()

            else:
                ErrorMsg = f"FATAL Error trying to connect in the last step! "
                messagebox.showerror("showerror", ErrorMsg)
        else:
            self.gamepad.threading = False
            self.controller.threading = False
            self.conn.stop_stream()
            self.serial.threading = False

            self.map.threading = False
            self.logger.threading = False
            self.serial.close_file()
            self.serial.SerialClose(self)
            self.logger.LoggerGUIClose()
            self.conn.ConnGUIClose()
            self.controller.GamepadGUIClose()
            self.map.MapGUIClose()
            self.controls.ControlsGUIClose()
            
            
            self.data.ClearData()
            
            InfoMsg = f"Connection is now closed! "
            messagebox.showwarning("Warning!", InfoMsg)
            self.btn_connect["text"] = "Connect!"
            self.btn_refresh["state"] = "active"
            self.drop_baudr["state"] = "active"
            self.drop_com["state"] = "active"



    def com_refresh(self):
        self.drop_com.destroy()
        self.ComOptionMenu()
        self.drop_com.grid(column=2, row=2)
        logic = []
        self.connect_ctrl(logic)

class LoggerGUI():
    def __init__(self, root, data, serial):
        self.root = root
        self.data = data
        self.serial = serial

        self.frame = LabelFrame(root, text="Data Logger", padx=5, pady=5, bg="gray")
        self.dataCanvas = Canvas(self.frame, width=800, height=265, background="black", highlightbackground="black")
        self.vsb = Scrollbar(self.frame, orient='vertical', command=self.dataCanvas.yview)

        self.dataFrame = Frame(self.dataCanvas, bg="black", pady=5)
        self.dataCanvas.create_window((5,10),window=self.dataFrame,anchor='nw')
        

        self.threading = True

        self.loggerThread = threading.Thread(target=self.PullLog, name="LogThread", daemon=True)
        self.loggerThread.start()
        self.LoggerGUIOpen()
    
    def PullLog(self):
        while self.threading:
            if self.data.data_ok:
                
                Label(master=self.dataFrame, text=f">>{self.data.parsedMsg}", foreground="lime", background="black", pady=3).pack()
                if len(self.dataFrame.winfo_children()) > 100:
                    self.dataFrame.winfo_children()[0].destroy()
                #print(self.data.parsedMsg)
                self.dataCanvas.yview_moveto(1)
            sleep(0.05)

            self.dataCanvas.config(scrollregion=self.dataCanvas.bbox("all"))
                
    def LoggerGUIOpen(self):
        
        #messagebox.showwarning("Warning!", "Should place!")
        self.frame.grid(row=3, column=2, rowspan=1, columnspan=5, padx=5, pady=5)
        self.dataCanvas.grid(row=0, column=0)
        self.vsb.grid(row=0, column=2, rowspan=100, sticky='ns')

        self.dataCanvas.config(yscrollcommand = self.vsb.set)

    def LoggerGUIClose(self):
        for widget in self.frame.winfo_children():
            widget.destroy()
        self.frame.destroy()
        self.root.geometry("360x120")

class MapGUI():
    def __init__(self, root, mainFont, data):
        self.root = root
        self.data = data
        self.threading = True

        self.mapSizeX = 60
        self.mapSizeY = 60

        self.address = "Vilnius"

        self.font = mainFont
        self.markerList = []

        self.currentX = 0
        self.currentY = 0
        self.tuple = (0.0, 0.0)
        
        

        self.frame = LabelFrame(self.root, text="Map frame", padx=5, pady=5, bg="gray", relief="ridge")
        
        self.map_widget = tkintermapview.TkinterMapView(self.frame, width=800, height=380, corner_radius=20)
        self.map_widget.set_tile_server("https://mt0.google.com/vt/lyrs=m&hl=en&x={x}&y={y}&z={z}&s=Ga", max_zoom=22)
        self.zoomLevel = Label(self.frame, text= f"Zoom level: {self.map_widget.last_zoom}x", padx=5, pady=5, bg="gray", font=self.font)

        self.MapGUIOpen()

        self.mapThread = threading.Thread(target = self.UpdateMap, name="MapThread", daemon=True)
        self.mapThread.start()
        print("Started thread!")

        #messagebox.showwarning("Warning!", "Should place!")
        

    def MapGUIOpen(self):
        self.root.geometry("1280x720")

        #Map frame
        self.frame.grid(row=0, column=7, padx=5, pady=5, rowspan=4, sticky=NW)
        self.zoomLevel.grid(row=0, column=1, rowspan=4)
        self.map_widget.grid(row=0, column=0)
        self.map_widget.set_address("Moletu aerodromas", marker=True)
        self.map_widget.set_zoom(18)
        self.initialMarker = self.map_widget.set_marker(54.687157, 25.279652)
        

    def UpdateMap(self):
        
        self.markerList.append(self.initialMarker.position)
        while self.threading:
            sleep(1)
            if self.data.data_ok:
                if self.data.parsedMsg[0] != 0.0:
                
                    self.currentX = float(self.data.parsedMsg[0])
                    self.currentY = float(self.data.parsedMsg[1])
                    self.tuple = (self.currentX, self.currentY)
                    print(self.tuple)
                        
                    self.markerList.append(self.tuple)
                    if len(self.markerList) > 20:
                        del self.markerList[0]
                    print(len(self.markerList))
                    self.map_widget.set_path(self.markerList)
                    self.zoomLevel["text"] = f"Zoom level: {self.map_widget.zoom}x"
                    
                    #self.map_widget.update()
            #self.infoLabel["text"] = f"Info: {self.map_widget.info}"
    
    def MapGUIClose(self):
        for widget in self.frame.winfo_children():
            widget.destroy()
        self.frame.destroy()


class GamepadGUI():
    def __init__(self, root, gamepad, data, serial):
        self.data = data
        self.root = root
        self.gamepad = gamepad
        self.serial = serial
        self.threading = True
        monitorThread = threading.Thread(target=self.UpdateControllerData, name="Gamepad Monitor", daemon=True)
        

        self.s = Style(self.root)

        self.s.layout("LabeledProgressbar",
            [('LabeledProgressbar.trough',
            {'children': [('LabeledProgressbar.pbar',
                            {'side': 'left', 'sticky': 'ns'}),
                            ("LabeledProgressbar.label",
                            {"sticky": ""})],
            'sticky': 'nswe'})])

        self.frame = LabelFrame(root, text="Controller data", padx=5, pady=5, bg="gray")
        self.barLeftX = Progressbar(self.frame, length=250, style="LabeledProgressbar", orient="vertical")
        self.barLeftY = Progressbar(self.frame, length=250, style="LabeledProgressbar", orient="vertical")
        self.barRightX = Progressbar(self.frame, length=250, style="LabeledProgressbar", orient="vertical")
        self.barRightY = Progressbar(self.frame, length=250, style="LabeledProgressbar", orient="vertical")
        self.labelLeftX = Label(self.frame, text="LX", bg="gray")
        self.labelLeftY = Label(self.frame, text="LY", bg="gray")
        self.labelRightX = Label(self.frame, text="RX", bg="gray")
        self.labelRightY = Label(self.frame, text="RY", bg="gray")
        self.minusOne = Label(self.frame, text="-1", bg="gray")
        self.plusOne = Label(self.frame, text="+1", bg="gray")
        self.zero = Label(self.frame, text="0", bg="gray")
        

        self.GamepadGUIOpen()
        monitorThread.start()
    
    def UpdateControllerData(self):
        
        while self.threading and len(self.gamepad.joysticks) > 0 and self.gamepad.threading ==True:
            try:
                sleep(0.1)
                if self.gamepad.isControlMode:
                    self.data.control_x = self.gamepad.x
                    self.data.control_y = self.gamepad.y
                    self.controlRequest = f"#C#{self.data.control_x}#{self.data.control_y}#\n"
                    
                    self.serial.ser.write(self.controlRequest.encode())
                else:
                    self.serial.ser.write(self.data.GPSModeCommand.encode())
                
                if self.gamepad.isBuzzing:
                    self.serial.ser.write(self.data.BuzzCommand.encode())
                else:
                    self.serial.ser.write(self.data.StopBuzzCommand.encode())
                    

                self.barLeftX["value"] = int(self.gamepad.lockLX)
                self.barLeftY["value"] = int(self.gamepad.lockLY)
                self.barRightX["value"] = int(self.gamepad.lockRX)
                self.barRightY["value"] = int(self.gamepad.lockRY)
                self.s.configure("LabeledProgressbar", text="".format(int(self.gamepad.lockLX)))
                self.s.configure("LabeledProgressbar", text="".format(int(self.gamepad.lockLY)))
                self.s.configure("LabeledProgressbar", text="".format(int(self.gamepad.lockRX)))
                self.s.configure("LabeledProgressbar", text="".format(int(self.gamepad.lockRY)))
                self.root.update()
                #print(self.gamepad.isControlMode)
            except Exception as e:
                print(e)

    def GamepadGUIOpen(self):
        #Gamepad frame
        self.frame.grid(row=3,column=0, rowspan=1, columnspan=2, padx=5, pady=5, sticky=NW)
        self.barLeftX.grid(column=1, row= 0, padx= 5, rowspan=3)
        self.barLeftY.grid(column=2, row= 0, padx= 5, rowspan=3)
        self.barRightX.grid(column=3, row= 0, padx= 5, rowspan=3)
        self.barRightY.grid(column=4, row= 0, padx= 5, rowspan=3)
        self.labelLeftX.grid(column=1, row=4)
        self.labelLeftY.grid(column=2, row=4)
        self.labelRightX.grid(column=3, row=4)
        self.labelRightY.grid(column=4, row=4)
        self.minusOne.grid(column=5, row=2)
        self.plusOne.grid(column=5, row=0)
        self.zero.grid(column=5, row=1)
    
    def GamepadGUIClose(self):
        for widget in self.frame.winfo_children():
            widget.destroy()
        self.frame.destroy()

class ControlsGUI():
    def __init__(self, root, serial, data):

        self.root = root
        self.serial = serial
        self.data = data

        self.frame = LabelFrame(self.root, text="Server manager", bg="gray", padx=5, pady=5)
        self.portLocName = Label(self.frame, text="Localizer port:", bg="gray")
        self.portWebName = Label(self.frame, text="Visualizer port:", bg="gray")
        self.portLocButton = Button(self.frame, text="Prisijungti!", bg="gray", command=self.button_mode_loc)
        self.portWebButton = Button(self.frame, text="Prisijungti!", bg="gray", command=self.button_mode_web)
        self.locPortEntry = Entry(self.frame)
        self.webPortEntry = Entry(self.frame)

        self.is_loc_on = False
        self.is_web_on = False

        self.OpenControlsFrame()

    def OpenControlsFrame(self):
        self.frame.grid(row=0, column=30, padx=5, pady=5, sticky=NW)
        self.portLocName.grid(row=0, column=0)
        self.portWebName.grid(row=1, column=0)

        self.locPortEntry.grid(row=0, column=1)
        self.webPortEntry.grid(row=1, column=1)

        self.portLocButton.grid(row=0, column=2, padx=5, pady=5)
        self.portWebButton.grid(row=1, column=2, padx=5, pady=5)

        self.webThread = threading.Thread(target=self.SendWebSerialData, name="Web", daemon=True)
        self.locThread = threading.Thread(target=self.SendLocSerialData, name="Localizer", daemon=True)

        self.webThreading = False

    def ControlsGUIClose(self):
        for widget in self.frame.winfo_children():
            widget.destroy()
        self.frame.destroy()
        


    def button_mode_web(self):

        if self.is_web_on:

            try:
                self.serial.web_sock.close()
                self.portWebButton["text"] = "Prisijungti!"
                self.is_web_on = False
                self.webThreading = False
            except Exception as e:
                messagebox.showerror("Klaida išjungiant thread!", e)
        else:
            self.webPortEntered = self.webPortEntry.get()
            if self.webPortEntered:
                if self.webPortEntered.isdigit():
                    try:
                        self.serial.web_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                        self.serial.web_sock.connect((HOST, int(self.webPortEntry.get())))
                        self.portWebButton["text"] = "Atsijungti!"
                        self.is_web_on = True
                        self.webThreading = True
                        if not self.webThread.is_alive():
                            self.webThread.start()
                        print("reached true section")
                        
                    except Exception as e:
                        messagebox.showerror("Klaida paleidžiant thread!", e)
                else:
                    print("NAN")
            else:
                print("Empty field")

    def button_mode_loc(self):

        
        
        if self.is_loc_on:
            print("OFF")
            try:
                self.serial.loc_sock.close()
                self.portLocButton["text"] = "Prisijungti!"
                self.is_loc_on = False
                self.localizerThreading = False
            except Exception as e:
                messagebox.showerror("Klaida išjungiant thread!", e)
        else:
            self.locPortEntered = self.locPortEntry.get()
            if self.locPortEntered:
                if self.locPortEntered.isdigit():
                    try:
                        self.serial.loc_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                        self.serial.loc_sock.connect((self.serial.host, int(self.locPortEntry.get())))
                        self.portLocButton["text"] = "Atsijungti!"
                        self.is_loc_on = True
                        self.localizerThreading = True
                        if not self.locThread.is_alive():
                            self.locThread.start()
                        
                    except Exception as e:
                        messagebox.showerror("Klaida paleidžiant thread!", e)
                else:
                    print("NAN")
            else:
                print("Empty field")

    def SendLocSerialData(self):
        while self.localizerThreading:
            sleep(0.01)
            try:
                if self.data.data_ok:
                    self.rotation = f"({self.data.parsedMsg[2]},{self.data.parsedMsg[3]},{self.data.parsedMsg[4]},{self.data.parsedMsg[0]},{self.data.parsedMsg[1]},{self.data.parsedMsg[8]}, {self.data.parsedMsg[12]})"
                    self.serial.loc_sock.sendall(self.rotation.encode("utf-8"))
                    response = self.serial.loc_sock.recv(1024).decode("utf-8")
                    print(response)
                    print(self.rotation)

            except Exception as e:
                try: 
                    self.serial.loc_sock.close()
                except Exception as x:
                    messagebox.showerror("Klaida!", x)
                    
                messagebox.showerror("Klaida!", e)
                self.localizerThreading = False
                self.is_loc_on = False
                self.portLocButton["text"] = "Prisijungti!"

    def SendWebSerialData(self):
        while self.webThreading == True:
            sleep(1)
            
            if self.data.data_ok:
                print("sending Hello") #Temp, pressure, humidity, servo x, servo y, v speed, altitude, g, uvb
                self.fullMsg = f"({self.data.parsedMsg[5]},{self.data.parsedMsg[7]},{self.data.parsedMsg[6]},{self.data.parsedMsg[10]},{self.data.parsedMsg[11]},{self.data.parsedMsg[12]},{self.data.parsedMsg[9]},{self.data.parsedMsg[14]},{self.data.parsedMsg[13]})"
                # f("{self.data.parsedMsg[6]}")
                try:
                    self.serial.web_sock.sendall(self.fullMsg.encode("utf-8"))

                    print(self.fullMsg)
                except Exception as e:
                    print(e)
            else:
                print("Data not ok")



class ConnGUI():
    def __init__(self, root, serial, data, mainFont, logger):
        self.root = root
        self.serial = serial
        self.data = data
        self.mainFont = mainFont
        self.logger = logger

        self.frame = LabelFrame(root, text="Connection manager", padx=5, pady=5, bg='grey')

        self.sync_label = Label(self.frame, text="Sync status: ", bg="gray", width=15, anchor="w")
        self.sync_status = Label(self.frame, text="..Sync..", bg="gray", fg="orange", width=5)

        self.ch_label = Label(self.frame, text="Active Channels: ", bg="gray", width=15, anchor="w")
        self.ch_status = Label(self.frame, text="...", bg="gray", fg="orange", width=5)

        self.btn_start_stream = Button(self.frame, text="Launch", state=DISABLED, width=5, command=self.start_stream)
        self.btn_stop_stream = Button(self.frame, text="Stop", state=DISABLED, width=5, command=self.stop_stream)

        #Create add/remove chart button objects not needed I think(O gal pridesiu veliau)
        self.btn_add_chart = Button(self.frame, text="+", font=self.mainFont, state="disabled", width=5, bg="grey", fg="orange", command=self.new_chart)
        self.btn_remove_chart = Button(self.frame, text="-", font=self.mainFont , state="disabled", width=5, bg="grey", fg="orange", command=self.remove_chart)

        self.save = False
        self.SaveVar = IntVar()

        self.save_check = Checkbutton(self.frame, text="Save data", variable=self.SaveVar, onvalue=1, offvalue=0, bg="grey", state="disabled", command=self.save_data)

        self.separator = ttk.Separator(self.frame, orient = 'vertical')

        self.pady = 11
        self.padx = 20

        self.ConnGUIOpen()
        self.chartMaster = DisGUI(self.root, self.serial, self.data, self.logger)

    def ConnGUIOpen(self):
        self.root.geometry('2560x420')
        self.root.wm_attributes("-topmost", True)
        # self.root.wm_attributes("-y", self.root.winfo_screenheight() - self.root.winfo_reqheight())

        # Conn frame
        self.frame.grid(row=0,column=3, rowspan=3, columnspan=3, padx=5, pady=5, sticky=NW)
        self.sync_label.grid(column=1, row=1)
        self.sync_status.grid(column=2, row=1)

        self.ch_label.grid(column=1, row=2)
        self.ch_status.grid(column=2, row=2, pady=self.pady)

        self.btn_start_stream.grid(column=3, row=1, padx=self.padx)
        self.btn_stop_stream.grid(column=3, row=2, padx=self.padx)
        
        #Button for adding/removing charts addition to grid not needed for now
        self.btn_add_chart.grid(column=4, row=1, padx=self.padx)
        self.btn_remove_chart.grid(column=5, row=1, padx=self.padx)

        self.save_check.grid(column=4, row=2, columnspan=2)

        self.separator.place(relx=0.6,rely=0,relwidth=0.001, relheight=1)
    
    def ConnGUIClose(self):
        for widget in self.frame.winfo_children():
            widget.destroy()
        self.frame.destroy()
        self.root.geometry("360x120")
    
    def start_stream(self):
        self.btn_start_stream["state"]="disabled"
        self.btn_stop_stream["state"]="active"
        self.serial.t1 = threading.Thread(target=self.serial.SerialDataStream, args=(self,), daemon=True)
        self.serial.t1.start()

    def stop_stream(self):
        self.serial.StopStream(self)
        self.btn_start_stream["state"]="active"
        self.btn_stop_stream["state"]="disabled"
        self.serial.threading = False
        sleep(1)
        self.data.data_ok = False

    def save_data(self):
        self.threading = True
        if self.SaveVar.get() == 1:
            self.serial.generate_file()
            t1 = threading.Thread(target=self.writeToFile, daemon=True)
            t1.start()
            self.threading = True
        else:
            self.threading = False
            self.serial.close_file()

            print("Closed")

    def writeToFile(self):


        while self.threading: 

            if self.data.data_ok:
                
                try:
                    self.serial.file.write(str(self.data.parsedMsg) + "\n")

                except Exception as e:
                    print(e)
                    sleep(2)

            sleep(0.01)



    def new_chart(self):
        self.chartMaster.AddChannelMaster()

    def remove_chart(self):
        try:

            if len(self.chartMaster.frames) > 0:
                totalFrame = len(self.chartMaster.frames)-1
                self.chartMaster.frames[totalFrame].destroy()
                self.chartMaster.frames.pop()
                self.chartMaster.figs.pop()

                self.chartMaster.ControlFrames[totalFrame][0].destroy()
                self.chartMaster.ControlFrames.pop()

                self.chartMaster.ChannelFrame[totalFrame][0].destroy()
                self.chartMaster.ChannelFrame.pop()

                self.chartMaster.ViewVar.pop()
                self.chartMaster.OptionVar.pop()
                self.chartMaster.FuncVar.pop()

                self.chartMaster.AdjustRootFrame()
        except:
            pass

class DisGUI():
    def __init__(self, root, serial, data, logger):
        self.logger = logger
        self.root = root
        self.serial = serial
        self.data = data

        self.frames = []
        self.framesCol = 0
        self.framesRow = 4
        self.totalFrames = 0

        self.figs = []
        self.ControlFrames = []
        self.ChannelFrame = []
        self.ViewVar = []
        self.FuncVar = []
        self.OptionVar = []

    def AddChannelMaster(self):
        self.AddMasterFrame()
        self.AdjustRootFrame()
        self.AddGraph()
        self.AddChannelFrame()
        self.AddBtnFrame()
    
    def AddChannelFrame(self):
        self.ChannelFrame.append([])
        self.ViewVar.append([])
        self.OptionVar.append([])
        self.FuncVar.append([])
        self.ChannelFrame[self.totalFrames].append(LabelFrame(self.frames[self.totalFrames],
                                                              pady=5, bg="white"))
        self.ChannelFrame[self.totalFrames].append(self.totalFrames)

        self.ChannelFrame[self.totalFrames][0].grid(
            column=0, row=1, padx=5, pady=5, rowspan=16, sticky="N")

        self.AddChannel(self.ChannelFrame[self.totalFrames])
    
    def AddChannel(self, ChannelFrame):
        if len(ChannelFrame[0].winfo_children()) < 8:
            NewFrameChannel = LabelFrame(ChannelFrame[0], bg="gray")
            # print(
            #     f"Mumber of element into the Frame {len(ChannelFrame.winfo_children())}")

            NewFrameChannel.grid(column=0, row=len(
                ChannelFrame[0].winfo_children())-1) 

            self.ViewVar[ChannelFrame[1]].append(IntVar())
            Ch_btn = Checkbutton(NewFrameChannel, variable=self.ViewVar[ChannelFrame[1]][len(self.ViewVar[ChannelFrame[1]])-1],
                                 onvalue=1, offvalue=0, bg="gray")
            Ch_btn.grid(row=0, column=0, padx=1)
            self.ChannelOption(NewFrameChannel, ChannelFrame[1])
            self.ChannelFunc(NewFrameChannel, ChannelFrame[1])

    def ChannelOption(self, frame, ChannelFrameNumber):
        self.OptionVar[ChannelFrameNumber].append(StringVar())

        bds = self.data.Channels
        self.OptionVar[ChannelFrameNumber][len(self.OptionVar[ChannelFrameNumber]) - 1].set(bds[0])
        drop_ch = OptionMenu(frame, self.OptionVar[ChannelFrameNumber][len(self.OptionVar[ChannelFrameNumber]) - 1].set(bds[0]), *bds)
        drop_ch.config(width=5)
        drop_ch.grid(row=0, column=1, padx=1)

    def ChannelFunc(self, frame, ChannelFrameNumber):
        self.FuncVar[ChannelFrameNumber].append(StringVar())

        bds = self.data.FunctionMaster
        self.FuncVar[ChannelFrameNumber][len(self.OptionVar[ChannelFrameNumber]) - 1].set(bds[0])
        drop_ch = OptionMenu(frame, self.FuncVar[ChannelFrameNumber][len(self.OptionVar[ChannelFrameNumber]) - 1].set(bds[0]), *bds)
        drop_ch.config(width=5)
        drop_ch.grid(row=0, column=2, padx=1)
            
    def DeleteChannel(self, ChannelFrame):
        if len(ChannelFrame[0].winfo_children()) > 1:
            ChannelFrame[0].winfo_children()[len(ChannelFrame[0].winfo_children())-1].destroy()
            self.ViewVar[ChannelFrame[1]].pop()
            self.OptionVar[ChannelFrame[1]].pop()
            self.FuncVar[ChannelFrame[1]].pop()

    def AddMasterFrame(self):
        self.frames.append(LabelFrame(
            self.root, text=f"Display Manager - {len(self.frames)+1}", padx=5, pady=5, bg="gray"))
        self.totalFrames = len(self.frames)-1

        if self.totalFrames % 2 == 0:
            self.framesCol = 0
        else:
            self.framesCol = 9

        self.framesRow = 4 + 4 * int(self.totalFrames/2)
        self.frames[self.totalFrames].grid(
            padx=5, column=self.framesCol, row=self.framesRow, columnspan=9, sticky=NW)

    def AdjustRootFrame(self):
        self.totalFrames = len(self.frames) - 1
        if self.totalFrames > 0:
            RootW = 800*2
        else:
            RootW = 800
        if self.totalFrames + 1 == 0:
            RootH = 120
        else:
            RootH = 120 + 430* (int(self.totalFrames/2) + 1)
        self.root.geometry(f"{RootW}x{RootH}")

    def AddGraph(self):
        self.figs.append([])
        self.figs[self.totalFrames].append(plt.Figure(figsize=(7, 5), dpi=80))

        self.figs[self.totalFrames].append(
            self.figs[self.totalFrames][0].add_subplot(111))

        self.figs[self.totalFrames].append(
            FigureCanvasTkAgg(self.figs[self.totalFrames][0], master=self.frames[self.totalFrames]))

        self.figs[self.totalFrames][2].get_tk_widget().grid(
            column=1, row=0, rowspan=17, columnspan=4, sticky=N)
        
    def AddBtnFrame(self):

        print(self.totalFrames)
        btnH = 2
        btnW = 4
        self.ControlFrames.append([])
        self.ControlFrames[self.totalFrames].append(
            LabelFrame(self.frames[self.totalFrames], pady=5, bg="gray"))
        self.ControlFrames[self.totalFrames][0].grid(
            column=0, row=0, padx=5, pady=5, sticky="N")
        self.ControlFrames[self.totalFrames].append(
            Button(self.ControlFrames[self.totalFrames][0], text="+", bg="white", width=btnW, height=btnH, command=partial(self.AddChannel, self.ChannelFrame[self.totalFrames])))
        self.ControlFrames[self.totalFrames][1].grid(
            column=0, row=0, padx=5, pady=5)
        self.ControlFrames[self.totalFrames].append(
            Button(self.ControlFrames[self.totalFrames][0], text="-", bg="white", width=btnW, height=btnH, command=partial(self.DeleteChannel, self.ChannelFrame[self.totalFrames])))
        self.ControlFrames[self.totalFrames][2].grid(
            column=1, row=0, padx=5, pady=5)

if __name__ == "__name__":
    RootGUI()
    ComGUI()
    ConnGUI()
    GamepadGUI()
    LoggerGUI()
    ControlsGUI()
    MapGUI()
    DisGUI()
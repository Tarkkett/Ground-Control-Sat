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

from functools import partial

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
        self.root.destroy()
        self.serial.SerialClose(self)
        self.serial.threading = False
        
        
        

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
                self.conn = ConnGUI(self.root, self.serial, self.data, self.mainFont)
                self.controller = GamepadGUI(self.root, self.gamepad)
                self.logger = LoggerGUI(self.root, self.data, self.serial)
                self.map = MapGUI(self.root, self.mainFont)

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
            self.serial.SerialClose(self)
            self.logger.LoggerGUIClose()
            self.conn.ConnGUIClose()
            self.controller.GamepadGUIClose()
            self.map.MapGUIClose()
            
            
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
        self.dataCanvas = Canvas(self.frame, width=600, height=280, background="black", highlightbackground="black")
        self.vsb = Scrollbar(self.frame, orient='vertical', command=self.dataCanvas.yview)

        self.dataFrame = Frame(self.dataCanvas, bg="black", pady=5)
        self.dataCanvas.create_window((5,10),window=self.dataFrame,anchor='nw')
        

        self.threading = True

        self.loggerThread = threading.Thread(target=self.PullLog, name="Log thread", daemon=True)
        self.loggerThread.start()
        self.LoggerGUIOpen()
    
    def PullLog(self):
        while self.threading:
            if self.data.data_ok:
                Label(master=self.dataFrame, text=f">>{self.data.msg}", foreground="lime", background="black", pady=3).pack()
            sleep(0.4)

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
    def __init__(self, root, mainFont):
        self.root = root

        self.threading = True

        self.mapSizeX = 60
        self.mapSizeY = 60

        self.font = mainFont

        
        self.frame = LabelFrame(root, text="Map frame", padx=5, pady=5, bg="gray", relief="ridge")
        
        self.map_widget = tkintermapview.TkinterMapView(self.frame, width=800, height=380, corner_radius=20)
        self.map_widget.set_tile_server("https://mt0.google.com/vt/lyrs=m&hl=en&x={x}&y={y}&z={z}&s=Ga", max_zoom=22)
        self.zoomLevel = Label(self.frame, text= f"Zoom level: {self.map_widget.last_zoom}x", padx=5, pady=5, bg="gray", font=self.font)

        self.mapThread = threading.Thread(target = self.UpdateMap, name="Map Thread", daemon=True)
        self.mapThread.start()
        

        #messagebox.showwarning("Warning!", "Should place!")
        self.MapGUIOpen()

    def MapGUIOpen(self):
        self.root.geometry("1280x720")

        #Map frame
        self.frame.grid(row=0, column=7, padx=5, pady=5, rowspan=4, sticky=NW)
        self.zoomLevel.grid(row=0, column=1, rowspan=4)
        self.map_widget.grid(row=0, column=0)
        self.map_widget.set_address("Moletu aerodromas")
        self.map_widget.set_zoom(18)
        

    def UpdateMap(self):
        
        while self.threading:
            self.zoomLevel["text"] = f"Zoom level: {self.map_widget.zoom}x"
            self.map_widget.update()
            #self.infoLabel["text"] = f"Info: {self.map_widget.info}"
    
    def MapGUIClose(self):
        for widget in self.frame.winfo_children():
            widget.destroy()
        self.frame.destroy()
        self.root.geometry("360x120")


class GamepadGUI():
    def __init__(self, root, gamepad):
        self.root = root
        self.gamepad = gamepad
        self.threading = True
        monitorThread = threading.Thread(target=self.UpdateControllerData, args=(self.gamepad,), name="Gamepad Monitor", daemon=True)
        

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
    
    def UpdateControllerData(self, gamepad):
        
        while self.threading and len(gamepad.joysticks) > 0 and gamepad.threading ==True:
            try:
                self.barLeftX["value"] = int(gamepad.lockLX)
                self.barLeftY["value"] = int(gamepad.lockLY)
                self.barRightX["value"] = int(gamepad.lockRX)
                self.barRightY["value"] = int(gamepad.lockRY)
                self.s.configure("LabeledProgressbar", text="".format(int(gamepad.lockLX)))
                self.s.configure("LabeledProgressbar", text="".format(int(gamepad.lockLY)))
                self.s.configure("LabeledProgressbar", text="".format(int(gamepad.lockRX)))
                self.s.configure("LabeledProgressbar", text="".format(int(gamepad.lockRY)))
                self.root.update()
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
        self.root.geometry("360x120")


class ConnGUI():
    def __init__(self, root, serial, data, mainFont):
        self.root = root
        self.serial = serial
        self.data = data
        self.mainFont = mainFont

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
        self.chartMaster = DisGUI(self.root, self.serial, self.data)

    def ConnGUIOpen(self):
        self.root.geometry('1280x720')

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
        sleep(2)
        self.data.data_ok = False
        print("Changed to false!")

    def save_data(self):
        pass

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
    def __init__(self, root, serial, data):
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
    MapGUI()
    DisGUI()
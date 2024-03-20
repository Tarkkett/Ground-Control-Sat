from tkinter import *
from tkinter import messagebox
from tkinter import ttk
import tkinter.font as TkFont
import threading
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

class RootGUI:
    def __init__(self, serial, data):
        self.root = Tk()
        self.root.title("MagnifiCanSatGround Station Comms!")
        self.root.geometry("360x120")
        self.root.config(bg="grey")
        self.root.iconbitmap("Assets/icon.ico")

        self.serial = serial
        self.data = data

        self.root.protocol("WM_DELETE_WINDOW", self.close_window)

    def close_window(self):
        print("Closing the main window now!")
        self.root.destroy()
        self.serial.SerialClose(self)
        self.serial.threading = False

class ComGUI():
    def __init__(self, root, serial, data):

        self.root = root
        self.serial = serial
        self.data = data
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
        bds = ["-", "300", "9600", "115200"]
        self.clicked_baudr.set(bds[0])
        self.drop_baudr = OptionMenu(self.frame, self.clicked_baudr, *bds, command=self.connect_ctrl)
        self.drop_baudr.config(width=10)

    def publish(self):
        self.frame.grid(row=0, column=0, rowspan=3, columnspan=3, padx=5, pady=5)

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
                InfoMsg = f"Connection success! "
                messagebox.showinfo("showinfo", InfoMsg)
                
                #Start Comms
                self.conn = ConnGUI(self.root, self.serial, self.data, self.mainFont)
                self.serial.t1 = threading.Thread(
                    target = self.serial.SerialSync, args = (self,), daemon=True
                )
                self.serial.t1.start()

            else:
                ErrorMsg = f"FATAL Error trying to connect in the last step! "
                messagebox.showerror("showerror", ErrorMsg)
        else:
            self.serial.threading = False
            self.serial.SerialClose()
            self.conn.ConnGUIClose()
            self.data.ClearData()
            
            InfoMsg = f"Connection is now closed! "
            messagebox.showwarning("Warning!", InfoMsg)
            self.btn_connect["text"] = "Connect"
            self.btn_refresh["state"] = "active"
            self.drop_baudr["state"] = "active"
            self.drop_com["state"] = "active"



    def com_refresh(self):
        self.drop_com.destroy()
        self.ComOptionMenu()
        self.drop_com.grid(column=2, row=2)
        logic = []
        self.connect_ctrl(logic)

class ConnGUI():
    def __init__(self, root, serial, data, mainFont):
        self.root = root
        self.serial = serial
        self.data = data
        self.mainFont = mainFont

        self.frame = LabelFrame(root, text="Connection manager", padx=5, pady=5, bg='grey', width=60)

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
        self.root.geometry('800x120')
        self.frame.grid(row=0,column=4, rowspan=3, columnspan=5, padx=5, pady=5)
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
        pass


    def stop_stream(self):
        pass


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
                self.chartMaster.ControlFrames.pop()
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

    def AddChannelMaster(self):
        self.AddMasterFrame()
        self.AdjustRootFrame()
        self.AddGraph()
        self.AddBtnFrame()

    def AddMasterFrame(self):
        self.frames.append(LabelFrame(self.root, text=f"Display Manager - {len(self.frames) + 1}", padx=5, pady=5, bg="grey"))
        self.totalFrames = len(self.frames) - 1
        print(len(self.frames)) # 1
        print(self.totalFrames) # 0

        if self.totalFrames % 2 ==0:
            self.framesCol = 0
        else:
            self.framesCol = 9

        self.framesRow = 4 + 4 * int(self.totalFrames/2)
        print("Col" + str(self.framesCol))
        print("Row" + str(self.framesRow))
        self.frames[self.totalFrames].grid(padx = 5, column = self.framesCol, row = self.framesRow, columnspan = 9, sticky="NW")

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
        self.figs[self.totalFrames].append(plt.figure(figsize=(7,5), dpi=80))

        self.figs[self.totalFrames].append(self.figs[self.totalFrames][0].add_subplot(111).set_title(f'{len(self.frames)}'))

        self.figs[self.totalFrames].append(FigureCanvasTkAgg(self.figs[self.totalFrames][0], master = self.frames[self.totalFrames]))
        self.figs[self.totalFrames][2].get_tk_widget().grid(column = 1, row = 0, columnspan = 17, sticky="N")
        
    def AddBtnFrame(self):
        '''
        Method to add the bottons to be used to add further channels 
        the button need to use a partial lib when calling the libary so it can keep the right 
        Widget target --> Further details in the YouTube WeeW-Stack
        '''
        btnH = 2
        btnW = 4
        self.ControlFrames.append([])
        self.ControlFrames[self.totalFrames].append(LabelFrame(self.frames[self.totalFrames],
                                                               pady=5, bg="white"))
        self.ControlFrames[self.totalFrames][0].grid(
            column=0, row=0, padx=5, pady=5,  sticky=N)

        self.ControlFrames[self.totalFrames].append(Button(self.ControlFrames[self.totalFrames][0], text="+",
                                                           bg="white", width=btnW, height=btnH))
        self.ControlFrames[self.totalFrames][1].grid(
            column=0, row=0, padx=5, pady=5)
        self.ControlFrames[self.totalFrames].append(Button(self.ControlFrames[self.totalFrames][0], text="-",
                                                           bg="white", width=btnW, height=btnH))
        self.ControlFrames[self.totalFrames][2].grid(
            column=1, row=0, padx=5, pady=5)

if __name__ == "__name__":
    RootGUI()
    ComGUI()
    ConnGUI()
    DisGUI()
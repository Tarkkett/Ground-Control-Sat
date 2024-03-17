from tkinter import *
from tkinter import messagebox

class RootGUI:
    def __init__(self):
        self.root = Tk()
        self.root.title("MagnifiCanSatGround Station Comms!")
        self.root.geometry("360x120")
        self.root.config(bg="grey")
        self.root.iconbitmap("Assets/icon.ico")

class ComGUI():
    def __init__(self, root, serial):

        self.root = root
        self.serial = serial

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
            else:
                ErrorMsg = f"FATAL Error trying to connect in the last step! "
                messagebox.showerror("showerror", ErrorMsg)
        else:
            self.serial.SerialClose()
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

if __name__ == "__name__":
    RootGUI()
    ComGUI()
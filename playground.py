from tkinter import *
from time import sleep

def show_values():
    print (w1.get(), w2.get())
    print("Hello")

master = Tk()
w1 = Scale(master, from_=0, to=42)
w1.set(19)
w1.pack()
w2 = Scale(master, from_=0, to=200, orient=HORIZONTAL)
w2.set(23)
w2.pack()
Button(master, text='Show', command=show_values).pack()
sleep(0.2)

mainloop()
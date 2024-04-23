import tkinter as tk

class App:
    def __init__(self, root):
        self.root = root
        self.root.geometry("400x300")

        # Create a frame
        self.frame = tk.Frame(root, bg="gray", width=200, height=200)
        self.frame.pack(fill="both", expand=True)

        # Calculate canvas size based on frame size
        frame_width = self.frame.winfo_width()
        frame_height = self.frame.winfo_height()
        canvas_width = int(frame_width * 0.8)
        canvas_height = int(frame_height * 0.8)

        # Create a canvas inside the frame
        self.canvas = tk.Canvas(self.frame, bg="white", width=canvas_width, height=canvas_height)
        self.canvas.pack(expand=True)

        # Draw a rectangle on the canvas
        self.canvas.create_rectangle(10, 10, canvas_width - 10, canvas_height - 10, fill="blue")

root = tk.Tk()
app = App(root)
root.mainloop()
import tkinter as tk
from tkinter import ttk

def update_progress():
    progress_var.set(progress_var.get() + 10)
    progress_label.config(text=f"Progress: {progress_var.get()}%")

root = tk.Tk()
root.title("ProgressBar with Text")

# Create progress bar
progress_var = tk.DoubleVar()
progress_bar = ttk.Progressbar(root, orient="horizontal", length=200, mode="determinate", variable=progress_var)
progress_bar.pack(pady=10)

# Create label to display progress text
progress_label = tk.Label(root, text="Progress: 0%")
progress_label.pack()

# Button to update progress
update_button = tk.Button(root, text="Update Progress", command=update_progress)
update_button.pack(pady=10)

root.mainloop()
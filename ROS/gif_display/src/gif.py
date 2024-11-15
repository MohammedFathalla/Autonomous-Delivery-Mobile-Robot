#!/usr/bin/env python3
import tkinter as tk
from PIL import Image, ImageTk

# Load the GIF
gif_path = 'home/ubuntu/catkin_ws/src/gif_display/gif/face.gif'
gif = Image.open(gif_path)

# Set up the tkinter window for full-screen display
root = tk.Tk()
root.title("GIF Display")
root.attributes('-fullscreen', True)  # Fullscreen mode
label = tk.Label(root)
label.pack()

frames = []

# Load the frames from the GIF without modification
try:
    while True:
        frame = gif.copy()
        frames.append(ImageTk.PhotoImage(frame))
        gif.seek(gif.tell() + 1)
except EOFError:
    pass  # End of GIF

# Function to loop and display frames
def animate(index):
    frame = frames[index]
    label.config(image=frame)
    index = (index + 1) % len(frames)  # Loop the GIF
    root.after(100, animate, index)  # Update every 100 ms

# Start the animation
animate(0)

# Make sure the window stays open
root.mainloop()
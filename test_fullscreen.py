import tkinter as tk
from PIL import Image, ImageTk
import os

# Make a fullscreen window
root = tk.Tk()
root.attributes('-fullscreen', True)
root.bind('<Escape>',lambda e: root.destroy())
w, h = root.winfo_screenwidth(), root.winfo_screenheight()
im_size = min(w, h)

# Show an apriltag image
canvas = tk.Canvas(root, width=w, height=h)      
canvas.pack()      
canvas.configure(background='black')

img_path = "images/tag36_11_%05d.png" % 0
assert os.path.isfile(img_path), img_path
img = Image.open(img_path)
img = img.resize((im_size, im_size), Image.NEAREST)
img = ImageTk.PhotoImage(img)
canvas.create_image(0,0, anchor=tk.NW, image=img)

root.mainloop()
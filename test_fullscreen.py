import tkinter as tk


root = tk.Tk()
root.attributes('-fullscreen', True)
root.bind('<Escape>',lambda e: root.destroy())
root.mainloop()
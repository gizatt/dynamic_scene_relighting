import consolemenu
from consolemenu import ConsoleMenu
from consolemenu.items import FunctionItem, SubmenuItem, CommandItem

class InterfaceManager():
    ''' Simple interface offering terminal control (since I'm running
    this over SSH right now) for RGB camera exposure + changing
    relighting modes. '''

    def __init__(self):
        self.menu = ConsoleMenu("Scene Relighting Control")

        # Camera controls
        self.camera_menu = ConsoleMenu("Camera Control")
        self.camera_menu_item = SubmenuItem("Change camera settings", self.camera_menu, menu=self.menu)
        self.set_camera_exposure = FunctionItem("Set exposure", self.update_exposure)
        self.camera_menu.append_item(self.set_camera_exposure)
        self.exposure = 0
        self.menu.append_item(self.camera_menu_item)

        # Enable/display meshcat vis
        self.meshcat_vis_active = True
        self.toggle_meshcat_vis_item = FunctionItem("Disable meshcat vis", self.toggle_meshcat_vis)
        self.menu.append_item(self.toggle_meshcat_vis_item)

        self.menu.start()

    def toggle_meshcat_vis(self):
        if self.meshcat_vis_active:
            self.meshcat_vis_active = False
            self.toggle_meshcat_vis_item.text = "Enable meshcat vis"
        else:
            self.meshcat_vis_active = True
            self.toggle_meshcat_vis_item.text = "Disable meshcat vis"

    def get_meshcat_vis_active(self):
        return self.meshcat_vis_active

    def update_exposure(self):
        new_exposure = input("New exposure [0 for autoexposure, 1-10000 otherwise]")
        try:
            new_exposure = int(new_exposure)
            if new_exposure >= 0 and new_exposure <= 10000:
                self.exposure = new_exposure
        except ValueError:
            pass

    def get_exposure(self):
        return self.exposure

    def is_alive(self):
        return self.menu.is_alive()
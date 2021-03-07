from functools import partial
import consolemenu
from consolemenu import ConsoleMenu
from consolemenu.items import FunctionItem, SubmenuItem, CommandItem
import logging


class InterfaceManager():
    ''' Simple interface offering terminal control (since I'm running
    this over SSH right now) for RGB camera exposure + changing
    relighting modes. '''

    def __init__(self, modes):
        self.menu = ConsoleMenu("Scene Relighting Control")

        self.refresh_item = FunctionItem("Refresh frame info", lambda: self.force_main_menu_refresh)
        self.menu.append_item(self.refresh_item)

        # Camera controls: exposure and depth range.
        self.camera_menu = ConsoleMenu("Camera Control")
        self.camera_menu_item = SubmenuItem("Change camera settings", self.camera_menu, menu=self.menu)
        self.set_camera_exposure = FunctionItem("Set exposure", self.update_exposure)
        self.camera_menu.append_item(self.set_camera_exposure)
        self.exposure = 0
        self.depth_range = [1.0, 1.25]
        self.set_min_depth_item = FunctionItem("Set min depth [%f]" % self.depth_range[0], self.set_min_depth)
        self.set_max_depth_item = FunctionItem("Set max depth [%f]" % self.depth_range[1], self.set_max_depth)
        self.camera_menu.append_item(self.set_min_depth_item)
        self.camera_menu.append_item(self.set_max_depth_item)
        self.menu.append_item(self.camera_menu_item)

        # Mode selection
        self.modes = modes
        self.mode_name = self.modes[0]
        self.mode_menu = ConsoleMenu("Demo Modes", "Active mode: %s" % self.mode_name)
        self.mode_menu_item = SubmenuItem("Change demo mode", self.mode_menu, menu=self.menu)
        self.mode_items = []
        for key in self.modes:
            new_mode_item = FunctionItem("Set mode [%s]" % key, partial(self.set_mode, key))
            self.mode_items.append(new_mode_item)
            self.mode_menu.append_item(new_mode_item)
        self.menu.append_item(self.mode_menu_item)

        # Enable/display meshcat vis
        self.meshcat_vis_active = True
        self.toggle_meshcat_vis_item = FunctionItem("Disable meshcat vis", self.toggle_meshcat_vis)
        self.menu.append_item(self.toggle_meshcat_vis_item)

        # Enable/disable face detection
        self.face_detection_active = True
        self.toggle_detector_item = FunctionItem("Disable face detector", self.toggle_face_detector)
        self.menu.append_item(self.toggle_detector_item)

        # Adjust image save rate
        self.image_save_rate = 0
        self.set_image_save_rate_item = FunctionItem(
            "Set frame divider for image saving [curr %d]" % self.image_save_rate,
            self.set_image_save_rate)
        self.menu.append_item(self.set_image_save_rate_item)
        self.menu.start()

    def set_mode(self, key):
        self.mode_name = key
        self.mode_menu.subtitle = "Active mode: %s" % self.mode_name

    def get_demo_mode(self):
        return self.mode_name

    def report_camera_stats(self, fps, min, max):
        # Updates subtitle to report camera stats
        self.menu.subtitle = "FPS [%.01f], Depth Range[%0.02f, %0.02f]" \
                              % (fps, min, max)

    def force_main_menu_refresh(self):
        self.menu.screen.clear()
        self.menu.draw()

    def get_depth_range(self):
        return self.depth_range

    def set_min_depth(self):
        new_depth = input("New min depth in meters:")
        try:
            new_depth = float(new_depth)
            if new_depth >= 0 and new_depth <= 30.0:
                self.depth_range[0] = new_depth
                self.set_min_depth_item.text = "Set min depth [%f]" % new_depth
        except ValueError():
            pass

    def set_max_depth(self):
        new_depth = input("New max depth in meters:")
        try:
            new_depth = float(new_depth)
            if new_depth >= 0 and new_depth <= 30.0:
                self.depth_range[1] = new_depth
                self.set_max_depth_item.text = "Set max depth [%f]" % new_depth
        except ValueError():
            pass

    def get_image_save_rate(self):
        return self.image_save_rate

    def set_image_save_rate(self):
        new_rate = input("Save image every [enter input] frames (0 to disable).")
        try:
            new_rate = int(new_rate)
            if new_rate >= 0:
                self.image_save_rate = new_rate
                self.set_image_save_rate_item.text = \
                    "Set frame divider for image saving [curr %d]" % self.image_save_rate
        except ValueError():
            pass

    def get_detector_active(self):
        return self.face_detection_active

    def toggle_face_detector(self):
        if self.face_detection_active:
            self.face_detection_active = False
            self.toggle_detector_item.text = "Enable face detector"
        else:
            self.face_detection_active = True
            self.toggle_detector_item.text = "Disable face detector"

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
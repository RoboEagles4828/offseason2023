import logging

class ToggleButton():
    def __init__(self, button, isAxis=False, onCallback=lambda: (), offCallback=lambda: ()):
        self.last_button = 0.0
        self.flag = False
        self.button = button
        self.isAxis = isAxis
        self.onCallback = onCallback
        self.offCallback = offCallback
    
    def toggle(self, buttons_list):
        currentButton = buttons_list[self.button]
        if self.isAxis:
            # currentButton = currentButton / 10000 if currentButton > 1 else currentButton
            if currentButton == -1.0  and self.last_button != -1.0:
                self.flag = not self.flag
                if self.flag:
                    self.onCallback()
                else:
                    self.offCallback()
                self.last_button = currentButton
                return self.flag
            else:
                self.last_button = currentButton
                return self.flag

        if currentButton == 1.0 and self.last_button == 0.0:
            self.flag = not self.flag
            if self.flag:
                self.onCallback()
            else:
                self.offCallback()
            self.last_button = currentButton
            return self.flag
        else:
            self.last_button = currentButton
            return self.flag
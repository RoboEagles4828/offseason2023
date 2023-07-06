class ToggleButton():
    def __init__(self, button, isAxis=False):
        self.last_button = 0.0
        self.flag = False
        self.button = button
        self.isAxis = isAxis
    
    def toggle(self, buttons_list):
        currentButton = buttons_list[self.button]
        if self.isAxis:
            # currentButton = currentButton / 10000 if currentButton > 1 else currentButton
            if currentButton == -10000.0  and self.last_button != -10000.0:
                self.flag = not self.flag
                self.last_button = currentButton
                return self.flag
            else:
                self.last_button = currentButton
                return self.flag

        if currentButton == 1.0 and self.last_button == 0.0:
            self.flag = not self.flag
            self.last_button = currentButton
            return self.flag
        else:
            self.last_button = currentButton
            return self.flag
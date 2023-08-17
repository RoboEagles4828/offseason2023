from commands2 import *
import logging

class DoNothingCommand(CommandBase):
    def __init__(self):
        super().__init__()
        
    def initialize(self):
        logging.info("DoNothingCommand initialized")
    
    def execute(self):
        logging.info("DoNothingCommand executing")
    
    def end(self, interrupted):
        logging.info("DoNothingCommand ended")
    
    def isFinished(self):
        return True
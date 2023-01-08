# -------------------------------------------- #
# Class Client                                 #
# Tasks:                                       #
# - emit serial messages to the ESP card       #
# - receive serial messages from the ESP card  #
# -------------------------------------------- #

# This module encapsulates the access for the serial port.
import serial
# This module converts between Python values and C structs
# represented as Python bytes objects.
from struct import *

class client:
    # initializing values and properties of objects
    def __init__(self, st_type, st_cid, st_rid, st_operation, st_ammount) -> None:
        self.st_type = st_type              # 0 or 1 w.r.t. being resp. REQUEST or RESPONSE
        self.st_cid = st_cid                # 0 .. 255 identification number for the client
        self.st_rid = st_rid                # 0 .. 255 identification number for the request
        self.st_operation = st_operation    # 0 or 1 w.r.t. being resp. withdraw or deposit
        self.st_ammount = st_ammount        # positive integer for the ammount 
    
    # method structurizing the message with struct.pack
    def structurer(self)-> bytes:
        st = struct.pack("<IIIQ", self.st_type, self.st_cid, self.st_rid, self.st_operation, self.st_ammount) 
        return st
    
    # method sending a message throw the serial port
    def send_message(self) -> None:
        ser = serial.Serial('/dev/ttyUSB0') # open serial port
        print(ser.name)                     # check which port was really used
        ser.write(self.structurer(self))    # write a string
        ser.close()                         # close port
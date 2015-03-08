from random import gauss
from math import *

class BattModel:
    def __init__(self):
        self.batt_v = 16.8
        self.batt_r = .04

    def batt_voltage(self,v=None):
        if v is not None:
            self.batt_v = v
        return self.batt_v

    def batt_resistance(self,r=None):
        if r is not None:
            self.batt_r = r
        return self.batt_r

    def current_meas(self,t,noise=0.1, min_current=1,max_current=20):
        return max((1+sin(t))/2 * (max_current-min_current) + gauss(0,noise),0)

    def voltage_meas(self,t,noise=0.1):
        return max(self.batt_voltage() - self.current_meas(t,0.0)*self.batt_resistance() + gauss(0,noise),0)

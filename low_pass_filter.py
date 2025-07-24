#!/bin/python3
from common import Utils, LPF_TYPE
#LPF: data = f(n-1)*a + (1-a)*f(n)
class Low_Pass_Filter():
    def __init__(self, alpha, is_angle, type:LPF_TYPE = LPF_TYPE.FIRST_ORDER):
        self._is_angle = is_angle
        self._type = type
        self._prev_filtered_output = None
        if(self._type == LPF_TYPE.FIRST_ORDER):
            if(alpha > 1):
                print("alpha must be less than 1")
                alpha = 1
            if(alpha < 0):
                alpha = 0
                print("alpha must be greater than 0")
        self._alpha = alpha

    def step(self, data_pt):
        if(self._type == LPF_TYPE.FIRST_ORDER):
            return self._first_order_step(data_pt=data_pt)
        elif(self._type == LPF_TYPE.OTHER):
            return self._other_step(data_pt=data_pt)

    def _other_step(self, data_pt):
        if(self._prev_filtered_output is None):
            self._prev_filtered_output = data_pt
            return data_pt
        if(self._is_angle):
            diff = Utils.angle_diff_rad(data_pt, self._prev_filtered_output)
            print("diff",diff,"alpha", self._alpha)
            if(abs(diff) > self._alpha):
                if(diff < 0):
                    self._prev_filtered_output = Utils.angle_diff_rad(self._prev_filtered_output, self._alpha)
                else:
                    self._prev_filtered_output = Utils.angle_diff_rad(self._prev_filtered_output, -self._alpha)
            else:
                self._prev_filtered_output = data_pt
        return self._prev_filtered_output

    def _first_order_step(self, data_pt):
        if(self._prev_filtered_output is None):
            self._prev_filtered_output = data_pt
            return data_pt
        if(self._is_angle):
            self._prev_filtered_output = Utils.angle_diff_rad((1-self._alpha) * self._prev_filtered_output, -(self._alpha)*data_pt)
        else:
            self._prev_filtered_output = (1-self._alpha) * self._prev_filtered_output + (self._alpha)*data_pt
        return self._prev_filtered_output

if __name__ == '__main__':
    import numpy as np
    filter = Low_Pass_Filter(alpha=0.8, is_angle=True, type = LPF_TYPE.FIRST_ORDER)
    out = filter.step(np.radians(4))
    print(np.degrees(out))
    out = filter.step(np.radians(20))
    print(np.degrees(out))
    out = filter.step(np.radians(40))
    print(np.degrees(out))
    out = filter.step(np.radians(30))
    print("30", np.degrees(out))
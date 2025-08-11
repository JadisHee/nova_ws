import dobot_api
import re
import numpy as np


def translater(recv_data):
    try:
        value_list = list(map(float, recv_data.split('{')[1].split('}')[0].split(',')))
        return value_list
    except Exception as e:
        print('err: ',e)
        return 0
if __name__ == '__main__':
    arm_status = dobot_api.DobotApiDashboard('192.168.5.1',29999)
    joints = translater(arm_status.GetAngle())
    joints_rad = np.deg2rad(joints)
    print(joints_rad)
    print(arm_status.GetPose(user=-1,tool=-1))




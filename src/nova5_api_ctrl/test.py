import dobot_api



if __name__ == '__main__':
    arm_status = dobot_api.DobotApiDashboard('192.168.5.1',29999)
    print(arm_status.GetPose())
    arm_status.close()



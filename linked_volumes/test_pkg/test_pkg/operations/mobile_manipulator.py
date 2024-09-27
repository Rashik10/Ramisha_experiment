from .robot import Robot
import time


class MobileManipulator(Robot):
    def __init__(self, group_id, backend_server_ip=None):
        super(MobileManipulator, self).__init__(group_id, backend_server_ip)

    def set_arm_pose(self, x, y, wait_s=2.6): # Not recommended to use a smaller wait time than this
        super(MobileManipulator, self).step([0., 0., 0., x, y, 1., 0.])
        if wait_s > 0.:
            time.sleep(wait_s)

    def set_mobile_base_speed_and_gripper_power(self, x : float, y : float, z : float, gripper_power : float):
        super(MobileManipulator, self).step([x, y, z, 0., 0., 0., gripper_power])


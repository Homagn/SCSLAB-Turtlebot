from Helpers.camera import camera
from Helpers.laser_scan import lscan
from Helpers.motion import mover

class environ(object):
    def __init__(self):
        self.steps = 0
        self.c = Camera("raw")
        self.l = lscan()
        self.m = mover(1000)
        self.m.get_model_pos()
        self.state = [float(self.m.x_sim),float(self.m.y_sim),float(self.m.r_sim)]

    def reset(self):
        self.m.teleport(0.0,0.0,0.0)
        obs_cam = self.c.see()
        obs_lidar = self.l.see()
        obs = {'camera':obs_cam, 'lidar':obs_lidar}
        self.update_state()
        rew = get_reward(self.state)
        return obs,rew,False,[]

    def get_reward(self,state):
        r = state[0]**2 + state[1]**2
        return r

    def update_state(self):
        self.state = [float(self.m.x_sim),float(self.m.y_sim),float(self.m.r_sim)]

    def isover(self):
        if self.steps >= 60: #or something happens to self.state
            self.steps = 0
            return True
        return False

    def step(self, a):
        self.m.teleport(a[0],a[1],a[2])

        obs_cam_next = self.c.see()
        obs_lidar_next = self.l.see()
        obs_next = {'camera':obs_cam_next, 'lidar':obs_lidar_next}

        self.update_state()
        rew = get_reward(self.state)

        self.steps += 1
        done = self.isover()

        info =[]

        return obs_next, rew, done, info
                
if __name__ =='__main__':
    e = environ()
import PyBulletEnv
import Obj
from numpy import random

if __name__ == "__main__":
    env = PyBulletEnv.PyBulletEnv()
    env.setup()
    ball = Obj.Obj("data/ball/basketball.obj")
    hoop = Obj.Obj("data/hoop/basketball_net_and_board.obj")

    ball.createObjectObj([0, 0, 0])
    ball.createObjectObj([0, 1, 0], 0.1)
    ball.createObjectObj([0, 2, 0], 0.001)
    hoop.createObjectURDF([-10, 0, 5], orient= [0, 0, 0, 1])

    env.run()



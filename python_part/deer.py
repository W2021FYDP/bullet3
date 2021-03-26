import PyBulletEnv
import Obj


if __name__ == "__main__":
    env = PyBulletEnv.PyBulletEnv()
    env.setup()
    cow = Obj.Obj("data/cow/cow.obj", 0.005)
    deer = Obj.Obj("data/deer/deer.obj", 0.02)

    for i in range(10):
        # For fun, try adding another 0!
        deer.createObjectObj([i, 0, 0], (i+1)/10000)

    #deer.createObjectURDF([0, 4, 0], orient=[0, 0, 0, 1])
    env.run()

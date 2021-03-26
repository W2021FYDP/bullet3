import PyBulletEnv
import Obj


if __name__ == "__main__":
    env = PyBulletEnv.PyBulletEnv()
    env.setup()
    cow = Obj.Obj("data/cow/cow.obj", 0.005)
    ratios = [0.1, 0.2, 0.4, 0.6, 0.8, 1.0] 

    cowIDs = []
    for r in ratios:
        cid = cow.createObjectURDF([0, 1, 2.7], r)
        cow.makeTransparent(cid)
        cowIDs.append(cid)
    
    env.run_dynamic(cowIDs)

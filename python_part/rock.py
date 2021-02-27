import pybullet as p
import time
import math
import pybullet_data
import PyBulletEnv
import Obj
import MeshSimplifier
import scipy

def trapezoid():
    base = 0.000624
    height = 0.172867
    # A shape wit 8 faces
    vertices = [
            [-0.246350, -0.246483, -base], [-0.151407, -0.176325, height],
            [-0.246350, 0.249205, -base], [-0.151407, 0.129477, height],
            [0.249338, -0.246483, -base], [0.154395, -0.176325, height],
            [0.249338, 0.249205, -base], [0.154395, 0.129477, height]
            ]
    indices = [
        0, 3, 2, 3, 6, 2, 7, 4, 6, 5, 0, 4, 6, 0, 2, 3, 5, 7, 0, 1, 3, 3, 7, 6, 7, 5, 4, 5, 1, 0, 6, 4,
        0, 3, 1, 5
    ]
        
    # create convex mesh from obj!
    stoneID = p.createCollisionShape(p.GEOM_MESH, vertices=vertices, 
                                    #indices=indices
                                    )

    # initialize and run object :) 
    p.createMultiBody(baseMass = 0, 
                    baseCollisionShapeIndex=stoneID, 
                    basePosition=[0, 0])

def cow():
    shift = [0, -0.02, 0]
    meshScaleParam = 2
    meshScale = [meshScaleParam, meshScaleParam, meshScaleParam]
    
    #cow = Obj.Obj("/data/cow.obj")
    cow = Obj.Obj("/data/deer.obj")
    vertices = cow.smaller()
    #print("COW VERTIVES : " + str(vertices))
    cowID = p.createCollisionShape(p.GEOM_MESH, vertices=vertices,
                                    meshScale = meshScale,
                                    flags = p.GEOM_FORCE_CONCAVE_TRIMESH)
    #print("Collision Shape ID: " + str(collisionShapeId))

    p.createMultiBody(baseMass = 1,
                        baseCollisionShapeIndex = cowID,
                        basePosition=[meshScale[0] * 2,
                                    meshScale[1] * 2, 
                                    1]
    )

def test_simplifier():
    # INFO: Mesh simplifier

    # Triangle  P = (1, 1, 1), Q = (1, 2, 0), R = (-1, 2, 1)
    triangle = [[1, 1, 1], [1, 2, 0], [-1, 2, 1]]

    # Random model
    simple = Obj.Obj("/data/simple.obj")
    simple.smaller()

    simplifier = MeshSimplifier.MeshSimplifier(simple.v, simple.group_faces['box'])

    plane_eqn = simplifier.triangle_to_plane(triangle)
    Kp = simplifier.calculate_quadric_Kp(plane_eqn)
    res = simplifier.simplify(simple)

    print("plane: {}".format(plane_eqn))
    print("Kp: {}".format(Kp))

if __name__ == "__main__":
    env = PyBulletEnv.PyBulletEnv()
    env.setup()
    #trapezoid()
    cow()
    test_simplifier()
    env.run()
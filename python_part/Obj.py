import os
import pybullet as p
import subprocess
import object2urdf
import PyBulletEnv

MINIMIZER = 'minimizeWithBlendr.py'

class Obj():
    
    def __init__(self, fn, scale=1):
        self.filename = fn
        self.scale = scale
        self.meshScale = [scale, scale, scale]
        self.obj = None
        self.decims = {}

        return 


    def smaller(self, ratio):
        '''
        Function smaller

        Takes in a ratio and creates a smaller .obj shape 
        '''
        if ratio >= 1.0:
            return self.filename

        name, suff = self.filename.split(".")
        output_file = name+ "_" + str(ratio) + '.' + suff
        if os.path.exists(output_file):
            return output_file

        ps = subprocess.Popen(["blender", "-b", "-P", MINIMIZER, "--", str(ratio), self.filename], stdout = subprocess.PIPE)
        output = subprocess.check_output(["grep", "after decimation"], stdin = ps.stdout)
        print(output)
        # Parse output to find new stats 
        # Find verts, edges, polys
        self.decims[ratio] = output_file
        return output_file
    
    def toURDF(self, filename, ratio):
        path = os.path.dirname(self.filename)
        builder = object2urdf.ObjectUrdfBuilder(path)
        # print("FILENAME IS " + filename)
        builder.build_urdf(filename=filename, output_folder=path+"/"+str(ratio)+"/", force_overwrite=True, decompose_concave=False, force_decompose =False, center = "bottom")
#        p.loadURDF("data/cow/data.urdf", globalScaling=self.scale, useFixedBase=True, basePosition = [0, 1, 2.7], baseOrientation=[1,1,1,1])
        return filename + ".urdf"

    def createObjectObj(self, position, ratio = 1.0):
        '''
        Function createObjectObj

        Creates an instance of the relevant obj file and adds it to the object instances dictionairy 
        '''
        if ratio in self.decims:
            file_to_load = self.decims[ratio]
        else: 
            file_to_load = self.smaller(ratio)
        #_, suff = self.filename.split('.')
        #if suff == "obj":
            # creating an obj
        visualShapeID = p.createVisualShape(
            shapeType = p.GEOM_MESH,
            fileName = file_to_load,
            rgbaColor = [1.0, 0.5, 0.0, 1.0], #[0.7, 0, 0.7, 1], #[3, 1, 1, 1],
            #specularColor = [0.4, 0.4, 0.4],
            visualFramePosition = position,
            meshScale = self.meshScale
        )
        collisionShapeID = p.createCollisionShape (
            shapeType = p.GEOM_MESH,
            fileName = self.filename, #file_to_load,
            #collisionFramePosition = position,
            meshScale = self.meshScale,
            #flags = p.GEOM_FORCE_CONCAVE_TRIMESH
        )

        #orn = p.getQuaternionFromEuler([0, 0, 0])
        return p.createMultiBody(
            baseMass = 1, #TODO: make a variable!
            baseInertialFramePosition = [0, 0, 0], #TODO: make a variable
            baseCollisionShapeIndex=collisionShapeID,
            baseVisualShapeIndex=visualShapeID,
            basePosition = position,
            # useMaximalCoordinates=True,
        #    baseOrientation = orn
            #useFixedBase = True
        )

    def printDecims(self):
        print(self.decims)

    def createObjectURDF(self, position, ratio = 1.0, orient = [1, 1, 1, 1]):
        # First, check if there's a dictionary value at that key!
        if ratio in self.decims:
            file_to_load = self.decims[ratio]
        else: 
            file_to_load = self.smaller(ratio)
        return p.loadURDF(self.toURDF(file_to_load, ratio), globalScaling=self.scale, useFixedBase=True, basePosition = position, baseOrientation=orient)  

    def move_x_y(self, x, y):
        if self.obj == None: 
            # TODO: Set up as an error
            print("Object not initialized!")
            return -1 
        #else, obj is a unique ID!
        pos, orient = p.getBasePositionAndOrientation(self.obj)
        _, _, oldZ = pos
        p.resetBasePositionAndOrientation(self.obj, [x, y, oldZ], orient)
        
        
    def change_position(self,roll, pitch, yaw):
        if self.obj == None:
            print("Object not initialized!")
            return -1

        # Otherwise, change it's position :) 
        pos, _ = p.getBasePositionAndOrientation(self.obj)
        orientation = p.getQuaternionFromEuler([roll, pitch, yaw])
        p.resetBasePositionAndOrientation(self.obj, pos, orientation)


    def makeTransparent(self, myID):
        p.changeVisualShape(myID, -1, rgbaColor = [0, 0, 0, 0])



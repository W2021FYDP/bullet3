import bpy
import argparse

DECIMATION_RATIO = 0.01

'''
Class MeshDecimator:
Calls headless blender in order to decimate relevant Obj instance

Due to Blender's very numerous limitations, we can only call this as a subprocess
Run as `blender -b -P minimizeWithBlendr.py -- <ratio> <obj location>`

Author: Stacy Gaikovaia 

All class methods are helpers, should not be instantiated directly! 

'''
class MeshDecimator:
    def __init__(self):
        return

    def delete_crap(self):
        bpy.ops.object.select_all(action="DESELECT")
        # Blender has a bunch of pre-done objects
        # stuff = ["Camera", "Cube", "Lamp"]
        # FOR OLDER API (< 2.8):  
        '''
        for s in stuff: 
            bpy.data.objects[s].select = True
            bpy.ops.object.delete()
        '''
        # FOR NEW API:
        for _, thing in enumerate(list(bpy.data.objects)):
            bpy.data.objects.remove(bpy.data.objects[thing.name])

    def get_meshes(self):
        meshes = []
        for obj in bpy.data.objects:
            if (obj.type == "MESH") and obj.name != 'Cube':
                meshes.append(obj)
        return meshes


    def main(self, ratio, input_model):
        #print(ratio)
        #print(input_model)
        inpa, inpb = input_model.split('.')
        output_model = inpa + "_" + str(ratio) + '.' + inpb
        self.delete_crap()
        bpy.ops.import_scene.obj(filepath=input_model)

        meshes = self.get_meshes()

        #print("meshes: ")
        #print(meshes)
        #print(list(bpy.data.objects))
        

        for (_, obj) in enumerate(meshes):
            #bpy.context.scene.objects.active = obj
            #obj.select_set(True)
            #print(bpy.context.active_object)
            bpy.context.view_layer.objects.active = obj
            modifier = obj.modifiers.new('DecimateMod', 'DECIMATE')
            modifier.ratio = ratio
            modifier.use_collapse_triangulate = True
            # NEW API uses: 
            bpy.ops.object.modifier_apply(modifier = 'DecimateMod')
            # Old API : 
            # bpy.ops.object.modifier_apply(apply_as='DATA', modifier = 'DecimateMod')
            # Print is necessairy because this is how we return to the main function 
            print("{} has {} verts, {} edges, {} polys after decimation".format(obj.name, len(obj.data.vertices), len(obj.data.edges), len(obj.data.polygons)))

        bpy.ops.export_scene.obj(filepath=output_model)
        print("Written to file " + output_model)
        return 

# main('data/trees9.obj')
print(__name__)
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Blender Mesh Decimator")
    _, args = parser.parse_known_args()
    for_this_script = args[args.index('--')+1 : ]
    parser.add_argument('ratio', metavar='R', type=float, help='Decimation ratio')
    parser.add_argument('filename', metavar='F', type=str, help='An .obj file')
    
    myArgs = parser.parse_args(for_this_script)
    #print(myArgs)

    md = MeshDecimator()
    md.main(myArgs.ratio, myArgs.filename)



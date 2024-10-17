#!/usr/bin/env python3
import io
import UnityPy
import rospy
from acroba_gym.unity_scene_manager import UnitySceneManager

class UnityAssetParser(object):
    def __init__(self, src) -> None:
        self._env = UnityPy.load(src)
        self._prefab_list = list([])

    def export_mesh(self, mesh, path = ""):
        
        if path=="":
            with open(f"{mesh.name}.obj", "wt", newline = "") as f:
                f.write(mesh.export())
        else:
            with open(path, "wt", newline = "") as f:
                f.write(mesh.export())

    def read_object_mesh_in_scene(self, name, level='level0'):
        mesh = None
        for obj in self._env.objects:
            if obj.type.name == "GameObject" and obj.assets_file.name == level:
                gobj = obj.read()
                if gobj.name == name:
                    if gobj.m_MeshFilter is not None:
                        mf = gobj.m_MeshFilter.read()
                        if mf.m_Mesh is not None:
                            mesh = mf.m_Mesh.read()
                            return mesh
        return mesh

    def read_prefab_mesh(self, name):
        mesh = None
        for obj in self._env.objects:
            if obj.type.name == "GameObject" and ("sharedassets" in obj.assets_file.name or "resources.assets" in obj.assets_file.name):
                gobj = obj.read()
                if gobj.name == name:
                    if gobj.m_MeshFilter is not None:
                        mf = gobj.m_MeshFilter.read()
                        if mf.m_Mesh is not None:
                            mesh = mf.m_Mesh.read()
                            return mesh
        return mesh

    def read_prefab_list(self):
        if len(self._prefab_list)==0:
            for obj in self._env.objects:
                if obj.type.name == "GameObject" and ("sharedassets" in obj.assets_file.name or "resources.assets" in obj.assets_file.name):
                    gobj = obj.read()
                    if gobj.m_MeshFilter is not None:
                        tf = gobj.m_Transform.read()
                        if tf.m_Father.get_obj() is None:
                            mf = gobj.m_MeshFilter.read()
                            if mf.m_Mesh is not None:
                                self._prefab_list.append(gobj.name)
        
        return self._prefab_list

if __name__ =="__main__":
    rospy.init_node('unity_asset_parser')
    src = "./automatica/automatica_cell_Data"
    #src = "dir_to_virtual_scene_data_folder" # /virtual_scenes/IMR_cell/IMR_cell_Data
    asset_parser = UnityAssetParser(src)
    prefabs_list = asset_parser.read_prefab_list()
    print(prefabs_list)
	#feed the name in the prefab list
    mesh = asset_parser.read_prefab_mesh("pth2")
    if mesh is None:
        print("prefab mesh is not found")
	#feed the object name you obtain from /model_states top
    sm = UnitySceneManager()
    obj_dict = {}
    while not bool(obj_dict):
        obj_dict = sm.get_all_models_dict()
    obj_name = list(obj_dict.keys())
    print(obj_name)
    obj_mesh = asset_parser.read_object_mesh_in_scene(obj_name[-1])
    if obj_mesh is None:
        print("obj_mesh is not found")
    #export function only support .obj file
    asset_parser.export_mesh(mesh,"pth2.obj")
    asset_parser.export_mesh(obj_mesh,"{}.obj".format(obj_name[-1]))
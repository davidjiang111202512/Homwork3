import numpy as np
import open3d as o3d
import copy

# load obj
def load_obj(path, texture_path=None):
  if texture_path is None:
    mesh = o3d.io.read_triangle_mesh(path)
  else:
    mesh = o3d.io.read_triangle_mesh(path, True)
    mesh.textures = [o3d.io.read_image(texture_path)]

  return mesh

# merge 2 meshes into 1
def combine(meshA, meshB): return meshA + meshB

# move the mesh along a given direction
def translate(mesh, shift=[0, 0, 0]):
  vertices = np.asarray(mesh.vertices)
  vertices += np.array(shift)

# scale along the center
def scale(mesh, scale=[1, 1, 1]):
  vertices = np.asarray(mesh.vertices)
  center = np.mean(vertices, axis=0)

  vertices -= center #先拉回跟原點對比
  vertices *= scale #再放大
  vertices += center #再跟原點對比

# rotate along the center
def rotate(mesh, theta=0, axis='x'):
  vertices = np.asarray(mesh.vertices)
  center = np.mean(vertices, axis=0)

  # define rotation matrix
  if axis == 'x': 
    m = np.array([
      [ 1,              0,              0],
      [ 0,  np.cos(theta), -np.sin(theta)],
      [ 0,  np.sin(theta),  np.cos(theta)]
    ])
  elif axis == 'y':
    m = np.array([
      [ np.cos(theta), 0, np.sin(theta)],
      [             0, 1,             0],
      [-np.sin(theta), 0, np.cos(theta)]
    ])
  elif axis == 'z':
    m = np.array([
      [np.cos(theta), -np.sin(theta), 0],
      [np.sin(theta),  np.cos(theta), 0],
      [            0,              0, 1]
    ])
  else: return

  vertices -= center
  vertices = np.dot(vertices, m.T) #算投影距離
  vertices += center

  mesh.vertices = o3d.utility.Vector3dVector(vertices)

if __name__ == "__main__":
  meshA = load_obj("/Users/jiangdawei/Desktop/lesson 1/6-cube.obj", "/Users/jiangdawei/Desktop/lesson 1/grass.png")
  meshB = load_obj("/Users/jiangdawei/Desktop/lesson 1/5-edge.obj", "/Users/jiangdawei/Desktop/lesson 1/grass.png")

  translate(meshA, [ 1, 0, 0])
  translate(meshB, [-1, 0, 0])

  scale(meshB, [0.5, 0.5, 0.5])
  scale(meshB, [1.5, 1.5, 1.5])

  rotate(meshA, np.pi/4, 'x')
  rotate(meshB, np.pi/4, 'z')

  mesh = combine(meshA, meshB)

  # save obj
  o3d.io.write_triangle_mesh("/Users/jiangdawei/Desktop/homework2/output/output8.obj", mesh)


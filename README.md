# Physically Base Simulation project - Group 11
## How to run the code
### main.py (in folder Simulator)
main.py represent our simulator. We have prepared a mock simulation
to simplify the imports of meshes and the set-up of the program.<br/>
main.py can be runned in smulation mode or skinning mode.<br/>
To run in simulation mode, no arguments are required:
```
python3 main.py
```
To run in skinning mode, two parameters are required:
```
python3 main.py True [num_frames]
```
Where "True" indicate that the skinning is active, while num_frames
is the number of frames produced. These frames can be rendered in any 3d
program. We used Blender. For example, to produce the whole animation:
```
python3 main.py True 2005
```
A sample execution is the following:
![alt text](./README_pictures/img3.png)
### mesh_generator.py (in folder MeshGenerator)
mesh_generator.py allow to build custom particle graphs that can be
used in the simulator. In the folder Meshes, we have prepared a duck
mesh that can be used as a test. The programme does not require any
parameter.
```
python3 mesh_generator.py
```
During the execution, a CLI will be used to set up some parameters.
These parameters are needed to process meshes of different shapes,
and sizes. Our suggestion is to use the duck mesh with the parameters
"0.90 1500 1.75 9".
A sample execution is the following:
![alt text](./README_pictures/img1.png)
![alt text](./README_pictures/img2.png)
We implemented several commands to interact with the generated mesh
```
q  # Exit from the script
```
```
vm  # Visualize the loaded mesh
```
```
vg  # Visualize the generated graph
```
```
g Meshes/graph_name  # Create graph_name.ply in the folder Meshes
```
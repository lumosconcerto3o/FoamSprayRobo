#!/bin/bash

# Convert and rename DAE files to STL
cd /home/lumos/pp_ws/src/robot_insulation/meshes/iiwa7/visual

# Link 0
assimp export base_link.dae link_0.stl

# Link 1
assimp export link_1.dae link_1.stl

# Link 2
assimp export link_2.dae link_2.stl

# Link 3
assimp export link_3.dae link_3.stl

# Link 4
assimp export link_4.dae link_4.stl

# Link 5
assimp export link_5.dae link_5.stl

# Link 6
assimp export link_6.dae link_6.stl

# Link 7
assimp export link_7.dae link_7.stl

echo "Mesh conversion complete"

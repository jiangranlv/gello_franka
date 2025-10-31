# fr3-urdf-pybullet
This repo contains a modified version of the [franka_description](https://github.com/frankaemika/franka_ros/tree/develop/franka_description) package, producing a .urdf model for the Franka Emika FR3 more suited to basic pybullet simulations. There is a test script showing that it can successfully be imported. Tested with pybullet==3.2.5 on python-3.11.0.

## Modifications

 - Removed gazebo specific-tag generation in franka_arm.xacro
 - Changed default value of hand and gazebo to true. Without the gazebo tag, the .urdf produced does not contain inertial parameters. We always want the end effector.
 - Replaced commands that dynamically look for the franka_description path on the system as referenced during the xacro parsing. Replaced with relative paths from the robots/fr3 folder. This is to ensure that we are using our modified xacro files.
 - Changed utils.xacro and franka_hand.xacro to have a material and color (rgba = 1 1 1 1), so that the color maps (Kd maps in the .mtl files) referencing the .png maps can be masked on top of the links.

## Usage

    git clone https://github.com/RumailM/fr3-urdf-pybullet
    cd fr3-urdf-pybullet/franka_description_pybullet/robots/fr3
    xacro fr3.urdf.xacro > fr3_pybullet.urdf.xml
We can run the test script to verify that we've generated a compatible .urdf file representing the fr3 robot

    cd ../../..
    python pybullet-test.py
    

![image](https://user-images.githubusercontent.com/36772370/198956116-d55df626-0314-4670-bc0c-d656d6e74270.png)

    
## Notes 
The original xacro file already accounts for the difference in joint limits between the panda and fr3. This is mentioned explicitly because both robots use common components for the robot. The inertial parameters do not come officially from Franka and these are for the Panda and not the FR3. It's not ideal, but it's our best for now.

### References
- https://blender.stackexchange.com/questions/18362/exporting-multiple-materials-to-a-single-diffuse-specular-map
- https://github.com/bulletphysics/bullet3/issues/1934
- https://www.youtube.com/watch?v=eYvgFWEiNp8&t=946s
- [This video shows how the .dae files were converted to .obj files in this repo](https://www.youtube.com/watch?v=rxbQ3IKlQgE)

# Change Camera Pitch

## Summary
This is a quick tip on how to change turtlebot3_waffle's camera pitch, so that the camera (in simulation) can look down or up

## Make change in urdf
Find this section of "Vision Camera" in turtlebot's urdf:
``` xml
<joint name="camera_joint" type="fixed">
      <origin xyz="0 0 ${1+base_height}" rpy="0 ${-cam_pitch} 0" />
      <parent link="base_link"/>
      <child link="camera_link" />
    </joint>
```
And you can modify ${-cam_pitch} to any angle you want.

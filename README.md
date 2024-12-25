## How to use the manipulator

Generate Trajectory:
- Use the path given
- Or, provide a mat file that contains the x,y,z and theta value (in rows) over time, and put them into matlab workspace, delete the given one
- Or, go to Trajectory_dynamics folder/trajectory generation and generate trapeziodal velocities via Sequence_points.m, add points in 3D and time stamp to the list

To run manipulator and visualize:
1. Select the folder with desired function, like inverse or object avoidance or dynamic
2. Linked the visualization folder
3. Run the init.m file and see the manipulator

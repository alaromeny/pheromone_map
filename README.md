# pheromone_map

#### How to Run
rosrun pheromone_map pheromone_wrapper.py





#### Parameters Essential (also what they are currently set to)

- size of Gazebo environment
```
environment_width = 100
environment_height = 100
```

- how often we want to update robot's local stigmergy (integer in Hz)
```
publisher_rate = 1
```
- number of ground robots (integer)
```
number_of_robots = 3
```
- resolution of the cells in the stigmergy map and  (in meters)
```
map_resolution = 0.25
```
- the radius of the trail of a single robot leaves on the pheromone map (in meters)
```
trail_radius = 0.8
```
- the value of the robot's trail in the matrix (integer)
```
robot_trail_value = 75
```
- the value of the obstacles trail in the matrix (integer)
```
wall_trail_value = 255
```

- These are diffusion parameters (Sigma is for gaussian blurr)
```
diffusion_sigma = 0.75
diffusion_rate = 10
```


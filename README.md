# pheromone_map

#### How to Run
rosrun pheromone_map pheromone_wrapper.py





#### Parameters Essential (also what they are currently set to)

- size of Gazebo environment
```
environment_width = 100
environment_height = 100
```

- how often we want to update robot's local stigmergy
```
publisher_rate = 1
```
- number of ground robots
```
number_of_robots = 4
```
- resolution of the cells in the stigmergy map
```
map_resolution = 0.25
```
- how many cells in each direction do our ground robots want to see - I haven't tested this if they are not equal
```
localResolution_x = 8
localResolution_y = 8
```
- These are diffusion parameters (Sigma is for gaussian blurr)
```
diffusion_sigma = 0.75
diffusion_rate = 10
```


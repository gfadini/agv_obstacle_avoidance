# AGV collision avoidance

Dependencies:
```
- python3
- scipy
- numpy
- matplotlib
- networkx
- sympy
- terminalplot
- casadi
- ffmpeg
- latex interpreter
```
To check the dependencies, in UNIX system it's possible to use ```install_python_dependencies.sh```.
In any case, the scripts are also proven to be working under Windows systems. In particular under Windows if ffmpeg was not installed in ```'C:\\ffmpeg\\bin\\ffmpeg.exe'```, modify the ```plt.rcParams['animation.ffmpeg_path']``` accordingly in ```agv_collision_avoidance.py```

All the simulation parameters of interest of the main simulation ```agv_collision_avoidance.py``` are modifiable from another script ```simulation_parameters.py```.

However, through command line arguments, some of them are more easily settable or can be toggled as follows:

**Example**
```python agv_collision_avoidance.py 'rvo' 'mhe' 'central' 4 'plant'```

Launches the simulation with the RVO algorithm, 4 robots in the default plant environment, with additional centralized kalman and MHE filter.

*Additional notes*
- Mutually exclusive parameters:
  - map ```plant``` or ```polygon```
  - collision avoidance algorithm ```rvo``` or ```potential```
- the highest integer number will be the number of robots
- to decide which map to launch other then default, modify the planning in ```agv_collision_avoidance.py```
- arguments specification is **not** positional


## Potential
### Collision avoidance with 3 robots encountering in the centre
![](Animation/simulation_3_potential_02-04_13-17.gif)
### Collision avoidance with 5 robots encounterig in the center
![](Animation/simulation_5_potential.gif)
### Collision avoidance with 2 robots encountering in the centre with a corridor
![](Animation/simulation_2_potential_02-04_15-25.gif)
### Collision avoidance with 8 robots in a simple map
![](Animation/simulation_8_potential_02-04_15-39.gif)

## RVO
### Collision avoidance with 2 robots encountering in the centre with a corridor
![](Animation/simulation_2_rvo_02-04_09-52.gif)
### Collision avoidance with 3 robots encountering in the centre
![](Animation/simulation_3_rvo_02-04_08-33.gif)
### Collision avoidance with 8 robots in a simple map
![](Animation/simulation_8_rvo_02-04_16-01.gif)

![](Animation/simulation_3_rvo_02-04_08-44.gif)

![](NOPE\Animation/simulation_3_rvo_02-04_08-44.gif)
![](NOPE\Animation/simulation_2_rvo_02-04_10-00.gif)


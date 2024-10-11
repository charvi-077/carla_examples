## Working on Carla Simulator : Self Driving Cars Platform 

##  Installation Steps : 
- Carla Version : 0.9.11
- Download tar file of 4.xx GB 
- Extract tar file in a folder 
  ```
  $ mkdir carla
  $ cd carla
  $ tar -xvzf CARLA_XX.tar.gz)
  ```
- Then created conda env with python version : 3.7 (supported with carla version 0.9.11 )
- Install pygame in conda env
  ```
   $ pip3 install pygame
  ```
- Install numpy in conda env
  ```
  $ pip3 install numpy
  ```
- Install networkx in conda env
   ```
  $ pip3 install networkx
  ```
   
## Running Instructions : 
- Run the server of carla
```
 $ ./CarlaUE4.sh
```
- Create and run client file of carla simulator 
( We will run the script to get synchronized camera data with vehicle in automatic control, build with carla Python API)
- Go to pythonAPI folder / Examples
```
python3 sync_camera_auto_control.py
```

## Example code of sync camera data : 

![demo](docs/sync_auto_control.gif)

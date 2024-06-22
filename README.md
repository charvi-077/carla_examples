Working on Carla Simulator : Self Driving Cars Platform 

Installation Steps : 
- Carla Version : 0.9.11
- Download tar file of 4.xx GB 
- Extract tar file in a folder 
  ($ mkdir carla; cd carla) 
  ($ tar tar -xvzf CARLA_XX.tar.gz)
- Then created conda env with python3 version : 3.7 (supported with above carla 
version)
- Install pygame in conda env ($ pip3 install pygame)
- Install numpy in conda env ($ pip3 install numpy)

Running Instructions : 
- Run the server of carla 
 $ ./CarlaUE4.sh

- Create and run client file of carla simulator 
(For now we will run the available python examples build with carla Python API)
- Go to pythonAPI folder / Examples 
python3 automatic_control.py

Example code of sync camera data : 

![demo](docs/sync_auto_control.gif)

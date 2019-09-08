## A* algorithm on turtle-bot , Simulation on V-rep
### Workspace (The dimensions are in metres.)
![rrl_map](https://user-images.githubusercontent.com/47286001/62319448-fa93c780-b46b-11e9-9bae-3c0c7c333eb5.JPG)
1. Python libraries to be imported
	* math
	* sys
	* matplotlib.pyplot
	* time
2. Files need to be included in the same folder to have established Remote API bindings
	* make sure "vrep.py" and "vrepConst.py" are also present in the same folder as the execution code these files can be found at (programming\remoteApiBindings\python\python)
	* Check the V-REP version as "remoteApi.dll" is based on either 32-bit or 64-bit version (for windows .dll file is needed , for other OS, other compatible files needed to be placed in the same folder and can be obtained from "programming\remoteApiBindings\lib\lib")
	* V-REP scene (VREP_map.ttt)
	* To include turtlebot model in your V-REP, simply add the model file (Turtlebot2.ttm) inside the main folder of V-REP installation at the location "models\robots\mobile".

3. Parameters defined in the program
	* clearence - 0.60 m
	* clearence for the obstacle space using half planes ((diameter_of_robot/2)+clearence) = 77 cm
	* RPM 1 = 5 rad/sec
	* RPM 2 = 10 rad/sec
	* Length between wheels = 0.23 m
	* Time interval for generating nodes = 2 sec
  
4.  Steps to run the code
	* Origin is present at the center of the workspace
	* Open V-REP and place the turtle-bot at the start location with initial orientation and initiate the simulation
	* run the python file
		- "Final_code_Euclidean.py" in which cost to come calculated as the Euclidean distance
		- "Final_Code_Manhattan.py" in which cost to come calculated as the Manhattam distance i.e cost to come fixed in such a way that going straight (ul = ur) has slightly less cost than other actions, It made the path very smooth with the expense of slightly higher overall cost 
	* Enter the exact location (in meters) and orientation of the robot (in radians) , as it was placed in the V-REP
	* Scatter plot shows the graphical node exploration (color - green) and the path(color - red), and then robot follows that path generated on V-rep
  
5. There are 2 video samples along with the code with simulation
	* "[-4,1]_to_[3,-0.3].avi" with initial cordinate(-4,1) and goal at (3,-0.3) with initial orientation as 0 radians
	* "[-4,-4]_to_[0.7,3].avi" with initial cordinate(-4,-4) and goal at (0.7,3) with initial orientation as 0 radians


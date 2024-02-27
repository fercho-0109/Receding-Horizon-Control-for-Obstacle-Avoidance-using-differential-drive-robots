# Receding-Horizon-Control-for-Obstacle-Avoidance-using-differential-drive-robots

Code developed for "A. Marino, C. Tiriolo, - Receding Horizon Obstacle avoidance Strategy for Feedback Linearized Differential-Drive".  
Research "Concordia University".  

# Sumary.
This repository contains an implementation of a Receding Horizon Control to solve the obstacle avoidance problem in mobile robots. The robot used is a differential drive robot which is linearized by using I/O Feedback linearization. The key aspect of the control is the management of input constraints, which change across the linearization procedure. The simulation is performed using Matlab, and the validation of the controller is achieved by implementing the control in a Digital Twin of the Qbot2 robot provided by Quanser Company. this control implementation is based on [1] and is part of a master thesis of the owner of this repository.
# Problem Formulation.
![image](https://github.com/fercho-0109/Receding-Horizon-Control-for-Obstacle-Avoidance-using-differential-drive-robots/assets/40362695/9f7fbefa-5117-4dd1-876d-5752bcc7adad)

Let 𝒑(𝒌)=〖[𝑥(𝑡),𝑦(𝑡)]〗^𝑇be the planar position of the differential-drive robot at time k, 𝒑_𝒇=〖[𝑥_𝑓,𝑦_𝑓]〗^𝑇  the desired target location, and 𝓞_𝒇 (𝒌)  the obstacle-free region. Under the assumption that an obstacle-free path exists from 𝑝(0) to 𝑝_𝑓 , design a feedback control strategy  
[𝜔_𝑅,𝜔_𝐿 ]=𝑔(𝑝(𝑘),𝑝_𝑓,𝒪_𝑓 (𝑘))  
such that the robot is asymptotically driven to 𝑝_𝑓 , avoiding collisions and fulfilling the velocity constraint.




  
# Prerequisites
- The code was created and tested on the Matlab/Simulink 2023a environment
- Install Quanser interactive labs "Qlabs" (https://es.mathworks.com/matlabcentral/fileexchange/123860-quanser-interactive-labs-for-matlab). This requires a license to get access to the digital twins 
# File description
The repository contains two main folders  
1. **Matlab-simulation**: This folder contains a replication of the paper [[1](https://ieeexplore.ieee.org/document/9956741)] using Quanser enviroment. This is implemented using different linearization techniques according to the paper which is "Dynamic feedback linearization". Moreover, it contains the Matlab files to run the control. 
2. **Quanser-simulation**: This folder contains a replication of the paper [[1](https://ieeexplore.ieee.org/document/9956741)] as well, but this time using the Qbot 2e Digital Twin provided by Quanser.
### Bibliography  
[1] Cristian Tiriolo, Giuseppe Franzè, and Walter Lucia. An obstacle-avoidance receding horizon control scheme for constrained differential-drive robot via dynamic feedback linearization. In 2023 American Control Conference (ACC), pages 1116–1121, 2023.

# Simulations 
### For Matlab Simulation  
Download the respective folder called Matlab_simulation, Then for:
- Tracking problem: run "**Tracking_using_dynmaic_lin.m**" for Dynamic linearization
### For Quanser Simulation using the Qbot2e Digital Twin.
Download the respective folder called Quanser_simulation. Then, open the Quanser interactive labs and select Qbot 2e.

- Setup the position of the robot, go to Options - Change reset location - choose x=-0.25, y=-1.75, rotation=180 deg
- First, run "**Main_tracking_using_dyn_lin_Quanser.m**". To configure the parameters.
- Second, open and run the Simulink file "**Tracking_with_dynamic_lin.slx**" Then, the robot in the simulator should start to move and follow the trajectory that is the red line in the environment. If the connection with the simulator fails, close the simulator and open it again.

# Example to run an experiment  
**"Tracking Problem using Dynamic linearization"**
### Matlab simulation 
1. Download the folder. 
2. Run the Matlab file  "**Tracking_using_dynmaic_lin.m**"
3. The simulation should start showing the following result  
![image](https://github.com/fercho-0109/RHC-Tracking-Trajectory-with-Obstacle-Avoidance/assets/40362695/9da97de6-8f37-4604-bd6f-a36ef1451159)
### Quanser simulation
1. Download the folder
2. Open the Quanser interactive labs and select Qbot 2e.
3. Setup the position of the robot in the virtual environment Qlab, go to Options - Change reset location - choose x=-0.25, y=-1.75, rotation=180 deg
4. Run the Matlab file "**Main_tracking_using_dyn_lin_Quanser.m**"
5. Open and Run the Simulink file "**Tracking_with_dynamic_lin.slx**"
6. The Qbot 2e should start to move following the reference trajectory "red line"  
![image](https://github.com/PreCyseGroup/RHC-Tracking-Trajectory-with-Obstacle-Avoidance/assets/40362695/855b62e5-6ebd-4bf2-a85c-3464a9948a70)

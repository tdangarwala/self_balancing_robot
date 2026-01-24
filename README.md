# Self Balancing Robot (WIP)
The goal of this project is to build something like this: https://www.shaysackett.com/inverted-pendulum-robot/ - a 2 wheeled imverted pendulum. I want it to be able to handle poke forces, objects balancing on top, and balancing itself. It was mostly inspired out the desire to learn more about controls design. Maybe eventually this could be combined with my "robot follower" project that I started and inevitably never finished which was to build a robot that could follow me around.

I currently work at Speedgoat which is a company that builds the physical side of Mathworks - simulation hardware to go along with the models you create in Simulink. Model Based Design. I have some experience with this from my time at GM but it would be interesting to try and apply that here. So I will try to model the system and get the specs for the design (mass, max poke force it can handle, length, etc). And then actually design it - though I got excited and already bought components (based on the blog link I copied). 

Steps I will take:
1. Understand Dynamics of System
2. Design and Simulate Controller (design body in parallel)
3. Create Model of System and Controls
4. Assemble Robot
5. Testing and Refinement

# Requirements:
See Requirements.docx


Component Selection:
For this part I mostly just copied what was used by Shay Sackett in the blog post linked. Honestly, I got a bit excited early on and ordered parts on a whim, which may or may not have been correct for my project requirements. So, here I'll retroactivey I'll provide justification for why each piece is used and it's potential shortcomings. 

- Arduino MEGA 2560
    - Pros: provides enough IO to handle the 2 motor encoder inputs and IMU. Low power requirements. Cheap
    - Cons: Cannot handle the additional compute necessary for the expansion to the robot follower project which will require a vision system and path planning. 
- DC Motor 43.8:1 gear reduction
    - The requirements of the motor is that it is capable of providing a quick response to correct the pendulum.
    - Pros: Gear ratio provides the response necessary (proof later), encoder included for position control  
    - Cons: 
- Cytron Motor controller (2 inputs)
    - Pros: low cost and capable of managing 2 motors with easy API interface. Capable means does not provide additional delays in sending commands to motor. 
    - Cons:
- MPU 6050
    - Pros: I2C interface, provides all measurements needed for accurate control and modeling, low cost
    - Cons: Will require special care to ensure it is mounted and packaged properly to reduce noise

# Understanding Dynamics of System
I knew I wanted the robot to be able to balance itself and withstand a poke force input or an object placed on top. With that the following is what I think the force diagram for the robot looks like. 
<img width="679" height="419" alt="Screenshot 2025-12-21 at 5 40 23 PM" src="https://github.com/user-attachments/assets/c8d0e7de-3f09-4740-bede-9ef3ec2a30ac" />

I initially started by going the classical controls route and creating a transfer function of the system. The feedback loop would look like this:
<img width="794" height="349" alt="Screenshot 2025-12-21 at 6 01 37 PM" src="https://github.com/user-attachments/assets/8ee550f1-9745-47c1-8101-aa15b5b6dea2" />

The problem with this loop is that there is no x position control so the robot could just "run away" in an effort to stabilize itself. Here is a corrected diagram to include this:

[insert picture of loop here]

Creating the transfer function of this got cumbersome really quickly. 

I found a couple blog posts and youtube channels who approached this problem a different way - Lagrangian mechanics. This simplified the modeling of the dynamics significantly. Using the equation L = T - V, the following is the dervation of the state equation of the robot. 

Lagrangian mechanics is a pretty cool concept. The way I understand it is Lagrangian is another way to model the dynamics of a system similar to using Newton's Laws like what we learned in physics class. In its simplest sense, it's models the transfer of energy from kinetic (T) to gravitational (V). Using the Euler-Lagrange equations, we derive the nonlinear equations of motion directly from the energy functions. Later, we use the Jacobian to linearize these equations around equilibrium points for control design. 

You can think of it like water flowing down a hill. Using Newtonian you have to model every type of force acting on the water as it goes down. With Lagrangian you know at the top it's all gravitational energy and at the bottom it's kinetic and that's all you need!

To make this mostly about design and findings, please see the attached doc [... Derivation.docx] walking through the derivation. 

# Controller Design
The main considerations behind the controller choice is based on the nature of the problem we are looking to solve. The self balancing robot which is effectively an inverted pendulum is inherently unstable and multivariable. I was between PID and LQR (which I very recently learned about). PID works with only one input and one output and can be forced to be multi input multi output by stacking the loops. This would require balancing the impact of different gains across the system to ensure balancing, which becomes an extremely tedious task. LQR solves this by being a model based approach that updates multiple states at once. All of these states within the system are tied together, LQR represents this, PID does not. 

LQR is pretty cool. At it's fundamental it's a verion of dynamic programming which is taking the current state and finding the least cost option to get to the next state. The least cost option is defined as the best move to make to get to the goal state (balanced in this case). In the case of LQR the cost function is defined as a quadratic. 

I'll be using MATLAB or python to find the K matrix in u = -Kx. But I need to define the Q and R matrices. The Q matrix is the are the weights which define how important the different states are to the overall problem. The R matrix is weighting matrix that penalizes the control input. A simple way to understand this is Q describes what mistakes matter and how much and R describes how much control effort youre willing to use to prevent the mistakes. 

Next I need to figure out how to determine Q and R empirically. 

There's something called Bryson's rules which says that Qjj = 1/(max acceptable error)^2 and Rii = 1/(max acceptable input)^2. 

Max acceptable error per state:
x : 0.1 m (worst case is that the robot may be placed on a table and I dont want it falling off)
x dot : 0.1 m/s 
theta : 0.175 radians (10 degrees)
theta dot : 2 rad/s (made up)

This makes the diagonal of the Q matrix [100, 100, 32.65,0.25]. 

Next I need to simulate this to understand the performance and limitations of this robot. I'd also like to use this to find the ideal parameteres of the robot. Ideally I could use MATLAB but I dont have that so python will have to do. Python has the Control Systems Library which is perfect for this.

I want this simulation to allow the user to:
- input all relevant parameters for the robot (mass, length, wheel radius, etc)
- IMU modeling
- simulate the self balancing, poke force input (configurable), and object balancing on top (configurable) for 1 minute each
- summary metrics
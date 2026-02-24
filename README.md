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
- simulate the self balancing, poke force input (configurable), and object balancing on top (configurable) 
- summary metrics

Creating the simulation was a pretty cool learning experience. 

- I learned the physical interpretation of the different matricies. The A matrix effectively shows the "physics" of the system and how each state is impacted by it without considering any external factors. It describes that positon and velocity are related, as are angle an angular velocity. It also shows the gravity causes the angular velocity to go positive (robot tips forwards) and linear velocity moves in the opposite direction to counter that (you'll see this theme throughout this paragraph). In the B matrix it shows which way the robot will move when the input force from the motor is applied to it.  The same is expected of the E matrix I created to model the "finger poke" external disturbance. This is actually where I found an error in my math. When initially testing the code, I would see behavior where the states would stay at 0 until the end of the test where it would "explode" to some very large number. In my B matrix both the terms for the linear and angular acceleration had the same sign which means that pushing the wheels forward (linear) also tilts the body forward (angular) which is not correct. There were also some errors with the actual values due to math. The E matrix should be the same as the B since the disturbance should have the same impact as the motor inputs on the system. 

- I learned you can do 1/(max value)^2 to estimate values for the Q and R matrices. 

- I learned how to make the model more realistic. There were two ways this happened. One was the IMU noise. The sensor noise was used to make the states more realistic. This was done by taking into account the low pass filtering done by the MPU6050 and modeling the noise of the IMU with a gaussian distribution. The states being impacted by this noise are theta and theta dot only. The reason a guassian distribution makes sense is because the noise is the sum of many smaller noises (for lack of a better term). When you add those together you get a guassian distribution (central limit theorem). The theta is made noisy with the gaussian noise and a recursive low pass filter is used to smooth out the jitter, which is basically just a low pass filter that remembers the past. The full state is updated with this new theta value. The other was motor lag. This was accounted for by taking the difference of the commanded input (u_cmd) and the actual and factoring an estimated time constant of the motor. The actual motor command is a value that is iteratively created (starting from 0) and compounded on from there. This makes sense because in the physical world, nothing happens instantly - so effectively the motor is receiving an "older" command. 

- I also learned some of the basics that go into creating a simulation. You have the initialization step, a calculation step, and then a time step. Though my model is quite simple the core concepts of creating a simulation are there. One new aspect for me was the time step. There are a lot of different ways to do the time step. I used the integral which is the easiest one and it works well for smaller dts. There's also RK4, Backward Euler, and Adaptive Integrators - these are all suited for different dt and simulation needs. 

- I think I now understand what a determinant is. 

The next thing that's important to model is the nonlinear dynamics of the system. The mathematical derivation of that can be found in the Self Balancing Robot Derviations document. 

I want to make sure that the responses shown by these models are realistic. My way of gauging that is to see if the rise time of the xdot state in the nonlinear dynamics simulation is realistic based on the hardware I'm using. The rise time I'm getting right now is 0.06s which is way too fast. This means that the motor almost instantaneously outputs the maximum torque to try to stabilize the robot. Even if the motors could do this it will most likely lead to wheel slip and other adverse effects and negatively impact the robot's performance.  

I think I've gotten this to a good point where the controls can be confirmed that it is possible to control this thing. I now need to set up how the software is going to look. 

# Software 

At this point I think it's fair to start coding everything up. My goal is to try and get the code working with the hardware to the point where I just need to CAD the chassis, assemble it, and update the controller as applicable. The basic functionality such as tilting the MPU6050 causing the motors to spin should be good to go. 

Along the way, I want to learn a bit more about motors, how they work and how to test them. Luckily (sort of) the motors I bought were on the cheaper end and did not have a ton of data with it. I will try to learn more about them by gathering this data and then updating my simulation to see if it makes any difference. 







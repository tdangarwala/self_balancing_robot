# Self Balancing Robot (WIP)
The goal of this project is to build something like this: https://www.shaysackett.com/inverted-pendulum-robot/ - a 2 wheeled imverted pendulum. It was mostly inspired out the desire to learn more about controls design. Maybe eventually this could be combined with my "robot follower" project that I started and inevitably never finished which was to build a robot that could follow me around. 

Steps I will take:
1. Understand Dynamics of System
2. Design and Simulate Controller (design body in parallel)
3. Assemble Robot
4. Testing and Refinement

Component Selection:
For this part I mostly just copied what was used by Shay Sackett in the blog post linked. Here I'll provide justification for why each piece was picked. 

- Arduino MEGA 2560
- DC Motor 43.8:1 gear reduction
- Cytron Motor controller (2 inputs)
- MPU 6050

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

[insert derivation here]



# Controller Design

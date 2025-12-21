# Self Balancing Robot
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

[insert force diagram here]

I initially started by going the classical controls route and creating a transfer function of the system. The feedback loop would look like this:

[insert picture of loop here]

However, this got very cumbersome very quickly. 
I found a couple blog posts and youtube channels who approached this problem a different way - Lagrangian mechanics. This simplified the modeling of the dynamics significantly. Using the equation L = T - V, the following is the dervation of the state equation of the robot. 

[insert derivation here]
<img width="679" height="242" alt="Screenshot 2025-12-21 at 4 15 49 PM" src="https://github.com/user-attachments/assets/d41ec184-9335-4a02-b0d3-31ad295c9f21" />


# Controller Design

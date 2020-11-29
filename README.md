# Highly-Dynamic Movements of a Humanoid Robot Using Whole-Body Trajectory Optimization
Project management and files related to my master's thesis. 

The results of the thesis are organized in **several repositories**:

- [ma-thesis](https://github.com/julesser/ma-thesis): Thesis and final presentation
- [crocoddyl](https://github.com/julesser/crocoddyl): Open-source contributions (Contact Stability Constrained DDP)
- [ma-thesis-simulation-results](https://github.com/julesser/ma-thesis-simulation-results): Optimization-based whole-body motions for the RH5 Humanoid robot.
- [ma-thesis-experimental-results](https://github.com/julesser/ma-thesis-experimental-results): Online stabilization of the planned motions on the RH5 Humanoid robot.

![RH5 Humanoid Performing Multiple Jumps](https://github.com/julesser/ma-thesis/blob/master/fig/jumpObstacles/snaps/1x.png)

**Abstract**:

Motion planning for legged robots is a challenging problem and remains an open area
of research. Particular difficulties arise from effective underactuation, the mechanism
complexity, as well as nonlinear and hybrid dynamics. A common approach is
to decompose this problem into smaller sub-problems that are solved sequentially.
Recent research indicates that using a local optimal control solver, namely Differential
Dynamic Programming (DDP), produces more efficient motions, with lower
forces and impacts. <br>
This master’s thesis contributes in this direction by applying, evaluating and extending
DDP-based whole-body trajectory optimization, pursuing three objectives.
First, we develop a method for constraining DDP-like solvers in order to generate
inherently balanced motion plans. Second, the proposed motion planning approach
is evaluated for quasi-static and dynamic motions in a real-time physics simulation
and in real-world experiments on the lightweight and biologically inspired RH5
humanoid robot. Finally, the limits of the approach and the system design are
examined by solving highly-dynamic movements.

### Table of contents
Overview
* [[Overview: Terminology]]
* [[Overview: Data model]]
* [[Overview: Skill model]]

Tutorials
* [[Tutorial: Edit ontology]]
* [[Tutorial: launch system]]
* [[Tutorial: Create a skill]]
* [[Tutorial: Use world knowledge in a skill]]
* [[Tutorial: User interface]]

### What is SkiROS?
[[/imgs/skiros2_architecture.png | height=400px | align="center" ]]

The *Skill-based platform for [ROS](http://wiki.ros.org/) V2 (SkiROS2)* is a software designed to support programming of complex coordination schemes on robots equipped with several sensing and actuation functionalities. 

As such, SkiROS allows the end-user to specifying the task to perform in a compact way, while the platform infers the majority of parameters and acts as a coordinator node re-configuring the existing nodes in the ROS network and scheduling the execution of available functionalities in order to achieve the specified goal. 

This can be related to the planning and execution layer of a classical 3-layered robot control architecture.

Robots coordinated with SkiROS can be used in partially structured environments, where the robot has a good initial understanding of the environment, but it is also expected to find discrepancies, fail using initial plans and react accordingly.

### Why SkiROS?
SkiROS goal is to integrate together two fundamental processes necessary to coordinate complex systems operating in unstructured environments: planning and acting. 

_Task planning_ methods guarantee to find the best action to perform at any time, with a in-depth analysis of the actions' state space. This comes at the cost of an high computational time and subsequent slow reaction time. 
On the other hand, _acting_, intended as a context and task dependent reaction to events and refinement of actions into robot commands, can be achieved with scripted procedures encoded in models such as [Hierarchical Finite State Machines](http://wiki.ros.org/smach) or [Behavior Trees](http://wiki.ros.org/decision_making), at the cost of a lower generality with respect to a planning-based system. 

Within SkiROS, planning is mixed with Behavior Trees in order to not impact significantly on the system's reaction time. Task planning is invoked when (i) the acting mechanism is not able to cope with the run-time failures or (ii) when a task script cannot be specified before hand due to highly variable conditions (e.g. random orders coming from an external source). In the first case, planning ensures that the execution will be recovered from the most convenient point or that the robot will ask for human support while leaving the system in a secure state. In second case, planning can generate a complete Behavior Tree to fulfill a task. 

### What else?
Task planning and acting relies upon the constant monitoring of some relevant conditions in the environment, that allows to decide which is the most appropriate action to perform at any time. These condition variables form a base of knowledge that tends inevitably grow in complexity. In fact, the more information is stored in this knowledge base the more knowledgeable decisions can be taken. 

SkiROS provides a set of interfaces to extend the knowledge base and run reasoning algorithms to ground symbols to the real-world.
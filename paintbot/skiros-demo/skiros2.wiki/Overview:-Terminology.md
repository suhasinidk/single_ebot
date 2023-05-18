We do here a brief introduction to the most common terminology.

# Task model
### Behavior tree
BT is a formalism for hierarchical representation and execution of procedures. It is possible to have a good general introduction to behavior trees in the book [Behavior Trees in Robotics and AI: An Introduction](https://www.researchgate.net/publication/319463746_Behavior_Trees_in_Robotics_and_AI_An_Introduction). SkiROS implements an extension of this model, the extended behavior tree model (eBT) introduced in [this](https://www.researchgate.net/publication/321820909_Extended_behavior_trees_for_quick_definition_of_flexible_robotic_tasks) conference paper. 

The eBT model links BT procedures to a set of parameters, pre-conditions and post-conditions. In SkiROS we refer to these software constructs with the term _skills_. Relevant characteristic of our implementation are:

* Iterative tick-based execution, without blocking nodes
* Left-right processing (by default) using processors
* Conditions are not available, but are integrated into the nodes itsel

# Skill model

Skills, either primitives or compound skills, are parametric procedures that modify the world model (world state), bringing it from an initial state to a final state according to their pre- and post-conditions.

### Skill description
The skill description defines the action performed by a skill on a semantic level.
The description defines: 
* **parameters**, defining input and output for the skills and also used to apply conditions
* **pre-conditions**, conditions that must be valid when the skill starts execution
* **hold-conditions**, conditions that must be valid while the skill executes
* **post-conditions**, conditions that must be valid when the skill ends execution

Each skill must specify a description from which inherit these conditions and parameters.
The explicit skill description allows us to dynamically swap primitives or skills that change the world state in the same way without caring about the implementation details. Additional conditions can be applied later on to constrain further the utilization of a particular implementation.

Skill parameters make skills dynamic similar to function parameters.
We differentiate between required and optional parameters.
Note that parameters are propagated to the consecutive skills and can be accessed by their name.
Changes made to parameters are always persistent during the execution of a task.

Conditions come in two flavors: On the one hand, there are precondition that are used to evaluate if a primitive/skill should/can be executed. If any precondition is not met, the primitive/skill returns a failure. The failure status can be used in conjunction with _Processors_ to model more complex behavior.
On the other hand, there are post conditions that specify the target state after primitive/skill execution.
Both pre- and post-conditions are used to plan tasks.

### Primitive skill
Primitive skills are atomic procedures in SkiROS.
They implement the code that actively changes the world model (e.g. by physically interacting with the environment).
Typical examples include MoveArm, NavigateTo, GripperCtrl, Locate and many more.

### Compound skill
Compound skills can be used to model more complex behaviors by combining primitives in a hierarchical behavior tree.
Compound skills are composition of 2 or more skills, that are coordinated with the policy defined by the [[processor |Overview: Processors]].
An example could be a simple pick skill, defined as a sequence of MoveArm and Grasp.

To continue with the introduction to skills, go to [[Overview: Skill model]].

***

# Data model

### Ontology
The ontology defines the knowledge domain for a robotic system.
It is a database of categories and concepts which describe the elements of the domain of interest, their properties, and their relations between each other.
Ontologies are defined in Description Logic (DL), a specialization of first-order logic, which is designed to simplify the description of definitions and properties of categories.

SkiROS comes with a predefined dataset described [[here | https://github.com/RVMI/skiros2/wiki/Overview%3A-Data-model]]. 

The base ontology can be modified and extended to the user's need, e.g. to define a new domain or new robot components. This procedure is described in detail [[here | Tutorial: Edit ontology]].
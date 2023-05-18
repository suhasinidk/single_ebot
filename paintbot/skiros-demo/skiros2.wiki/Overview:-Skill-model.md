### Skills
Skills, either [[primitives | Overview: Skill model#primitive skill]] or [[compound skills | Overview: Skill model#compound skill]], are parametric procedures that modify the [[world model | Overview: Data model#world model]]  (world state), bringing it from an initial state to a final state according to their pre- and post-conditions.

```python
from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_skill.core.skill import SkillBase

class MySkillDescription(SkillDescription):
    """Definition of a skill/primitive descriptor"""
    def createDescription(self):
        """Add parameters and conditions"""
        pass

class my_primitive(PrimitiveBase):
    """Definition of a skill primitive"""
    def createDescription(self):
        """Set primitive type"""
        self.setDescription(MySkillDescription())

    def execute(self):
        """Do stuff. Return progress using step/error/success functions."""
        return self.success("Done")

class my_skill(SkillBase):
    """Definition of a compound skill"""
    def createDescription(self):
        """Set skill type"""
        self.setDescription(MySkillDescription())
        
    def expand(self, skill):
        """Define compound skill tree"""
        pass
```


### Skill description

The skill description defines the action performed by a skill on a semantic level.
The description defines: 
* **parameters**, defining input and output for the skills and also used to apply conditions
* **pre-conditions**, conditions that must be valid when the skill starts execution
* **hold-conditions**, conditions that must be valid while the skill executes
* **post-conditions**, conditions that must be valid when the skill ends execution

Each skill must specify a description from which inherit these conditions and parameters.
The explicit skill description allows us to dynamically swap primitives or skills that change the world state in the same way without caring about the implementation details. 

Skill descriptions have to be derived from the abstract ```SkillDescription``` class.

The simplest case for a skill description neither defining parameters nor conditions:
```python
from skiros2_skill.core.skill import SkillDescription

class MySkillDescription(SkillDescription):
    def createDescription(self):
        pass
```

### Skill parameters

```python
from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element

class MySkillDescription(SkillDescription):
    def createDescription(self):
        self.addParam("StaticBool",     True,                                 ParamTypes.Required)
        self.addParam("RequiredInt",    int,                                  ParamTypes.Required)
        self.addParam("RequiredObject", Element("skiros:TransformationPose"), ParamTypes.Required)
        self.addParam("DefaultString",  "My default value",                   ParamTypes.Optional)
        self.addParam("OptionalList",   list,                                 ParamTypes.Optional)
        self.addParam("OptionalObject", Element("skiros:TransformationPose"), ParamTypes.Optional)
```

This code defines a new skill description without any pre- and post-condition (a similar example is contained in [skiros2_template_lib](https://github.com/ScalABLE40/skiros2_template_lib)).
A skill without pre- and post-conditions can be executed, but will not be considered for planning.
Note that parameters are propagated to the consecutive skills and can be accessed by their name.
Changes made to parameters are always persistent during the execution of a task.

Parameters come in two flavors:
* **Required parameters** (`ParamTypes.Required`) are parameters that are required to execute the skill.
The parameter value can either be a data type such as `string`, `int`, `float`, `bool`, `list`, `dict`, a specific value, or a [[world model element | Overview: Data model#elements]] `Element(`[[concept | Overview: Data model#ontology]]`)`.
If a specific value is used (for data types only), it overwrites any existing parameter with the same name and therefore becomes a static parameter during the execution of the task.
This can e.g. be used to implement two skills based on the same primitive with different configurations such as a open and close skill for a gripper based on the same gripper primitive.
In other cases, the parameter has to be specified through the GUI or by a previously executed primitive/skill that defines the parameter.
If the parameter is not present, the skill execution will result in an error.
* **Optional parameters** (`ParamTypes.Optional`) do not have to be specified during execution time.
These parameters are used as optional input parameters (not required for execution) and/or output parameters (changed or created during primitive/skill execution).
Similar to the configuration parameters, the parameter value can either be a data type such as `string`, `int`, `float`, `bool`, `list`, `dict`, a specific value, or a [[world model element | Overview: Data model#elements]] `Element(`[[concept | Overview: Data model#ontology]]`)`.
However, if a specific value is used, it is handled as a default value for the parameter.
It is only set if the parameter is not present in the system yet.
In other cases, either existing parameter values are used (optional input), specified through the GUI or a previously executed skill, or the value has to be specified in the skill (output) by simply changing properties of the world model element.

Parameters are defined by using the `SkillDescription` member function `addParam`.
The function takes three arguments: The parameter name (id), the parameter value and the parameter type.

In this example, the skill description defines six parameters using the command `addParam`:
1. _"StaticBool"_ is a static parameter. Every time the skill/primitive is executed, the parameter is set to `True`.
2. _"RequiredInt"_ is a required parameter of type `int`. You have to make sure it is set before the primitive/skill is executed.
3. _"RequiredObject"_ is a required world model element based on the concept `skiros:TransformationPose`. It should be set to an actual instance before execution. Editing the properties of the element are reflected in the global world model.
4. _"DefaultString"_ is an optional parameter of type `string` with a default value. If the parameter is not specified, the default value is used.
5. _"OptionalObject"_ is an optional parameter of type `list`. It can be used as an optional parameter, or as an output parameter where the value has to be specified within the primitive/skill.
6. _"RequiredObject"_ is an optional world model element based on the concept `skiros:TransformationPose` and behaves analogously to parameter 5.

When creating a primitive or skill, defined parameters can be accessed using the member variable (dictionary) `params` of the skill\primitive class:
```python
value = self.params["<paramName>"].value
self.params["<paramName>"].value = new_value
```

### Skill conditions
A condition can be used as:
* **pre condition**, to evaluate if a skill should/can be executed. If any precondition is not met, the skill returns a failure. The failure status can be used in conjunction with [[Processors | Overview: Processors]] to model more complex behavior.
* **hold condition**, to verify if a running skill can continue to be executed. They are similar to pre-conditions, but they are checked at every tick, rather than only on start.
* **post condition**, to specify the target state after primitive/skill execution. They are ignored during execution.

All conditions are used to plan tasks. 

Condition are defined in the module `skiros2_common.core.conditions`.
Here, usually only 4 conditions are required to define skill descriptions:
* `ConditionHasProperty(name, property, parameter, is_true)`: Check if a parameter has a specified property.
* `ConditionProperty(name, property, parameter, operator, value, is_true)`: Compare a specified property of a parameter to a value.
* `ConditionRelation(name, relation, subject, object, is_true)`: Check if a parameter (subject) has a specified relation with another parameter (object).
* `ConditionAbstractRelation(name, relation, subject, object, is_true)`: Check if a relation between parameters is consistent with predefined relations on a conceptual level. ("Should the subject and object have the specified relation?")

```python
from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.conditions import ConditionHasProperty, ConditionProperty, ConditionRelation, ConditionAbstractRelation
from skiros2_common.core.world_element import Element

class Locate(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Camera",    Element("skiros:Camera"),   ParamTypes.Required)
        self.addParam("Container", Element("skiros:Location"), ParamTypes.Required)
        self.addParam("Object",    Element("skiros:Product"),  ParamTypes.Optional)
        #=======PreConditions=========
        self.addPreCondition(ConditionRelation("RobotAt", "skiros:at", "Robot", "Container", True))
        self.addPreCondition(ConditionAbstractRelation("ContainerForObject", "skiros:partReference", "Container", "Object", True));
        self.addPreCondition(ConditionProperty("NotEmpty", "skiros:ContainerState", "Container", "=", "Empty", False))
        #=======PostConditions=========
        self.addPostCondition(ConditionRelation("InContainer", "skiros:contain", "Container", "Object", True));
        self.addPostCondition(ConditionHasProperty("HasPosition", "skiros:Position", "Object", True))
```
In this example, a locate skill description is defined.
It has three parameters:
Two required parameters specifying the camera to use and the location (container) where to locate an object.
The last parameter is an optional parameter and specifies the type of object to be located.
At the same time, the parameter is also used as an output.
That is, the locate skill finds an object and changes the parameter to an actual instance.

Additionally, the description has three pre-conditions:
The first one validates if the robot is at the correct location (container).
The second one checks if the container can actually hold the specified object type.
This is mainly used for the planning domain.
The third condition makes sure that the container is not empty.

Furthermore, we define the target state by two post-conditions:
The first one states that the container now contains the located object.
The second one makes sure that the object has a valid spatial position.


### Primitive skill

Primitive skills are atomic actions in SkiROS.
They implement the code that actively changes the world model (e.g. by physically interacting with the environment).
Typical examples include MoveArm, Drive, GripperCtrl, Locate and many more.
Primitives have to be derived from the abstract base class `PrimitiveBase` and set the appropriate description in the `createDescription` function.
The following code defines a new primitive implementation of `MySkillDescription`:

```python
from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.primitive import PrimitiveBase

class MyPrimitive(SkillDescription):
    def createDescription(self):
        self.addParam("Boolean", False, ParamTypes.Required)

class my_primitive(PrimitiveBase):
    def createDescription(self):
        """Set the primitive type"""
        self.setDescription(MyPrimitive())

    def onInit(self):
        """Called once when loading the primitive. If return False, the primitive is not loaded"""
        return True

    def onPreempt(self):
        """ Called when skill is requested to stop. """
        pass
    
    def onStart(self):
        """Called just before 1st execute"""
        return True

    def onEnd(self):
        """Called just after last execute"""
        pass

    def execute(self):
        """ Main execution function """
        return self.success("Done")
```
The functions `createDescription` and `execute` are mandatory and have to be implemented, whereas the other functions (`onXXX`) are optional.
The `execute` function is called several times during the skill execution (on average at 25hz) and allow the skill to sync its parameters with other skills.
Therefore, `execute` has to return the current progress status (running, succeeded, failed) of the primitive by returning the value created by one of the member functions `step(msg)`, `success(msg)` or `fail(msg, code)`.

**Note**: execute must return as fast as possible and _never_ execute blocking tasks.

This behavior gives rise to two ways of implementing primitives:
On the one hand, algorithms can be implemented directly in the `execute` function.
This is mostly useful for iterative algorithms that do not require a lot of processing time.
On the other hand, the functionality can be outsourced to a separate thread or procedure, e.g. using an action server or a service.
This allows us to define primitives that require more time for execution or the integration of third party nodes.


### Compound skill

Compound skill are used to design more complex behavior based on multiple skill primitives.
A compound skill defines a tree structure where its children are either primitives or compound skills themselves, thereby defining a tree structure (hence: behavior tree).

The following code defines a new compound implementation of `MySkillDescription`:

```python
from skiros2_skill.core.skill import SkillDescription, SkillBase, SerialStar

class MySkill(SkillDescription):
    def createDescription(self):
        pass

class my_skill(SkillBase):
    def createDescription(self):
        self.setDescription(MySkillDescription())
        
    def expand(self, skill):
        skill.setProcessor(SerialStar())
        skill(
            self.skill("MyPrimitive", "my_primitive", specify={"Boolean": True}),
        )
```
The `createDescription` has the same functionality has seen for primitive skills, while all the other functions are replaced by the `expand`. 

In the expand function the user can specify how the compound skill decomposed into a behavior tree. 

By default, the tree uses the Serial processor, traversing and executing sequentially from left to right. The `setProcessor` changes the default processor, in this case with the SerialStar. Find [[here | Overview: Processors]] a complete description of processors and their behavior.  

The skill () operator allows to set the skill children. To add several children at once it is possible to use the syntax:

```python
skill(
    self.skill(child1type, child1label),
    self.skill(child2type, child2label),
    ...,
    self.skill(childNtype, childNlabel)
)
```

`self.skill(type, label="", remap={}, specify={})` is a placeholder for a skill which is replaced at run-time with an available implementation. For more details about this, go to the [[skill creation tutorial | Tutorial: Create a skill]]. 
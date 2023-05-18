Before following this tutorial you have gone through the [[skill model overview | Overview: Skill model]].

This tutorial explains how to create a skill from scratch using the default ROS workspace.

In the first part, we are creating a new skill package in the workspace and setting up the required files.  
In the second part, we develop a simple skill.  
The last part is concerned with integrating the developed skill into the system or testing it stand-alone.  

# Part 1: Package configuration

First, we have to navigate to the base directory for your skill.
This can be any folder in your catkin workspace, in our case, we we use the skill folder in the ROS workspace.

```sh
roscd
cd ../src/skills
```

Here, we create a new catkin package (`skills_sandbox`) that will contain all our newly developed skills.
The package requires the following dependencies:
   * build dependencies: `rospy`, `skiros2_msg`
   * runtime dependencies: `rospy`, `skiros2_msgs`, `skiros2_common`, `skiros2_resource`, `skiros2_skill`

You can create the catkin package by running:
```sh
catkin_create_pkg skills_sandbox rospy skiros2_msgs
```

After creating the package, you need to add the runtime dependencies in the `package.xml` file.
Stripped down, the file should look similar to this:
```xml
?xml version="1.0"?>
<package format="2">
  <name>skills_sandbox</name>
  <version>0.0.0</version>
  <description>The skills_sandbox package</description>
  <maintainer email="noname@todo.todo">noname</maintainer>
  <license>TODO</license>

  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>rospy</build_depend>
  <build_depend>skiros2_msgs</build_depend>
  <build_export_depend>rospy</build_export_depend>
  <build_export_depend>skiros2_msgs</build_export_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>skiros2_common</exec_depend>
  <exec_depend>skiros2_msgs</exec_depend>
  <exec_depend>skiros2_resource</exec_depend>
  <exec_depend>skiros2_skill</exec_depend>
</package>
```

Furthermore, we have to enable the python setup in the `CMakefile` by uncommenting `catkin_python_setup()`.
Stripped down, the file should look similar to this:
```cmake
cmake_minimum_required(VERSION 2.8.3)
project(skills_sandbox)

find_package(catkin REQUIRED COMPONENTS rospy skiros2_msgs)
catkin_python_setup()
catkin_package()
include_directories(${catkin_INCLUDE_DIRS})
```

Additionally, you need to create a python setup file that registers your python packages.
In your main folder, create a file named `setup.py` that should have the following format:
```python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['<your_package_1>', '<your_package_2>', '<...>'],
    package_dir={'': 'src'},
)
setup(**setup_args)
```

Note that you have to add each python package in your src folder here.


Finally, your can create a new python package for your skill(s), e.g. test_skill.
For the sake of understanding, we create three separate files here containing the skill descriptions, the skill primitives and the compound skills.
Note that you can also simply using a single file.

```sh
mkdir test_skill
cd test_skill
touch __init__.py

touch descriptions.py
touch primitives.py
touch skills.py
```

Do not forget to add `test_skill` to the packages in the setup file.


# Part 2: Implementing primitives

Implementation of `descriptions.py`:
```python
from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.params import ParamTypes

class TestPrimitive(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Force", 0.0, ParamTypes.Required)

class TestSkill(SkillDescription):
    def createDescription(self):
        pass
```

Implementation of `primitives.py`:
```python
from skiros2_common.core.primitive import PrimitiveBase
from descriptions import TestPrimitive

class test_primitive(PrimitiveBase):
    def createDescription(self):
        self.setDescription(TestPrimitive(), self.__class__.__name__)

    def onPreempt(self):
        return self.success("Stopped")

    def onInit(self):
        return True

    def onStart(self):
        self.index = 0
        f = self.params["Force"].value
        print "Send force {}".format(f)
        return True

    def execute(self):
        self.index += 1
        if self.index > 10:
            return self.success("Done")
        return self.step("")
```
This example skill just initializes an internal state index in the onStart, that is increased at every tick in the execute function. After the index reaches 10, the skill returns success, otherwise returns step (running).

# Part 3: Create a simple skill

Implementation of `skills.py`:
```python
from skiros2_skill.core.skill import SkillBase, Selector, Serial, State
from descriptions import TestSkill

class test_skill(SkillBase):
    def createDescription(self):
        self.setDescription(TestSkill(), self.__class__.__name__)

    def expand(self, skill):
        skill.setProcessor(Serial()) #SerialStar, Selector, ParallelFf, ParallelFs
        skill(
              self.skill("TestPrimitive", ""),
              self.skill("TestPrimitive", "", specify={"Force": 1.0})
        )
```

`skill.setProcessor` set the processor used to tick the children. Find all available processor [[here | Overview: Processors]].

`self.skill(type, label="", remap={}, specify={})` is a placeholder for a skill which is replaced at run-time with an available implementation. The parameters are in particular:

* `type` (mandatory) - defines the skill type (e.g. "MoveArm") or a processor type (e.g. SerialStar(), see example below)
* `label` (optional) - defines a specific implementation label, that will be prioritized.
* `remap` (optional) - defines parameters remappings (e.g. {"Object": "LocalizationOutput"}). This is useful when you to link the parameter value to the output generated from other skills.
* `specify` (optional) - set the value of some parameters (e.g. {"Speed": 1.0, "Force": 0.1}). Note: when a parameter value is set in this way, it becomes static.

Now we have just seen a skill with a single layer of hierarchy, but it is possible to define a complex skill, with several layers. For example:

```python
from skiros2_skill.core.skill import SkillBase, Selector, Serial, State
from descriptions import TestSkill

class test_skill(SkillBase):
    def createDescription(self):
        self.setDescription(TestSkill(), self.__class__.__name__)

    def expand(self, skill):
        skill.setProcessor(ParallelFs()) #SerialStar, Selector, ParallelFf, ParallelFs
        skill(
              self.skill("Timer", "timer", specify={"Timeout": 10.0}),
              self.skill(SerialStar())(
                   self.skill("TestPrimitive", ""),
                   self.skill("TestPrimitive", "", specify={"Force": 1.0})
              )
        )
```

In this example, the skill is composed of a timer running in parallel with 2 test primitives, executed in sequence. ParallelFs (first success) stop with success as soon as 1 skill returns success, so in this case if the sequence of test primitives is still running after 10 seconds, the test_skill still stop successfully, assuming that the timer returns Success when it reaches the Timeout.

# Part 4: Skill integration

Have a look how to configure and launch SkiROS with your skills [[here | Tutorial: launch system]].


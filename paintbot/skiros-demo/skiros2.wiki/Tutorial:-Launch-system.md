In this tutorial we describe the parameters necessary to launch the main SkiROS nodes. All launch files are contained in `skiros2/skiros2/launch` folder.

## Complete SkiROS system

[[/imgs/skiros2_architecture.png | height=400px | align="center" ]]

The complete SkiROS system can be launched with the following:

```roslaunch skiros2 skiros.launch```

The launch file starts 4 main nodes: world model, skill manager, task manager, and GUI. 

By default, the skill manager doesn't load any skill. See below how to load skills on the skill manager.

## World model server

This node stores the ontology and the world model runtime instance.

```  
<node launch-prefix="$(arg prefix)" name="wm" pkg="skiros2_world_model" type="world_model_server_node" respawn="true">
    <param name="workspace_dir" value="$(find my_package)/owl" />
    <arg name="init_scene" value="my_scene.turtle"/>
    <arg name="verbose" value="false"/>
    <arg name="reasoners_pkgs" value="[skiros2_std_reasoners]"/>
    <arg name="debug" value="$(arg debug)"/>
</node>
```
The parameters are the following:
* **workspace_dir**, defines the workspace folder. All the ontology files (.owl) in the folder will be loaded in the world model (see [[Tutorial: Edit ontology]] for more info). The workspace is also use as folder to load and save the run-time scene.
* **init_scene**, a scene to load at boot. The file must be in the directory specified in workspace_dir
* **verbose**, if true more debug message will be printed
* **reasoners_pkgs**, id of ROS packages containing reasoners for the world model. 
* **debug**, if true launches the node in the python debugger

## Skill manager

This node load the skills related to a specific robotic system and offers services to start and stop them and monitor their execution.

```
  <include file="$(find skiros2)/launch/skill_mgr.launch">
      <arg name="verbose" value="false"/>
      <arg name="debug" value="false"/>
      <arg name="libraries_list" value="[my_package_name]"/>
      <arg name="primitive_list" value="[my_primitive1, my_primitive2]"/>
      <arg name="skill_list" value="[my_skill1, my_skill2]" />
      <arg name="robot_ontology_prefix" value="my_ontology"/>
      <arg name="robot_name" value="my_robot"/>
  </include>
```

Where the parameters are:
* **verbose**, if true more debug message will be printed
* **debug**, if true launches the node in the python debugger
* **libraries_list**, names of packages defining skills or primitives
* **primitive_list**, names of primitive skills to load. 
* **skill_list**, names of compound skills to load
* **robot_ontology_prefix**, the name of the ontology defining the robot model (see [[Tutorial: Edit ontology]] for more info)
* **robot_name**, the robot model to load in the world model
* **deploy**, if false, run the node in a stand-alone xterm

## Task manager

This node offers an action server to request a task planning. It automatically uses the world model instance as starting state and the available skills to setup the planning domain. 

```
  <node launch-prefix="$(arg prefix)" name="tm" pkg="skiros2_task" type="task_manager_node" respawn="true">
    <param name="verbose" value="$(arg verbose)" />
  </node>
```
Where the parameters are:
* **verbose**, if true more debug message will be printed

## GUI launch

The GUI offers an user interface to edit the world model and control the skill execution. It currently support the execution of only one skill at a time. It is implemented as a plugin for rqt. The launch code looks like the following:
```
<node name="skiros_gui" pkg="rqt_gui" type="rqt_gui" args="-s gui.skiros" output="screen" />
```


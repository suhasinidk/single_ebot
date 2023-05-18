## Ontology
An ontology defines the knowledge domain for a robotic system.
It is a database of categories and concepts which describe the elements of the domain of interest, their properties, and their relations between each other.
Ontologies are defined in Description Logic (DL), a specialization of first-order logic, which is designed to simplify the description of definitions and properties of categories.
The ontology can be easily modified and extended to the user's need, e.g. to define a new domain or new robot components.
For a better introduction to ontologies we suggest to follow the tutorials in [link](
https://protegewiki.stanford.edu/wiki/Main_Page).

The ontologies can be layered, so that the base ones define very abstract and general concepts, meanwhile the top ones define very precise concepts, related to a specific domain. 

The SkiROS ontology is based on the Standard IEEE Std 1872™-2015 [link](http://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=7084073).
The Standard IEEE Std 1872™-2015 is a core ontology that specifies the main, most general concepts, relations, and axioms of robotics and automation (R&A). It is intended as a reference for knowledge representation and reasoning in robots, as well as a formal reference vocabulary for communicating knowledge about R&A between robots and humans. This standard is composed of a core ontology about R&A, called CORA, together with other ontologies that give support to CORA.

The SkiROS ontology extends the IEEE standard with a data model for the pick and place domain.

To see how to visualize and modify an ontology or create a new one, follow the [[Tutorial: Edit Ontology]]. 

## World model

A world model is a semantic representation of the work environment including relevant objects and the robot itself. Theses world model elements (objects with parameters) are defined explicitly in an ontology.
The world model represents the scene by spatially linking available elements resulting in a hierarchical scene graph. World model node can be executed with `roslaunch skiros2 world_model_server.launch`. Then it is possible to use the python interface to get, e.g., the current scene:

```python
import skiros2_world_model.ros.world_model_interface as wmi

wmi_inst = wmi.WorldModelInterface()

(scene, scene_id) = wmi_inst.get_scene()
for e in scene:
    print e.printState()
```

Note: skills already come with an instantiated interface, see [[Tutorial: Use world knowledge in a skill]] for how to communicate with the world model in a skill.

## Elements

The Element class represents objects contained in the world model. It is a tuple with: id, type, label, properties and relations. `Type` and `label` map the object to an individual in the ontology, the `id` is a unique string identifier of the object in the scene, `properties` is a list of tuples (key, value, datatype) and `relations` is a list of tuples (src, type, dst) where src and dst are two Element ids and type is the type of relation (e.g. skiros:contain). src or dst can contain "-1", that is considered as the self-reference value.

```python
from skiros2_common.core.world_element import Element

pose_element = Element("skiros:TransformationPose")
```
Before following this tutorial, make sure your went through the data model overview, skill overview and skill tutorial.

The world model is a core tool in the skill developer's toolbox to store and organize the necessary knowledge.

As introduced in the data model overview, the world model is a graph where nodes are Elements and edges are relations between them.

# World model interface

Skills contain an instance of the world model interface that can be accessed using `self.wmi`.

## Element parameter

An Element can be included as skill parameter. Lets take as example the _follow_pose_ tutorial in _skiros2_lib_test_:

```python
class PoseHandler(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Pose", Element("skiros:TransformationPose"), ParamTypes.Required)
        self.addParam("Direction", 0, ParamTypes.Required, description="x: 0, y: 1, z: 2")

```

Here a parameter `Pose` is defined as an Element of type _skiros:TransformationPose_. The type is important as it defines which Elements are accepted in input. The most general type is _sumo:Object_, that allows to parameterize the parameter with any element in the world model. Using more specific types constraints more the selection. The hierarchy of types is described in the SkiROS ontology, for more details refer to the [[data model overview | Overview: Data model]].

## Access element properties

Each element contains a set of properties (e.g. position, orientation, size, etc). A short example of a skill that sets an element property:

```python
from skiros2_common.core.world_element import Element

class linear_mover(PrimitiveBase):
    def createDescription(self):
        self.setDescription(PoseHandler(), self.__class__.__name__)

    def execute(self):
        pose_element = self.params["Pose"].value
        pose_element.setProperty("X", 10.0)
        x = pose_element.getProperty("X").value
        self.params["Pose"].value = pose_element
        return self.success("Set property X to {}.".format(x))
```
**NOTE**: the element is automatically synchronized with the world model server in the background, but for more advanced interaction you can use the world model interface.

## Individuals, template and scene elements

An individual is an instance of a class in the ontology. We subdivide individuals into template and scene elements. A template element is an ontology individual that is not instantiated in the scene. 

```python
    def execute(self):
        individuals = self.wmi.get_individuals()
        return self.success(individuals)
```

The command `get_individuals` retrieves the names of all the individuals of a particular class, both template and scene elements. The following code transforms the names into Element classes:

```python
    def execute(self):
        individuals = self.wmi.get_individuals()
        s = []
        for e in individuals:
            if self.wmi.is_scene_element(e):
                s.append(self.wmi.get_element(e))
            else:
                s.append(self.wmi.get_template_element(e))
        return self.success("{}".format(s))
```
`is_scene_element` verify if the name refers to an instance or a template, `get_element` and `get_template_element` return the Element class for respectively scene and template elements.

## Access element relations

A relation binds the element to other elements in the scene or template elements. They are expressed as a triple (subject -  predicate - object) and saved as a dictionary `{'src': subject_id, 'type': predicate, 'dst': object_id, 'state': True/False, 'abstract': True/False}`. 

The following code retrieves element relations and prints them out:

```python
    def execute(self):
        pose_element = self.params["Pose"].value
        relations = pose_element.getRelations()
        return self.success("{}".format(relations))
```

To set a relation:

```python
    def execute(self):
        pose_element = self.params["Pose"].value
        pose_element.setRelation(target.id, "skiros:contain", "-1")
        self.params["Pose"].value = pose_element
        return self.success("Changed element position.")
```

# Reasoners

Sometimes it is necessary to handle several correlated properties. This requires to define some methods for e.g. conversion or to reason on the data. We collect these methods into discrete reasoners. 

## Handling spatial properties
At the moment the only reasoner included in SkiROS is the spatial reasoner `AauSpatialReasoner`, handling spatial properties.

It is defined in the _skiros2_std_lib_ package. The most relevant properties handled by the spatial reasoner are: PositionX, PositionY, PositionZ, OrientationX, OrientationY, OrientationZ, OrientationW, FrameId, BaseFrameId and LinkedToFrameId. 

The spatial reasoner helps to convert complex data types into the right Element property format. The use of reasoner's conversions can be accessed using the `setData` and `getData` functions, to respectively set or retrieve some properties in a specific format. For example: 

```python
    def execute(self):
        obj = self.params["Object"].value
        ref = self.params["ReferencePose"].value
        obj.setData(":PoseStampedMsg", ref.getData(":PoseStampedMsg"))
```
In this example, a skill is retrieving two Elements from its parameters and then is setting the Pose of the Object equal to the Pose of the ReferencePose, using `PoseStampedMsg` as data format. This implies retrieving and setting several properties at once. 
The complex datatypes handled by the spatial reasoner are:

* **:TransformMsg**, Transformation in format geometry_msgs/Transform
* **:PoseMsg**, Pose in format geometry_msgs/Pose
* **:Pose**, Pose in format list(Position, Orientation)
* **:PoseStampedMsg**, Pose with frame id in format geometry_msgs/PoseStamped
* **:Position**, Position in format list(x, y, z), unit: meters
* **:Orientation**, Orientation in format list(x,y,z,w), unit: quaternion
* **:OrientationEuler**, Orientation in format list(x, y, z), unit: degrees
* **:Size**, Sizein format list(x, y, z), unit: meters

Other examples on usage of spatial reasoners can be found in the _follow_pose_ tutorial in _skiros2_lib_test_. For example:

```python
class linear_mover(PrimitiveBase):
    def createDescription(self):
        self.setDescription(PoseHandler(), self.__class__.__name__)

    def execute(self):
        pose = self.params["Pose"].value
        direction = self.params["Direction"].value
        position = pose.getData(":Position")
        position[direction] = position[direction] + 0.1
        pose.setData(":Position", position)
        self.params["Pose"].value = pose
        if self._progress_code<10:
            return self.step("Changing position to: {}".format(position))
        else:
            return self.success("Done")

```
This skill is increasing at each tick one component of the position of _Pose_ element, based on the defined _Direction_. After 10 ticks the skill returns success.

## TF integration

World model elements are seamlessly integrated into the ROS tf system: if an element has a defined pose, this is published on tf at a rate of 25hz with frame id equal to the object ID (the format is skiros:<TYPE>-<ID_NUMBER>).

To stop publishing the transformation on tf, set the property `skiros:PublishTf` to False.

If you want an element to track a transformation published on TF (i.e. always be equal to another transformation), you can set the property `skiros:LinkedToFrameId` equal to the frame you want to track (e.g. base_link). The pose of the element then will be regularly updated at a rate of 25hz. 

Finally, if you want to publish the transformation under a different name, you can set the property `skiros:PushToFrameId`.

In a skill you can also get the reasoner to transform the element into different base frames. For example:
```python
    def execute(self):
        pose = self.params["Pose"].value
        if not self.wmi.get_reasoner("AauSpatialReasoner").transform(pose, "another_frame"):
            return self.fail(-1, "Failed to transform.")
        else:
            object_pose = pose.getData(":Pose")
            return self.success("Successfully transformed: {}".format(object_pose))
```
In this case, the pose Element will be directly modified. Alternatively user can run:
```python
    def execute(self):
        pose = self.params["Pose"].value
        pose_frame_id = pose.getProperty("skiros:FrameId").value
        object_pose = self.wmi.get_reasoner("AauSpatialReasoner").get_transform(pose_frame_id, "another_frame", rospy.Duration(0.25))
        if object_pose[0] is None:
            return self.fail("Failed to transform.", -1)
        else:
            return self.success("Successfully transformed: {}".format(object_pose))
```

**!!Attention!!**: the parent of the Element in the world model has priority on the pose BaseFrameId! This means that the Element pose will be converted to the parent frame if the skiros:BaseFrameId property doesn't match. This also means that you can normally move objects in the world model without having to manually transform all the time.
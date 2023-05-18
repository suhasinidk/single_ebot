
from skiros2_skill.core.skill import SkillDescription, SkillBase, Serial, ParallelFf, ParallelFs, Selector, Sequential
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element
from skiros2_common.core.conditions import ConditionRelation

import numpy as np

#################################################################################
# NavigateToGoalDescription
#################################################################################

class NavigateToGoalDescription(SkillDescription):
    def createDescription(self):
        self.addParam('Start', Element('skiros:Location'), ParamTypes.Inferred)
        self.addParam('Destination', Element('skiros:Location'), ParamTypes.Required)

        self.addPreCondition(ConditionRelation('RobotAtStart', 'skiros:at', 'Robot', 'Start', True))
        self.addPreCondition(ConditionRelation('RobotNotAtDest', 'skiros:at', 'Robot', 'Destination', False))

        self.addPostCondition(ConditionRelation('RobotNotAtStart', 'skiros:at', 'Robot', 'Start', False))
        self.addPostCondition(ConditionRelation('RobotAtDest', 'skiros:at', 'Robot', 'Destination', True))

class NavigateToGoal(SkillBase):
    def createDescription(self):
        self.setDescription(NavigateToGoalDescription(), 'navigatetogoal')

    def set_relation(self, src, rel, dst, state):
        return self.skill('WmSetRelation', 'wm_set_relation',
            remap={'Dst': dst},
            specify={'Src': self.params[src].value, 'Relation': rel, 'RelationState': state})

    def expand(self, skill):
        skill(
            self.skill('NavigateToGoalPrimitiveDescription', 'NavigateToGoalPrimitive'),
            self.set_relation('Robot', 'skiros:at', 'Start', False),
            self.set_relation('Robot', 'skiros:at', 'Destination', True),
            self.skill("NavigateToGoalPrimitiveDescription", "NavigateToGoalPrimitive", specify={"Destination":"4.0,4.0"})

        )
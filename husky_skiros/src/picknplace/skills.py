#!/usr/bin/env python

from skiros2_skill.core.skill import SkillBase, Sequential

from skiros2_common.core.conditions import ConditionProperty, ConditionRelation
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element
from skiros2_skill.core.skill import SkillDescription

class PickObjectDescription(SkillDescription):
    def createDescription(self):
        self.addParam('Arm', Element('husky:Pick'), ParamTypes.Required)

class PlaceObjectApplyPaintDescription(SkillDescription):
    def createDescription(self):
        self.addParam('Arm', Element('husky:Place'), ParamTypes.Inferred)
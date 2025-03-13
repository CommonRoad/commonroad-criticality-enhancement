import pathlib

from commonroad.common.file_reader import CommonRoadFileReader

import crtemplate
from crtemplate.main import TemplateClass

scenario, planning_problem = CommonRoadFileReader(
    pathlib.Path(crtemplate.__file__).parent.joinpath("./../scenarios/ZAM_Tjunction-1_307_T-1.xml")
).open()

crtemplate_object = TemplateClass(scenario)

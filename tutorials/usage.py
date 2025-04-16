import os
import sys

# Add the parent directory (my_project) to the system path
sys.path.append(
    os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "enhance_criticality"))
)

import pathlib

from commonroad.common.file_reader import CommonRoadFileReader

# import crtemplate
# from crtemplate.main import TemplateClass
# from main import TemplateClass

scenario, planning_problem = CommonRoadFileReader(
    pathlib.Path(__file__).parent.joinpath("./../scenarios/ZAM_Tjunction-1_307_T-1.xml")
).open()

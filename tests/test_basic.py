import pathlib
import unittest

from commonroad.common.file_reader import CommonRoadFileReader

import crtemplate
from crtemplate.main import TemplateClass


class TemplateClassTest(unittest.TestCase):
    def setUp(self):
        scenario, _ = CommonRoadFileReader(
            pathlib.Path(crtemplate.__file__).parent.joinpath(
                "./../scenarios/ZAM_Tjunction-1_307_T-1.xml"
            )
        ).open()
        self.object = TemplateClass(scenario)

    def test_number_of_lanelets(self):
        self.assertEqual(self.object.return_number_of_lanelets(), 12)

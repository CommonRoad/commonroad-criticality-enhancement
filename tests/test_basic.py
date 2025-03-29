import os
import sys

sys.path.append(
    os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "enhance_criticality"))
)
import pathlib
import unittest

from commonroad.common.file_reader import CommonRoadFileReader
from main import TemplateClass

print("hello world")


class TemplateClassTest(unittest.TestCase):
    def setUp(self):
        scenario, _ = CommonRoadFileReader(
            pathlib.Path(__file__).parent.joinpath("./../scenarios/ZAM_Tjunction-1_307_T-1.xml")
        ).open()
        self.object = TemplateClass(scenario)

    def test_number_of_lanelets(self):
        self.assertEqual(self.object.return_number_of_lanelets(), 12)

import os

from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.scenario.scenario import Scenario


def save_modified_scenario(scenario: Scenario, planning_problem_set: PlanningProblemSet) -> str:
    """
    Saves a modified CommonRoad scenario and planning problem set to a fixed XML file path.

    Parameters:
    - scenario (Scenario): The modified CommonRoad scenario.
    - planning_problem_set (PlanningProblemSet): The associated planning problem set.

    Returns:
    - str: The relative file path to the saved scenario.
    """
    temp_file = os.path.join("scenarios", "modified_scenario.xml")
    writer = CommonRoadFileWriter(scenario, planning_problem_set)
    writer.write_to_file(temp_file, overwrite_existing_file=OverwriteExistingFile.ALWAYS)
    print("The new scenario was saved in modified_scenario.xml")
    return "scenarios/modified_scenario.xml"

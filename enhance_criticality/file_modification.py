import os

from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile


def save_modified_scenario(scenario, planning_problem_set) -> str:
    temp_file = os.path.join("scenarios", "modified_scenario.xml")
    writer = CommonRoadFileWriter(scenario, planning_problem_set)
    writer.write_to_file(temp_file, overwrite_existing_file=OverwriteExistingFile.ALWAYS)
    print("The new scenario was saved in modified_scenario.xml")
    return "modified_scenario"

from commonroad.scenario.scenario import Scenario


class TemplateClass:
    """
    This is a class in the CommonRoad Template repository.
    """

    def __init__(self, scenario: Scenario) -> None:
        """
        Initialize a TemplateClass object.

        :param scenario: CommonRoad Scenario.
        :returns: TemplateClass object.
        :raises ValueError: If input is not a CommonRoad scenario.
        """
        if not isinstance(scenario, Scenario):
            raise ValueError("Input is not a CommonRoad scenario")

        self.scenario = scenario

    def return_number_of_lanelets(self) -> int:
        """
        Compute number of lanelets of a scenario contained in TemplateClass object.

        :returns: Number_of_lanelets: Number of lanelets in the scenario.
        """
        number_of_lanelets = len(self.scenario.lanelet_network.lanelets)
        return number_of_lanelets

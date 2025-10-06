# CommonRoad Criticality Enhancement

CommonRoad Criticality Enhancement is a toolbox to increase the criticality of CommonRoad scenario by reducing the drivable area.

## Quick Start

This project uses poetry and supports Python 3.10 and 3.11.

### Installation

This project is currently only available on GitHub. To get started you first need to clone the repository:

```sh
$ git clone https://github.com/CommonRoad/commonroad-criticality-enhancement.git && cd commonroad-criticality-enhancement
```

Then you can install the project:
```sh
$ poetry install
```

### Usage

The `tutorials/` folder contains several scripts to get started with the functionality of this project. The scripts can be executed like so:

```
poetry run python tutorials/compare_all.py.py
```

## Supported Criticality Enhancement Methods

Currently, the program supports gradient-based optimization, simulated annealing, and Bayesian optimization to adjust ego vehicle's velocity and position.

## Documentation

The full documentation can be found at [cps.pages.gitlab.lrz.de/commonroad/criticality-enhancement](https://cps.pages.gitlab.lrz.de/commonroad/criticality-enhancement/).

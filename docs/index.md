# CommonRoad-Template
The following code has been taken from the [Criticality-Enhancement repository](https://gitlab.lrz.de/cps/commonroad/criticality-enhancement).

[![PyPI pyversions](https://img.shields.io/pypi/pyversions/commonroad-prediction.svg)](https://pypi.python.org/pypi/commonroad-prediction/)
![Linux](https://img.shields.io/badge/Linux-FCC624?style=for-the-badge&logo=linux&logoColor=black)
![Windows](https://img.shields.io/badge/Windows-0078D6?style=for-the-badge&logo=windows&logoColor=white)
![macOS](https://img.shields.io/badge/mac%20os-000000?style=for-the-badge&logo=macos&logoColor=F0F0F0)
[![PyPI version fury.io](https://badge.fury.io/py/commonroad-prediction.svg)](https://pypi.python.org/pypi/commonroad-prediction/)
[![PyPI download month](https://img.shields.io/pypi/dm/commonroad-prediction.svg?label=PyPI%20downloads)](https://pypi.python.org/pypi/commonroad-prediction/)
[![PyPI download week](https://img.shields.io/pypi/dw/commonroad-prediction.svg?label=PyPI%20downloads)](https://pypi.python.org/pypi/commonroad-prediction/)
[![PyPI license](https://img.shields.io/pypi/l/commonroad-prediction.svg)](https://pypi.python.org/pypi/commonroad-prediction/)

A collection and interface for the Criticality-Enhancement features.

## Project status
Currently implemented and tested models:

- BO – Bayesian Optimization routines for minimizing drivable area.
- SA – Simulated Annealing routines for minimizing drivable area.
- Optimization – Objective function construction and parameter integration. [1]
- File Modification – Utilities to manipulate and save CommonRoad scenario files.
- Profile Matrix Computation – Computes drivable area sensitivity across planning steps.[1]
- Reach Flow – Calculates and creates reachable areas using reachability graphs.[2]

We highly welcome your contribution.
If you want to contribute, please create an issue/pull request in our [GitHub repository](https://gitlab.lrz.de/cps/commonroad/criticality-enhancement).


## Installation and Usage
We recommend using PyCharm (Professional) as IDE.

### Development
This project currently uses Anaconda to manage environment.
Clone the repository and install it with conda.
```shell
git@gitlab.lrz.de:cps/commonroad/criticality-enhancement.git
conda env create -f environment.yaml
conda activate myenv
```
Further, the project uses the [CommonRoad-Reach-Flow repository](https://gitlab.lrz.de/cps/commonroad/commonroad-reach-flow.git).
You can find more information in how to install it in the ReadMe file of this repository.

## Documentation
You can generate the documentation within your activated Poetry environment.
The documentation will be located under site, where you can open `index.html` in your browser to view it.
```bash
poetry install
poetry add mkdocstrings[python]
poetry add --dev mkdocs-material
mkdocs serve
```

## Authors
Responsible: Hristina Ivanova


## References
The implemented algorithms are based on the subsequent publications:

[1] M. Althoff and S. Lutz,
“Automatic Generation of Safety-Critical Test Scenarios for Collision Avoidance of Road Vehicles,”
in 2018 IEEE Intelligent Vehicles Symposium (IV), Changshu, China, 2018.

[2] [CommonRoad-Reach-Flow repository](https://gitlab.lrz.de/cps/commonroad/commonroad-reach-flow.git)

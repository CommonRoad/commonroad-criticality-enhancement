# Criticality-enhancement


## Description
This is a thesis project that aims to make AV scenarios more critical by reducing drivable area.
It is based on the CommonRoad framework.

## System Requirements

The software is written in Python 3.10.

## Building the code

- The code can be built using Anaconda to manage a virtual python environment.
- Note that currently the poetry package manager is not used
- All dependencies can be installed with the following command:

```
conda env create -f environment.yml
```
- All required versions are listed in the environment.yml file.
- Activate the environment after creating it
- The project uses CommonRoad-reach-flow
- The dependencies for it are also included in the environment.yaml file
- CommonRoad-reach-flow has to be installed by cloning its repository in the python environment
- Switch to the development branch of cr-reach-flow
- Run the following command in the CommonRoad-reach-flow repository:

```
pip install -v .
```

## Pre-commit hooks

- To run the pre-commit hooks use the following command:
```
pre-commit run --all-files
```

## Running the tutorials
- Run scripts like this:
```
python tutorials/script.py
```
The tutorials can be run manually and can be used for testing and program evaluation.
***

## Common errors
- If errors occur with CMake or libraries not found:
  - Check if environment variables are set correctly for gcc and g++
  - Delete build folder in cr-reach-flow and rerun the installation
  -
## Project status
- Currently, the program supports gradient-based optimization, simulated annealing, and Bayesian optimization to adjust ego vehicle's velocity and position
- Further features can be added in the future

## Documentation
You can generate the documentation within your activated environment.
The documentation will be located under site, where you can open `index.html` in your browser to view it.
```bash
mkdocs serve
```
## Authors and acknowledgment
Project: Hristina Ivanova
Supervisor: Florian Finkeldei

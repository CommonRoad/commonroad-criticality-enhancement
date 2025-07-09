# Criticality-enhancement


## Description
This is a thesis project that aims to make AV scenarios more critical by reducing drivable area.
It is based on CommonRoad libraries.

## System Requirements

The software is written in Python 3.10.

## Building the code

- The code can be built using Anaconda to manage a virtual python environment.
- Note that currently the poetry package manager is not used
- All dependencies can be installed with the following command:

```
conda env create -f environment.yaml
conda activate myenv
```
- The project uses CommonRoad-reach-flow
- The dependencies for it are also included in the environment.yaml file
- CommonRoad-reach-flow has to be installed by cloning its repository in the python environment
- Switch to the development branch of cr-reach-flow
- Run the following command in the CommonRoad-reach-flow repository:

```
pip install -v .
```
- If errors occur with CMake or libraries not found:
  - Check if environment variables are set correctly for gcc and g++
  - Delete build folder in cr-reach-flow and rerun the installation
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
As poetry is currently not used (cr-reach-flow repository not recognized), gitlab-ci is not able to recognize imports and run tests. Therefore the tutorials can be run manually and used for testing.
***

## Support
Tell people where they can go to for help. It can be any combination of an issue tracker, a chat room, an email address, etc.

## Roadmap
If you have ideas for releases in the future, it is a good idea to list them in the README.

## Documentation
You can generate the documentation within your activated Poetry environment.
The documentation will be located under site, where you can open `index.html` in your browser to view it.
```bash
poetry install
poetry add mkdocstrings[python]
poetry add --dev mkdocs-material
mkdocs serve
```
## Authors and acknowledgment
Show your appreciation to those who have contributed to the project.

## License
For open source projects, say how it is licensed.In general, we use the BSD 3-Clause License. However, please check the licenses of the submodules and the dependencies of the submodules. The strictest license applies to your repository.

## Project status
If you have run out of energy or time for your project, put a note at the top of the README saying that development has slowed down or stopped completely. Someone may choose to fork your project or volunteer to step in as a maintainer or owner, allowing your project to keep going. You can also make an explicit request for maintainers.

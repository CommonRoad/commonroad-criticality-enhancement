# CommonRoad Repository Template

[![pipeline status](https://gitlab.lrz.de/cps/commonroad/commonroad-template/badges/dev/pipeline.svg)](https://gitlab.lrz.de/cps/commonroad/commonroad-template/-/commits/dev)
[![coverage report](https://gitlab.lrz.de/cps/commonroad/commonroad-template/badges/dev/coverage.svg)](https://gitlab.lrz.de/cps/commonroad/commonroad-template/-/commits/dev)

This repository should be used as a reference when creating new CommonRoad-related repositories or when updating
existing ones to match the CommonRoad coding conventions. Please also refer to the [corresponding wiki entry](https://collab.dvb.bayern/display/TUMcpsgroup/CommonRoad+Software+Development+Workflow).

## Pre-commit hooks
The pre-commit hook can be executed with
```bash
pre-commit run --all-files
```
or it's automatically run once you commit your changes.

## Use Current Development Versions of Other CommonRoad Repositories
To use the current development versions of other CommonRoad repositories, you can add the following to your [`pyproject.toml`](pyproject.toml):
```toml
commonroad-io = { git = "git@gitlab.lrz.de:cps/commonroad/commonroad-io.git", branch = "develop" }
```
Make sure to enable access to the target repository.
To this end, edit the target repository's Settings → CICD → Job token permissions.
Moreover, the [CICD config](.gitlab-ci.yml) of your repository must be configured to enable access to the target repository:
```yml
- git config --global url."https://gitlab-ci-token:${CI_JOB_TOKEN}@gitlab.lrz.de".insteadOf "ssh://git@gitlab.lrz.de"
```

## License
In general, we use the BSD 3-Clause License. However, please check the licenses of the submodules and the dependencies
of the submodules. The strictest license applies to your repository.

## Push to PyPI
Please check the correct license first!
Detailed instructions for publishing your package to [PyPI](pypi.org) are given [here](https://collab.dvb.bayern/display/TUMcpsgroup/Publishing+CommonRoad+tools+on+PyPI).

## Mirroring to GitHub
To establish a mirror to GitHub, follow [these instructions](https://collab.dvb.bayern/display/TUMcpsgroup/Gitlab+Repository+Mirrors).

## Running CICD Pipeline as Nightly
To test your repository regularly (e.g., detect changes in upstream repos that cause errors in your code),
you can set up a nightly pipeline.
To this end, add a schedule in your repository's Build → Pipeline schedules.
Do not use Sunday 0 AM as the time of execution (because the docker registry is cleaned up at this time).

## Further information
are given in the [CPS wiki > Software tools > CommonRoad](https://collab.dvb.bayern/display/TUMcpsgroup/CommonRoad) and its subpages.

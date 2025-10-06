# Development

For the development of `commonroad-criticality-enhancement` we recommend [python 3.10](https://www.python.org/downloads/) and [poetry](https://python-poetry.org/).

To get started with development, clone the repo and install the necessary dependencies:

```sh
$ git clone https://github.com/CommonRoad/commonroad-criticality-enhancement.git && cd commonroad-criticality-enhancement
$ poetry install --extras test --extras dev
```

## Tests

To run the tests you can use:

```sh
$ poetry run pytest
# Run the tests with coverage
$ poetry run coverage run -m pytest
# Report the coverage on the command line
$ poetry run coverage report
# Alternativly generate HTML coverage files
$ poetry run coverage html
```

## pre-commit

pre-commit is used to run a variety of tools and checks on the code (e.g. formatting with `ruff`).
You can install the hooks with:

```sh
$ pre-commit install
```

Those hooks will be run every time you run `git commit`.
Alternatively you can execute the pre-commit hooks manually:

```sh
$ pre-commit run --all-files
```


## Documentation

The documentation is built with [MkDocs](https://www.mkdocs.org/). Before building, make sure to install the required dependencies:

```sh
$ poetry install --extras docs
```

Then you can build the documentation with:

```sh
$ poetry run mkdocs build
```

While writing/developing the documentation you can run a hot-reloading webserver:

```sh
$ poetry run mkdocs serve
```

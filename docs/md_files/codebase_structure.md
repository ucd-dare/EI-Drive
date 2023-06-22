## OpenCDA Codebase Structure

The `OpenCDA` codebase directory is structured as follows:

```sh
OpenCDA
├── docs  # documents of EIdrive, no need to pay attention.
├── EIdrive
│   ├── assests  # customized map and sumo xml package.
│   ├── delete_co_simulation  # source codes for sumo background traffic generation.
│   ├── core  # the core part of the code
│   │   └── actutation # implementation of control algorithms
│   │   ├── delete_application # implementation of cooperative driving applications
│   │   ├── common # base class and communication class for cavs
│   │   ├── map # HDMap manager
│   │   ├── plan # planning algorithms
│   │   └── sensing # perception adn localization algorithms.
│   ├── customize # where the users put their customized algorithm to replace the default modules
│   ├── scenario_testing # where all scenario testing scripts and yaml files exist
│   │   └── config_yaml # the yaml file defining the testing test_scenario
│   │   ├── evluations # evluation scripts for different modules' performance
│   │   ├── utils # utility functions to construct test_scenario based on given yaml files.
├── scripts  # installation scripts
├── tests  # unit tests
```

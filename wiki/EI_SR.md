## EI-Drive & SenarioRunner Integration

### Context

The `opensenario_test` scripts introduced a vehicle following demo in the absence of a separately executed `scenario_runner`. Here are commands needed to run the demo.

```bash
# 1. Pull up the Carla server
$ ./CarlaUE4.sh
# 2. Run EI-Drive
$ python EI_Drive.py --t opensenario_test -v 0.9.14
# 3. Run scenario_runner
$ python scenario_runner.py --scenario FollowLeadingVehicle_1
```

As `EI_Drive` and `scenario_runner` are manually launched and having two separate user-interfaces is error-prone, we added `sr_ei` demo to the codebase executing two subprocesses with a single launch.

```bash
$ python EI_Drive.py --t ei_sr -v 0.9.14
```

### Configuration

- We need to first install [SenarioRunner documents](https://carla-scenariorunner.readthedocs.io/en/latest/).

- Create an empty `__init__.py` under the `SenarioRunner` installation folder, so that the folder is treated as a package.

- Insert `SenarioRunner` to the Python API paths so that it can be imported. `SenarioRunner` locates resource paths with `SENARIO_RUNNER_ROOT`. Therefore, we need to set up the two environment variables as follows.

```bash
# Execute in the shell or put in .bashrc
export PYTHONPATH="$HOME/scenario_runner:$PYTHONPATH"
export SCENARIO_RUNNER_ROOT="$HOME/scenario_runner
```

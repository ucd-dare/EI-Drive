#!/bin/bash

autopep8 scenario_runner.py --in-place --max-line-length=120
autopep8 scenario_runner/srunner/scenariomanager/*.py --in-place --max-line-length=120 --ignore=E731
autopep8 scenario_runner/srunner/scenariomanager/actorcontrols/*.py --in-place --max-line-length=120
autopep8 scenario_runner/srunner/scenariomanager/scenarioatomics/*.py --in-place --max-line-length=120
autopep8 scenario_runner/srunner/scenarios/*.py --in-place --max-line-length=120
autopep8 scenario_runner/srunner/autoagents/*.py --in-place --max-line-length=120
autopep8 scenario_runner/srunner/tools/*.py --in-place --max-line-length=120
autopep8 scenario_runner/srunner/scenarioconfigs/*.py --in-place --max-line-length=120


pylint --rcfile=.pylintrc --disable=I scenario_runner/srunner/scenariomanager/
pylint --rcfile=.pylintrc scenario_runner/srunner/scenarios/
pylint --rcfile=.pylintrc scenario_runner/srunner/autoagents/
pylint --rcfile=.pylintrc scenario_runner/srunner/tools/
pylint --rcfile=.pylintrc scenario_runner/srunner/scenarioconfigs/
pylint --rcfile=.pylintrc scenario_runner.py

# Robotpy

1. [Follow instructions on WPILib's Python Setup page](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/python-setup.html)
2. (Optional) [Install the WPILib IDE](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html). With robotpy, all you really need is the robotpy CLI tool and _any_ IDE.
3. Copy a valid robotpy project or run `robotpy init`. This will create/steal a `pyproject.toml` file with the required dependencies specified.
4. Run `robotpy sync`. This will install all dependencies in `pyproject.toml`.

From here, you can test your code in the simulator with `robotpy sim` or using unit tests with `robotpy test`. To run the code on your robot, simply run `robotpy deploy`.

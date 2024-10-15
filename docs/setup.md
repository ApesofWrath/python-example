# Python

1. [Follow instructions on WPILib's Python Setup page](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/python-setup.html)
2. (Optional) [Install the WPILib IDE](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html). With robotpy, all you really need is the robotpy CLI tool and _any_ IDE.
3. Copy a valid robotpy project or run `robotpy init`. This will create/steal a `pyproject.toml` file with the required dependencies specified.
4. Run `robotpy sync`. This will install all dependencies in `pyproject.toml`.

From here, you can test your code in the simulator with `robotpy sim` or using unit tests with `robotpy test`. To run the code on your robot, simply run `robotpy deploy`.

# Git

1. Go to the [Git download page](https://git-scm.com/downloads) and follow the instructions for your operating system (probably the one displayed in the monitor image on the right).
2. Create a [GitHub](https://github.com) account.
3. (Only for team 668) Request the programming captain to add your account to the `ApesOfWrath` organization. This should give you write permissions for the team codebases.
4. Set your git username and email on your local machine using the following commands:
```shell
git config --global user.name "Your Name"
git config --global user.email "your@email.example"
```
5. Use the team standard default branch name
```shell
git config --global init.defaultBranch main
```

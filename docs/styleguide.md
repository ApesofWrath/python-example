# General
- Adhere to ([pep 8](https://peps.python.org/pep-0008) style guide for Python
- Exceptions to above are listed in this document
- It's a good idea to change your editor settings to match these

# Linting
- Use `mypy robotcontainer.py --strict --ignore-missing-imports --allow-subclassing-any` to enforce good practice
- Adding exceptions via `# type: ignore` are only allowed on `float.m_as()` calls and state machine enums in attributes
- Additional exceptional circumstances may only be added with the permission of the current programming captain

# Indentation
- Use tabs. I will take no arguments.
- Change "Editor: Insert Spaces" and "Editor: Detect Indentation" to false

# Exploding
"Exploding" refers to the practice of taking a line which contains multiple values (list, tuple, function arguments) and putting those values on different lines.
- Exploding is to be used when the aforementioned multiple values make the code difficult to read on one line
- You may not explode some values but not others
- The character that begins the values (`[` or `(`) is to be put on the same line as preceding function definition/call or variable declaration
- The ending character (`]` or `)`) is to be put on its own line on the same indentation level (not column) as the line containing the beginning character
- Values are to be indented one additional level then the beginning and ending characters

# Constants
- Constants are stored in a root-level `constants.py` file
- `constants.py` is separated into multiple sections (at least one per mechanism) using classes
- Mechanisms are only allowed to import their class from `constants.py`

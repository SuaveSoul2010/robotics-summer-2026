# OOP

These are the Python object-oriented programming exercises I worked through before starting on ROS. They all build on the same small example, a robot with a battery, and each file adds one new idea.

| File | What it covers |
| --- | --- |
| `basics.py` | A single `Robot` class with a name and a battery level, and methods to report its status and charge it |
| `basics2.py` | Splitting the battery out into its own `Battery` class, so that a `Robot` owns a `Battery` and each move drains it |
| `properties.py` | Using `@property` and a setter so that a `Robot` can't be given an empty name. A similar property that would keep the battery level between 0 and 100 is written out too, but it's currently switched off |
| `inheritance.py` | `LightRobot` and `HeavyRobot` subclasses of `Robot`. A light robot drains 5 from its battery each move and a heavy one drains 20, and `HeavyRobot` checks its weight with a setter of its own |

`inheritance.py` imports its base classes from `properties.py`, so both files need to be in the same folder to run it:

```bash
cd OOP
python3 inheritance.py
```

The example at the bottom of `inheritance.py` creates a `HeavyRobot` that is lighter than the setter allows, so running it ends with the setter's `ValueError: Wrong weight class`.

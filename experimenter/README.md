# SkinSim Experimenter

# About
These tools enable automatic configuring and experimenting on the SkinSim platform.

# Run
(1) Generate experiment model files and control specifications:
```
./auto_generator [EXP NAME] [EXP TYPE]
```
This creates a folder ``skinsim/data/[EXP NAME]/`` (the default experiment name is "exp01"). The experiment types with their corresponding values are listed below.

| Value |  Type             | Description   |
|:-----:|-------------------|---------------|
| 0     | None              |               |
| 1     | TactileLayout     |               |
| 2     | PlungerOffset     |               |
| 3     | LayoutAndOffset   |               |
| 4     | ModelParam        |               |
| 5     | PIDtuning         |               |
| 6     | TimeStep          |               |

For example,
```
./auto_generator pid_03 5
```
saves the desired Gazebo model and world files inside ```skinsim/model/``` and 
creates a data folder ``skinsim/data/pid_03`` containing the model and control specification files ``mdlSpecs.yaml`` and ``ctrlSpecs.yaml``.

(2) Automatically run calibration procedure using the generated model and control specification files:
```
./auto_calibrator [EXP NAME]
```
Essentially, the controller applies a step-input force of 1N and Kc is calibrated at steady-state.
The calibration values are saved to ``tactile_calibration.yaml`` and the plunger positions to ``plunger_position.yaml``. In case of a crash, temporary results are stored in ``tmp_*.yaml`` files. Any CSV files containing plunger, joint, tactile, layout, and COP data are stored inside the ``/calibration`` folder.

(3) Automatically run the experiments using the generated specification files and calibration values:
```
./auto_experimenter [EXP NAME]
```
 All the experiment data is saved to the ``skinsim/data/[EXP NAME]/experiment`` folder.

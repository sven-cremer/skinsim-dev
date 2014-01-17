# Initial README

# Generate new models

1. Edit config/model_params.yaml to change model parameters

2. You have two options
- Generate the joint_names.yaml and the model.sdf inside generated_models folder by doing:
- roslaunch model_generator model_generator.launch
OR
- Update the joint_names.yaml and the model.sdf in spring_array package, they reside in config/joint_names.yaml and models/spring_board/model.sdf by doing:
- roslaunch model_generator spring_array_model_update.launch

3. Happily do following to see your new models:

roslaunch spring_array spring_array.launch

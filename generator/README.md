# Initial README

# Generate new models

1. Edit ```config/model_params.yaml``` to change model parameters

2. Generate the joint_names.yaml and the model.sdf inside model/models folder by doing:

		./skin_model_generator

3. Happily do following to see your new models in Gazebo:

		gazebo skinsim_model.world

geometry_msgs/Pose pose

# List of types of objects being searched in given nbv.
string[] object_type_name_list

# old index before sorting
uint32 oldIdx

# weighted rating
float32 rating
# utility and costs
float32 utility
float32 inverse_costs
# specific costs
float32 base_translation_inverse_costs
float32 base_rotation_inverse_costs
float32 ptu_movement_inverse_costs
float32 recognition_inverse_costs

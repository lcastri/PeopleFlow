# peopleflow_msgs/msg/pfAgent.msg

Header header
uint64 id
int32 starting_time
int32 exit_time
geometry_msgs/Point position
std_msgs/Bool is_stuck
std_msgs/Bool at_work
std_msgs/Bool is_quitting


std_msgs/String past_WP_id
std_msgs/String current_WP_id
std_msgs/String[] path
std_msgs/String next_WP_id
geometry_msgs/Point next_destination
float64 next_destination_radius
std_msgs/String final_WP_id

float64 task_duration
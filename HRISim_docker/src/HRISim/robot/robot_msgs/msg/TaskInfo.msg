# robot_msgs/msg/TaskInfo.msg

int32 task_id                # Unique integer ID for the task
string final_destination     # Final destination as a string
string[] path                # List of waypoints representing the path
float64 tot_inf_time         # Total time taken for inference in seconds
float64 mean_inf_time        # Mean time taken for inference in seconds
float64 planning_time        # Total time taken for planning in seconds
int32 evaluations            # Number of path evaluations performed
time start_time              # Task start time (ROS time format)
time end_time                # Task end time (ROS time format)
int8 result                  # Task result: 1 for success, -1 for fail
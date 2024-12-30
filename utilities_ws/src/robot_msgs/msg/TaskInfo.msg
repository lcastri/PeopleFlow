# robot_msgs/msg/TaskInfo.msg

int32 task_id                # Unique integer ID for the task
string final_destination     # Final destination as a string
string[] path                # List of waypoints representing the path
time start_time              # Task start time (ROS time format)
time end_time                # Task end time (ROS time format)
int8 result                  # Task result: 1 for success, -1 for fail
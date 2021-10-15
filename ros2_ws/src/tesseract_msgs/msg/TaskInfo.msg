# This message corresponds to tesseract_planning::TaskInfo

# Value returned from the Task on completion
int32 return_value

# Unique ID generated for the Task by Taskflow
uint64 unique_id

# Name of the Task
string task_name

# Debug message
string message

# XML of instructions passed to the task (optionally set)
string instructions_input

# XML of instructions after running the task (optionally set)
string instructions_output

# XML of seed/results passed to the task (optionally set)
string results_input

# XML of seed/results after running the task (optionally set)
string results_output

# Clone of the environment at the beginning of the task (optionally set)
tesseract_msgs/Environment environment

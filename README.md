# localization_evaluation
Evaluation node of dead-reckoning methods for AI Racing Tech in the Indy Autonomous Challenge.

## Overview

![Overview](https://i.ibb.co/WPWn6cz/Pasted-Graphic-1.png)

This package is used alongside a rosbag (or a source of sensors messages) and a sensor-fusion module. It will redirect the sensors messages to the sensor-fusion module, with drop out on GPS, and evaluate its performance.

This node subscribes to:
- `/gps_bot/pose`
- `/gps_bot/orientation`
- `/gps_bot/imu`
- `/filtered_publisher/pose`

It publishes to:
- `/intermediate/gps_pose`
- `/intermediate/gps_orientation`

It republises `/gps_bot/pose` and `/gps_bot/orientation` to `/intermediate/gps_pose` and `/intermediate/gps_orientation` outside the GPS denied zone. In the GPS denied zone, only IMU data is not blocke. The GPS zone is defined by the user using the command line arguments when running this package.


## How to use

This package is designed to evaluate the performance of dead-reckoning methods in conjunction with sensor fusion for AI Racing Tech in the Indy Autonomous Challenge. It redirects sensor messages to a sensor-fusion module while simulating GPS signal dropouts. This evaluation node subscribes to several topics and publishes data to aid in the evaluation process.

### Prerequisites

Before using this package, make sure you have the following prerequisites:

1. ROS 2: Ensure you have ROS 2 installed on your system.

2. Sensor-Fusion Module: You should have a sensor-fusion module that subscribes to sensor messages and performs sensor fusion.

### Running the Package

Follow these steps to run the `localization_evaluation` package:

1. Build your ROS 2 workspace if not already built:
   ```colcon build```
2. Source your ROS 2 installation:
 ```source install/local_setup.bash```
3. Run the evaluation node:
   ```ros2 run localization_evaluation intermediate_node```

### Command Line Arguments

You can configure the behavior of the evaluation node using command line arguments. Here are the available arguments:

- `--time_interval_gps_denied` (`-t`): Specify a list of time intervals (relative to the first GPS message received) in seconds when GPS is denied. For example, `-t 30 60 90 120` will create two GPS denied zones: 30s to 60s and 90s to 120s. By default, this option is None.

- `--areas_gps_denied` (`-a`): Specify a list of areas with center coordinates, width, and height (in meters) where GPS is denied. For example, `-a 0 0 10 10` will create a GPS denied zone of 10x10m centered on (0,0). By default, this option is None.

- `--random_gps_denied`: Use this flag to randomly set GPS denied zones. The probability (`prob`) of a new GPS denied zone appearing is calculated using an exponential growth model, where `prob` increases with the time since the last GPS denied zone. By default, this option is disabled.

- `--show_plot`: Use this flag to display a plot of the evaluation results after execution. By default, the plot is shown.

- `--save_plot`: Specify the file name to save the plot. Use 'None' to prevent saving the plot. By default, the plot is saved to `results_localization_test.png`.

- `--manual_prediction`: Use this flag to compute and plot manual predictions for comparison. By default, this option is enabled.

- `--verbose`: Use this flag to print informative messages during process. By default, informative messages are printed.

**Note:** Only one of `time_interval_gps_denied`, `areas_gps_denied`, and `random_gps_denied` can be used at the same time.

### Results and Plotting

The evaluation node collects data and computes errors between fused and GPS data, both for the entire trajectory and within GPS denied zones. You can view the results and error analysis for the entire trajectory as well as GPS denied zones.

If the `--show_plot` flag is set or a file name is specified using `--save_plot`, the evaluation node will plot and display the data after execution.

The plotted data includes:

- Full trajectory comparison between GPS, fused, and manual predictions (if enabled).
- Zoomed-in views of GPS denied zones with comparisons between GPS, fused, and manual predictions (if enabled).

The evaluation node provides statistics such as mean error, maximum error, minimum error, and standard deviation for error analysis.

### Example Usage

Here's an example command to run the evaluation node with specific settings:

```bash
ros2 run localization_evaluation localization_evaluation_node --random_gps_denied true --save_plot plot_run_3
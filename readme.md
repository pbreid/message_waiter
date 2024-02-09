# Message Waiter ROS Package

## Introduction
The Message Waiter is a ROS (Robot Operating System) package designed to wait for messages on specified topics before executing a given command. This functionality is particularly useful in scenarios where the execution of a command or script depends on the receipt of data from one or more ROS topics, ensuring that all necessary inputs are available before proceeding.

## Functionality
The package subscribes to a list of user-defined ROS topics and waits until a message is received on each. Once messages have been received on all specified topics, the package executes a predefined command. This process is especially useful in automated workflows and systems where subsequent actions rely on the availability of specific data.

- **Subscribing to Multiple Topics:** The node can listen to multiple topics simultaneously, waiting for at least one message on each.
- **Command Execution:** Executes a user-defined command or script upon receiving messages on all topics.
- **Flexible Usage:** Can be used in a variety of scenarios, including but not limited to, initialization checks, data dependency resolution, and workflow control.

## Usage

### Prerequisites
Ensure you have ROS installed on your system, compatible with the C++11 standard or newer. This package has been tested with ROS Kinetic and newer versions.

### Installation
1. Clone the package into your catkin workspace's `src` directory.
2. Navigate back to the root of your catkin workspace.
3. Run `catkin_make` to build the package.
4. Source the workspace's setup file with `source devel/setup.bash`.

### Running the Node
To run the Message Waiter node, use the included launch file with the necessary parameters for topics and the command to run.

#### Launch File Parameters
- `topic_name`: The name of the topic(s) to wait for messages from. For multiple topics, separate them with commas.
- `command_to_run`: The command to execute once messages have been received on all specified topics.

#### Example Launch Command
```xml
<launch>
    <node name="message_waiter_node" pkg="message_waiter" type="message_waiter_node" output="screen">
        <param name="topic_name" value="your_topic_1,your_topic_2" />
        <param name="command_to_run" value="your_command" />
    </node>
</launch>
```
Replace your_topic_1,your_topic_2 with the topics you're interested in, and your_command with the command you wish to execute once messages are received.

## Contributions
Contributions to the Message Waiter package are welcome. Please submit pull requests or issues through the package's GitHub repository.

## License
The Message Waiter package is provided under a standard ROS-compatible open source license. Please see the LICENSE file for full details.

## Support
For support with the Message Waiter package, please open an issue on the GitHub repository, and we will address it as soon as possible.

This `README.md` provides a comprehensive guide to the Message Waiter package, from its purpose and functionality to detailed installation and usage instructions. Be sure to replace placeholders (like `your_topic_1,your_topic_2` and `your_command`) with actual values relevant to your use case. Adjust any part of the document as necessary to fit the specifics of your project or environment.
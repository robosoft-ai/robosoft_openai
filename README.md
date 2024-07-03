# ros2_openai_server

OpenAI server node for ROS2 Applications

## What is this useful for?

If you don't have an image processing pipeline or a GPU on your robot, calls to OpenAI can replace much of that functionality. As of July 2024 we generally recommend yes/no questions, otherwise you will need to parse the response yourself. This package includes a simple filter which parses for yes/no. For example:

- Is it safe for the robot to drive forward? Please respond in one word, yes or no.
- Is the door open? Please respond in one word, yes or no.
- Is there a customer at that table? Please respond in one word, yes or no.
- Did the robot successfully grasp the cube?  Please respond in one word, yes or no.
- Does the control panel look identical to the last time I checked? Please respond in one word, yes or no.

Some of these examples are useful for reinforcement learning, i.e. for determining if a reward was earned.

## Setup

Requires an OpenAI key as described here: https://help.openai.com/en/articles/4936850-where-do-i-find-my-openai-api-key

Set it as an environment variable: `export OPENAI_API_KEY="..."`

## Run

Here's a quick example:

`ros2 run ros2_openai_server openai_server`

Send it a prompt. Here's an example that returns a bool from a yes/no question. Not that the `image` field of the service request may be left empty.

`ros2 service call /openai_bool_response ai_msgs/srv/BoolResponse prompt:\ "Are you a pirate? Please respond with a one-word answer, yes or no"`

Here's an example that returns a full string.

`ros2 service call /openai_string_response ai_msgs/srv/StringResponse prompt:\ "Are you a pirate?"`

There's an example client which sends an image of a wooden table and prompts whether it is indeed a wooden table:

`ros2 run ros2_openai_server example_client`

## Citation

If you use this work, please cite it like so:

 - Zelenak, A., Lock, J., & Aldrich, B. (2024) *An OpenAI Server for ROS2*. Github. **https://github.com/robosoft-ai/ros2_openai_server**

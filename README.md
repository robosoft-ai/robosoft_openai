# ros2_openai_server

OpenAI server node for ROS2 Applications

## Setup

Requires an OpenAI key as described here: https://help.openai.com/en/articles/4936850-where-do-i-find-my-openai-api-key

Set it as an environment variable: `export OPENAI_API_KEY="..."`

## Run

Here's a quick example:

`ros2 run ros2_openai_server openai_server`

Send it a prompt. Here's an example that returns a bool from a yes/no question. Not that the `image` field of the service request may be left empty.

`ros2 service call /openai_bool_response ai_msgs/srv/BoolResponse prompt:\ "are you a pirate? please respond with a one-word answer, yes or no"`

Here's an example that returns a full string.

`ros2 service call /openai_string_response ai_msgs/srv/StringResponse prompt:\ "are you a pirate?"`

There's an example client which sends an image of a wooden table and prompts whether it is indeed a wooden table:

`ros2 run ros2_openai_server example_client`

## Citation

If you use this work, please cite it like so:

 - Zelenak, A., Lock, J., & Aldrich, B. (2024) *An OpenAI Server for ROS2*. Github. **https://github.com/robosoft-ai/ros2_openai_server**

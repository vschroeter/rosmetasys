# ROS 2 Meta System Exporter

This commandline tool can be used to export the meta information of a ROS2 system.
This includes:
- the node names (and their namespaces)
- publishers and subscribers of each node
- services and clients of each node
- message types of each topic

# Basic usage

## Export your dataset

Exports the system meta information, named [SYSTEM_NAME]:

```bash
rosmetasys export [OPTIONS] [SYSTEM_NAME]
```

Options:
  - `-v` | `--verbose`      Print more output.
  - `-i` | `--interactive`  Interactive version asking you the options.
  - `-a` | `--anonymize`    Anonymizing of node names and topics.
  - `-z` | `--zip`          Create the zip file.
  - `-p` | `--pretty`       Pretty formatting for the json.

## Upload your dataset

The datasets are gathered in the [rosmetasys-datasets](https://github.com/vschroeter/rosmetasys-datasets) repository. Feel free to [create a new issue](https://github.com/vschroeter/rosmetasys-datasets/issues/new?assignees=vschroeter&labels=dataset&projects=&template=providing-a-new-dataset-.md&title=%5BDATASET%5D+New+dataset) with your dataset attached or create a pull request.

# Example output

```json
{
    "version": "1.0.0",
    "nodes": [
        {
            "name": "first_node",
            "namespace": "/",
            "localhost_only": true,
            "publishers": [
                {
                    "name": "/parameter_events",
                    "type": "rcl_interfaces/msg/ParameterEvent"
                },
                {
                    "name": "/my/topic/1",
                    "type": "std_msgs/msg/Empty"
                },
                {
                    "name": "/rosout",
                    "type": "rcl_interfaces/msg/Log"
                }
            ],
            "subscribers": [
                {
                    "name": "/my/topic/1",
                    "type": "std_msgs/msg/Empty"
                },
                {
                    "name": "/my/topic/2",
                    "type": "custom_type"
                }
            ],
            "services": [
                ...
            ],
            "clients": [
                ...
            ]
        },
        {
            "name": "second_node",
            "namespace": "/",
            "localhost_only": false,
            ...
        }
    ]
}

```

# Known issues

ROS nodes need a little time to start up. Sometimes, the introspection methods does not return all nodes of the system.
At the moment, the exporter waits for 5 seconds before starting the introspection.
But it's not guaranteed that this is enough time for all nodes to start up.
# ROS 2 Meta System Exporter

This commandline tool can be used to export the meta information of a ROS2 system.
This includes:
- the node names (and their namespaces)
- publishers and subscribers of each node
- services and clients of each node
- message types of each topic

# Installation

```bash
pip install rosmetasys
```

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

The datasets are gathered in the [rosmetasys-datasets](https://github.com/vschroeter/rosmetasys-datasets) repository. 
Feel free to [create a new issue](https://github.com/vschroeter/rosmetasys-datasets/issues/new?assignees=vschroeter&labels=dataset&projects=&template=providing-a-new-dataset-.md&title=%5BDATASET%5D+New+dataset) with your dataset attached or create a pull request.

If you want to anonymously submit your dataset without creating a pull request and have it uploaded to the repository, you can zip your dataset and send it to me via email (valentin.schroeter@hpi.uni-potsdam.de). 

## Visualize your Dataset

You can test the visualization of your data on https://vschroeter.github.io/RosComGraph/#/.
Open the website and upload the data set at the `Upload JSON file` input field in the left drawer.

<img width="881" alt="image" src="https://github.com/user-attachments/assets/e6a3b8ba-3035-49d2-b3c9-00651f907a43">

# Example output

```json
{
    "version": "1.0.0",
    "created_at": "2024-01-01T10:40:40.000000",
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

If you use the `--anonymize` option, the node names and topic names are irreversibly anonymized by replacing the node and topic names with consecutive IDs.

# Known issues

ROS nodes need a little time to start up. Sometimes, the introspection methods does not return all nodes of the system.
At the moment, the exporter waits for 5 seconds before starting the introspection.
But it's not guaranteed that this is enough time for all nodes to start up.
If your output seems incomplete, just try to export another time.

# ROS 2 Meta System Exporter

This commandline tool can be used to export the meta information of a ROS2 system.
This includes:
- the node names (and their namespaces)
- publishers and subscribers of each node
- services and clients of each node
- message types of each topic

# Basic usage

```bash
rosmetasys export output_filename [--format=json|yaml] [--localhost-only]
```

- `output_filename`: the name of the file to which the meta information will be written
- `--format`: the format of the output file. Default is `json`
- `--localhost-only`: Set this flag to discover nodes started with environment variable `ROS_LOCALHOST_ONLY=1` 







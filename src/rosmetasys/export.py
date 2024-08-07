from datetime import datetime
from io import BytesIO
import os
import logging
import zipfile

import rclpy
from rclpy.node import Node as RclpyNode

from rich import print
from rich.panel import Panel

from rosmetasys.console import console

import json

logger = logging.getLogger(__name__)

VERSION = "1.0.0"
TEMP_NODE_NAME = "_rosmetasys_temp_node" + datetime.now().strftime("%Y%m%d%H%M%S")


class Topic:
    def __init__(self, name: str, data_type: str):
        self.name = name
        self.data_type = data_type

    def __str__(self):
        return f"{self.name} [{self.data_type}\]"

    def __repr__(self):
        return self.__str__()


class Node:
    def __init__(self, name: str, namespace: str = "/"):
        self.name = name
        self.namespace = namespace
        self.localhost_only = os.environ["ROS_LOCALHOST_ONLY"] == "1"

        self.publishers: list[Topic] = []
        self.subscribers: list[Topic] = []
        self.services: list[Topic] = []
        self.clients: list[Topic] = []

        # TODO?
        # self.actions: list[Topic] = []

    @property
    def name_with_namespace(self):
        return self.namespace + "/" + self.name if self.namespace != "/" else self.name

    def __str__(self):
        s = f"{self.name} ({self.namespace}):\n"

        for name, list in [
            ("Publishers", self.publishers),
            ("Subscribers", self.subscribers),
            ("Services", self.services),
            ("Clients", self.clients),
        ]:
            if len(list) == 0:
                continue
            s += f"\t{name}:\n"
            for item in list:
                s += f"\t\t{item}\n"

        return s

    def __repr__(self):
        return self.__str__()

    def discover(self, node: RclpyNode):
        self.publishers = [
            Topic(item[0], item[1][0])
            for item in node.get_publisher_names_and_types_by_node(
                self.name, self.namespace
            )
        ]
        self.subscribers = [
            Topic(item[0], item[1][0])
            for item in node.get_subscriber_names_and_types_by_node(
                self.name, self.namespace
            )
        ]
        self.services = [
            Topic(item[0], item[1][0])
            for item in node.get_service_names_and_types_by_node(
                self.name, self.namespace
            )
        ]
        self.clients = [
            Topic(item[0], item[1][0])
            for item in node.get_client_names_and_types_by_node(
                self.name, self.namespace
            )
        ]

    @property
    def all_topics(self) -> list[Topic]:
        return self.publishers + self.subscribers + self.services + self.clients


class RosSystem:
    def __init__(self, nodes: list[Node] = None, kwargs: dict[str, str] = {}):
        self.nodes: list[Node] = nodes or []
        self.name = kwargs.get("name", "rosmetasys")
        self.description = kwargs.get("description", "")
        self.author = kwargs.get("author", "")
        self.cc_by_sa_consent = not kwargs.get("no_consent", False)
        self.created_at = datetime.now()

    @property
    def sorted_nodes(self):
        return sorted(self.nodes, key=lambda node: node.name)

    def discover(self):
        temp_node = rclpy.create_node(TEMP_NODE_NAME)

        logger.info("Waiting for other nodes to be registered...")
        # Wait until the node is registered
        rclpy.spin_once(temp_node, timeout_sec=1.0)
        nodes_and_ns = temp_node.get_node_names_and_namespaces()
        nodes_and_ns = list(set(nodes_and_ns))

        for node_name, node_ns in nodes_and_ns:
            if node_name == TEMP_NODE_NAME:
                continue
                # pass

            node = Node(node_name, node_ns)
            node.discover(temp_node)
            self.nodes.append(node)

        temp_node.destroy_node()


class RosSystemEncoder(json.JSONEncoder):
    def default(self, o: any):
        if isinstance(o, RosSystem):
            return {
                "version": VERSION,
                "name": o.name,
                "description": o.description,
                "created_at": o.created_at.isoformat(),
                "node_count": len(o.nodes),
                "author": o.author,
                "cc_by_sa_consent": o.cc_by_sa_consent,
                "nodes": o.nodes,
            }

        if isinstance(o, Node):
            return {
                "name": o.name,
                "namespace": o.namespace,
                "localhost_only": o.localhost_only,
                "publishers": o.publishers,
                "subscribers": o.subscribers,
                "services": o.services,
                "clients": o.clients,
            }

        if isinstance(o, Topic):
            return {"name": o.name, "type": o.data_type}

        return super().default(o)


class AnonymousRosSystemEncoder(RosSystemEncoder):
    name_anonymizer: dict[str, str] = {}
    namespace_anonymizer: dict[str, str] = {}
    topic_anonymizer: dict[str, str] = {}
    type_anonymizer: dict[str, str] = {}

    def default(self, o: any):
        if isinstance(o, Node):
            return {
                "name": self.anonymize(o.name, self.name_anonymizer, "node_"),
                "namespace": self.part_anonymize(
                    o.namespace, self.namespace_anonymizer, "ns_"
                ),
                "localhost_only": o.localhost_only,
                "publishers": o.publishers,
                "subscribers": o.subscribers,
                "services": o.services,
                "clients": o.clients,
            }

        if isinstance(o, Topic):
            return {
                "name": self.anonymize(o.name, self.topic_anonymizer, "topic_"),
                "type": self.anonymize(o.data_type, self.type_anonymizer, "type_"),
            }

        return super().default(o)

    @staticmethod
    def anonymize(s: str, anonymizer: dict[str, str], prefix: str):
        if s not in anonymizer:
            anonymizer[s] = f"{prefix}{len(anonymizer)}"
        return anonymizer[s]

    @staticmethod
    def part_anonymize(
        s: str, anonymizer: dict[str, str], prefix: str, delimiter: str = "/"
    ):
        if s == delimiter:
            return s

        parts = [p for p in s.split(delimiter) if len(p) > 0]
        anonymized_parts = [
            AnonymousRosSystemEncoder.anonymize(part, anonymizer, prefix)
            for part in parts
        ]
        return delimiter.join(anonymized_parts)


def export(system_name: str, **kwargs: dict[str, str]):
    console.rule("ROS2 Meta System Exporter")
    logger.debug("Starting ROS2 Meta System Exporter.")

    total_system = RosSystem(kwargs = kwargs)
    

    for localhost_value in ["1", "0"]:
        os.environ["ROS_LOCALHOST_ONLY"] = localhost_value

        logger.debug("Initialized rclpy.")
        rclpy.init()

        logger.debug("Starting discovery...")
        system = RosSystem()
        system.discover()


        # Add nodes, if they are not already in the total_system
        existing_nodes = set([node.name_with_namespace for node in total_system.nodes])
        new_nodes: list[Node] = []
        for node in system.sorted_nodes:
            if node.name_with_namespace not in existing_nodes:
                new_nodes.append(node)
                total_system.nodes.append(node)

        # msg = f"Found {len(system.nodes)} nodes in the system (localhost_only={localhost_value == '1'}):\n"
        msg = f"Found {len(new_nodes)} new nodes in the system (localhost_only={localhost_value == '1'}):\n"
        for node in new_nodes:
            msg += f"\t{node.namespace + '/' if node.namespace != '/' else ''}{node.name} ({len(node.all_topics)} Topics)\n"
        logger.info(msg)

        rclpy.shutdown()

    encoder_cls = AnonymousRosSystemEncoder if kwargs["anonymize"] else RosSystemEncoder

    datestring = total_system.created_at.strftime("%Y-%m-%d_%H-%M-%S")
    count_nodes = len(total_system.nodes)

    # Format count_nodes to  have 4 digits
    count_nodes = f"{count_nodes:04d}"

    export_name = count_nodes + "nodes_" + datestring + f"_{system_name}"

    total_system.name = export_name
    indent = 4 if kwargs["pretty"] else None
    json_string = json.dumps(total_system, cls=encoder_cls, indent=indent)

    output_file = export_name + ".json"
    with open(output_file, "w") as f:
        f.write(json_string)

    logger.info(f"Exported system to '{output_file}'")

    if kwargs["zip"]:
        # Create zip compressed file
        zip_buffer = BytesIO()

        with zipfile.ZipFile(zip_buffer, "a", zipfile.ZIP_DEFLATED, False) as zip_file:
            zip_file.writestr(output_file, json_string)

        with open(export_name + ".zip", "wb") as f:
            f.write(zip_buffer.getvalue())

        logger.info(f"Exported system to '{export_name}.zip'")

    console.rule("Finished ROS2 Meta System Exporter.")

    link = "https://github.com/vschroeter/rosmetasys-datasets/issues/new?assignees=vschroeter&labels=dataset&projects=&template=providing-a-new-dataset-.md&title=%5BDATASET%5D+New+dataset"
    print(
        Panel(
            f"[green]Upload you dataset here:[/green]\n[cyan]{link}",
            title="Upload Dataset",
            subtitle="Thank you!! :)",
        )
    )


if __name__ == "__main__":
    export("rosmetasys.json")

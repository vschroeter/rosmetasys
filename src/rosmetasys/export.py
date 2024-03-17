
import os
import logging

import rclpy
from rclpy.node import Node as RclpyNode

from rich.console import Console
from rich.logging import RichHandler

from rosmetasys.console import console



logger = logging.getLogger(__name__)


TEMP_NODE_NAME = "_rosmetasys_temp_node"

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
        # TODO:
        self.localhost_only = False

        self.publishers: list[Topic] = []
        self.subscribers: list[Topic] = []
        self.services: list[Topic] = []
        self.clients: list[Topic] = []

        # TODO?
        # self.actions: list[Topic] = []

    def __str__(self):
        s = f"{self.name} ({self.namespace}):\n"

        for name, list in [
            ("Publishers", self.publishers),
            ("Subscribers", self.subscribers),
            ("Services", self.services),
            ("Clients", self.clients)
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
            Topic(item[0], item[1][0]) for item in node.get_publisher_names_and_types_by_node(self.name, self.namespace)
        ]
        self.subscribers = [
            Topic(item[0], item[1][0]) for item in node.get_subscriber_names_and_types_by_node(self.name, self.namespace)
        ]
        self.services = [
            Topic(item[0], item[1][0]) for item in node.get_service_names_and_types_by_node(self.name, self.namespace)
        ]
        self.clients = [
            Topic(item[0], item[1][0]) for item in node.get_client_names_and_types_by_node(self.name, self.namespace)
        ]

class RosSystem:
    def __init__(self, nodes: list[Node] = None):
        self.nodes: list[Node] = nodes or []

    def discover(self):

        temp_node = rclpy.create_node(TEMP_NODE_NAME)
        
        # Wait until the node is registered
        rclpy.spin_once(temp_node, timeout_sec=1.0)

        nodes_and_ns = temp_node.get_node_names_and_namespaces()
        print(nodes_and_ns)

        for node_name, node_ns in nodes_and_ns:
            if node_name == TEMP_NODE_NAME:
                continue

            node = Node(node_name, node_ns)
            node.discover(temp_node)
            self.nodes.append(node)

        temp_node.destroy_node()



# if __name__ == "__main__":
def export(output_file: str, **kwargs: dict[str, str]):

    console.rule("ROS2 Meta System Exporter...")
    logger.info("Starting ROS2 Meta System Exporter...")

    os.environ['ROS_LOCALHOST_ONLY'] = '1'

    rclpy.init()
    system = RosSystem()
    system.discover()

    for node in system.nodes:
        print(node)

    rclpy.shutdown()
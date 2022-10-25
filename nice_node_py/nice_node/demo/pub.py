from dataclasses import dataclass

from nice_node import NiceNode, run
from std_msgs.msg import String


@dataclass
class PubCfg:
    hello_topic: str = "/topic"
    pub_rate: int = 10
    pub_size: int = 10


node: NiceNode[PubCfg] = NiceNode("NiceNodePub")
node.declare_config(PubCfg)

msg = ""


@node.init()
def run_once(_):
    """Callback only runs once"""
    pass


@node.init("pub_size")
def init_msg(_):
    """Callback runs after config option specified change"""
    global msg
    msg = "M" * node.cfg.pub_size


@node.interval("pub_rate")
def publish():
    rosmsg = String(data=msg)
    node.pub("hello_topic", rosmsg)


# @node.clean("hello_topic", "pub_size", "pub_rate")
@node.clean(...)
def clean(_):
    """callback runs before any config option changes"""
    pass


def main():
    run(node)


if __name__ == "__main__":
    main()

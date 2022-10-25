import time
from collections import deque
from dataclasses import dataclass

from nice_node import NiceNode, run


@dataclass
class SubCfg:
    hello_topic: str = "/topic"
    log_rate: int = 0.5
    buf_size: int = 1000
    buf_duration: int = 2


class State:
    buffer: deque = deque()
    prev_time: int = -1


G = State()

node: NiceNode[SubCfg] = NiceNode("NiceNodeSub")
node.declare_config(SubCfg)

log = node.get_logger()


@node.init("buf_size")
def init_buffer():
    G.buffer = deque([], node.cfg.buf_size)


@node.sub("hello_topic")
def on_msg(msg):
    now = time.time()
    if G.prev_time != -1:
        G.buffer.append(now - G.prev_time)
    G.prev_time = now
    while sum(G.buffer) >= node.cfg.buf_duration and len(G.buffer) > 1:
        G.buffer.popleft()


@node.interval("log_rate")
def log_rate():
    log.info(f"{len(G.buffer)/sum(G.buffer) if G.buffer else 0:.1f}Hz")


def main():
    run(node)


if __name__ == "__main__":
    main()

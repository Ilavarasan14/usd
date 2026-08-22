"""Builds rover_01's ROS2 LaserScan-publishing action graph.

Runs INSIDE Isaac Sim (Kit) -- unlike the rest of tools/*.py, which author
USD standalone against usd-core and never touch Kit at all. Companion to
tools/author_rover.py / README.md, which cover getting the rover driving;
this is the perception half of the walkthrough (00:11:11-00:13:36): read the
rover's lidar beam buffer every tick, timestamp it, publish it as a ROS 2
LaserScan so RViz can show live returns as the rover patrols.

Why a Python script and not more USD
--------------------------------------
simulation/sensors.usda now authors the sensor prims this graph needs
(Sensors/lidar_mount/physics_lidar, Sensors/camera_mount/rgb) as plain typed
attributes -- low risk, same pattern as the existing RTX lidar prim there.
The graph ITSELF is not authored as raw USD, on purpose: an OmniGraph's text
serialization (node prim types, exec-pin customData, connection syntax) is a
third, independent layer of guesswork on top of node names and namespace
tokens, and a mistake in it fails SILENTLY -- the graph just sits there
inert, which is exactly the bug this whole thread started from. Building it
through omni.graph.core.Controller.edit() instead means a wrong node type or
pin name raises a real Python traceback naming the bad node/pin, which is
something you can actually act on.

Run inside Isaac Sim
---------------------
Open root.usda, ensure the ROS 2 Bridge extension is enabled (Window >
Extensions, search "ros2") and ROS 2 is sourced on your system -- both
required by the walkthrough too. Then in Window > Script Editor:

    import sys; sys.path.append("<repo>/tools")
    import ros_bridge_setup
    ros_bridge_setup.build()

Verify (matches the walkthrough's own test): press Play, then in a terminal,
`ros2 topic list` should show /rover_01/scan. Launch rviz2, set Fixed Frame
to rover_01_lidar_frame, add a LaserScan display on that topic.

API-drift note
---------------
The four node type tokens below (isaacsim.core.nodes.*,
isaacsim.sensors.physics.IsaacReadLidarBeams,
isaacsim.ros2.bridge.ROS2PublishLaserScan) are this repo's best-documented
guess at the Isaac Sim 6.0.1 names for the exact nodes the walkthrough
narrates ("Isaac read lidar beams", "ROS2 publish laser scan", "Isaac read
simulation time") -- not verified against a live 6.0.1 Kit session. Likewise
the pin names in _CONNECT below are the standard/textbook wiring for this
node pair, not confirmed against this specific build. If Controller.edit()
raises on a node type, the fix is correcting that one token; if it raises on
a connection, open the two nodes in the Action Graph editor and check their
actual input/output names -- the walkthrough builds this same graph by hand
in under two minutes if scripting it turns out not to be worth chasing.
"""

ROVER_PATH = "/World/Scenario/Fleet/rover_01"
LIDAR_PATH = f"{ROVER_PATH}/Sensors/lidar_mount/physics_lidar"
GRAPH_PATH = f"{ROVER_PATH}/ActionGraph"
TOPIC_NAME = "/rover_01/scan"
FRAME_ID = "rover_01_lidar_frame"

_NODES = [
    ("OnTick", "omni.graph.action.OnPlaybackTick"),
    ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
    ("ReadLidar", "isaacsim.sensors.physics.IsaacReadLidarBeams"),
    ("PublishScan", "isaacsim.ros2.bridge.ROS2PublishLaserScan"),
]

_CONNECT = [
    ("OnTick.outputs:tick", "ReadLidar.inputs:execIn"),
    ("OnTick.outputs:tick", "ReadSimTime.inputs:execIn"),
    ("ReadLidar.outputs:execOut", "PublishScan.inputs:execIn"),
    ("ReadSimTime.outputs:simulationTime", "PublishScan.inputs:timeStamp"),
    ("ReadLidar.outputs:azimuthRange", "PublishScan.inputs:azimuthRange"),
    ("ReadLidar.outputs:depthRange", "PublishScan.inputs:depthRange"),
    ("ReadLidar.outputs:horizontalFov", "PublishScan.inputs:horizontalFov"),
    ("ReadLidar.outputs:horizontalResolution", "PublishScan.inputs:horizontalResolution"),
    ("ReadLidar.outputs:numCols", "PublishScan.inputs:numCols"),
    ("ReadLidar.outputs:numRows", "PublishScan.inputs:numRows"),
    ("ReadLidar.outputs:rotationRate", "PublishScan.inputs:rotationRate"),
    ("ReadLidar.outputs:linearDepthData", "PublishScan.inputs:linearDepthData"),
    ("ReadLidar.outputs:intensitiesData", "PublishScan.inputs:intensitiesData"),
]

_SET_VALUES = [
    ("ReadLidar.inputs:lidarPrim", [LIDAR_PATH]),
    ("PublishScan.inputs:topicName", TOPIC_NAME),
    ("PublishScan.inputs:frameId", FRAME_ID),
    ("PublishScan.inputs:queueSize", 10),
]


def build():
    import omni.graph.core as og

    keys = og.Controller.Keys
    graph, nodes, _, _ = og.Controller.edit(
        {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: _NODES,
            keys.CONNECT: _CONNECT,
            keys.SET_VALUES: _SET_VALUES,
        },
    )
    print(f"ros_bridge_setup: built {GRAPH_PATH} -- publishing {TOPIC_NAME} "
          f"(frame {FRAME_ID}) from {LIDAR_PATH}")
    return graph, nodes


if __name__ == "__main__":
    print(__doc__)

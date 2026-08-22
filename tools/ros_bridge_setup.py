"""Publishes rover_01's RTX lidar to ROS 2 as a LaserScan.

Runs INSIDE Isaac Sim (Kit) -- unlike the rest of tools/*.py, which author
USD standalone against usd-core and never touch Kit at all. This is the
perception half of the "build a robot from scratch" walkthrough
(00:11:11-00:13:36): stream the rover's lidar out so RViz shows live returns
as it patrols.

Why RTX lidar and not the walkthrough's physics Lidar
------------------------------------------------------
The walkthrough adds a legacy physics Lidar (omni.isaac.range_sensor, prim
type "Lidar") and reads it with IsaacReadLidarBeams. That does not work on
THIS robot, and the failure is visible rather than subtle: a physics Lidar is
a PhysX raycast sensor, so its beams and its drawLines/drawPoints debug
visualisation come from the pose PhysX holds for it. rover_01 is driven by
time samples in scenario/timeline.usda, not by PhysX, so the sensor never
moves -- the beams hang at the spawn point while the rover drives away.
(Making the rover root a kinematic rigid body did not bridge that gap
either.) The walkthrough gets away with it because its robot is genuinely
PhysX-driven via a differential controller.

The RTX lidar authored in simulation/sensors.usda
(Sensors/lidar_mount/obstacle_lidar) is a RENDER-pipeline sensor, so it
follows the composed USD transform exactly like the meshes do. It is the
only lidar that can track an animation-driven robot, so it is what this
graph publishes from.

Run inside Isaac Sim
---------------------
Open root.usda, enable the ROS 2 Bridge extension (Window > Extensions,
search "ros2") and make sure ROS 2 is sourced on your system -- both required
by the walkthrough too. Then in Window > Script Editor:

    import sys; sys.path.append("<repo>/tools")
    import ros_bridge_setup
    ros_bridge_setup.build()

Verify (the walkthrough's own test): press Play, then in a terminal
`ros2 topic list` should show /rover_01/scan. Launch rviz2, set Fixed Frame
to rover_01_lidar_frame, add a LaserScan display on that topic.

API-drift note
---------------
ROS2RtxLidarHelper is the documented one-node path for RTX-lidar -> ROS 2 in
recent Isaac Sim, but the exact node type token and its input names are NOT
verified against a live 6.0.1 Kit session -- the same caveat
simulation/sensors.usda already carries for the RTX lidar prim itself. This
is built through omni.graph.core.Controller.edit() rather than hand-authored
USD precisely so that a wrong token raises a Python traceback naming the bad
node or pin, instead of silently producing a graph that does nothing. If it
raises, fix that one token; the Action Graph editor shows the real names.
"""

ROVER_PATH = "/World/Scenario/Fleet/rover_01"
LIDAR_PATH = f"{ROVER_PATH}/Sensors/lidar_mount/obstacle_lidar"
GRAPH_PATH = f"{ROVER_PATH}/ROS2LidarGraph"
TOPIC_NAME = "/rover_01/scan"
FRAME_ID = "rover_01_lidar_frame"

_NODES = [
    ("OnTick", "omni.graph.action.OnPlaybackTick"),
    ("RtxLidarPub", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
]

_CONNECT = [
    ("OnTick.outputs:tick", "RtxLidarPub.inputs:execIn"),
]

_SET_VALUES = [
    ("RtxLidarPub.inputs:renderProductPath", ""),   # filled in by build()
    ("RtxLidarPub.inputs:topicName", TOPIC_NAME),
    ("RtxLidarPub.inputs:frameId", FRAME_ID),
    ("RtxLidarPub.inputs:type", "laser_scan"),
]


def build():
    import omni.graph.core as og
    import omni.replicator.core as rep

    # The helper node consumes a render product bound to the lidar prim --
    # that binding is what makes it a render-pipeline sensor, and therefore
    # what makes it follow the rover instead of standing still.
    render_product = rep.create.render_product(
        LIDAR_PATH, [1, 1], name="rover_01_lidar_rp")
    rp_path = (render_product.path
               if hasattr(render_product, "path") else str(render_product))

    values = [(k, rp_path) if k.endswith("renderProductPath") else (k, v)
              for k, v in _SET_VALUES]

    keys = og.Controller.Keys
    graph, nodes, _, _ = og.Controller.edit(
        {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: _NODES,
            keys.CONNECT: _CONNECT,
            keys.SET_VALUES: values,
        },
    )
    print(f"ros_bridge_setup: built {GRAPH_PATH} -- publishing {TOPIC_NAME} "
          f"(frame {FRAME_ID}) from RTX lidar {LIDAR_PATH}")
    return graph, nodes


if __name__ == "__main__":
    print(__doc__)

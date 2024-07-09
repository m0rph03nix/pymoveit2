#!/usr/bin/env python3

from threading import Thread
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import MoveIt2, MoveIt2State
from pymoveit2.robots import ur as robot
from fastapi import FastAPI
from pydantic import BaseModel
import uvicorn

# Create FastAPI app
app = FastAPI()

# Define request model
class MoveRequest(BaseModel):
    joint_positions: list[float]
    synchronous: bool = True
    cancel_after_secs: float = 0.0

# ROS2 Initialization
rclpy.init()

# Create node for this example
node = Node("ex_joint_goal")

# Declare parameter for joint positions
node.declare_parameter(
    "joint_positions",
    [
        -1.5708,
        -1.5708,
        -2.827,
        1.3521,
        0.0 ,
        0.0 ,   
    ],
)
node.declare_parameter("synchronous", True)
# If non-positive, don't cancel. Only used if synchronous is False
node.declare_parameter("cancel_after_secs", 0.0)
# Planner ID
node.declare_parameter("planner_id", "RRTConnectkConfigDefault")

# Create callback group that allows execution of callbacks in parallel without restrictions
callback_group = ReentrantCallbackGroup()

# Create MoveIt 2 interface
moveit2 = MoveIt2(
    node=node,
    joint_names=robot.joint_names(),
    base_link_name=robot.base_link_name(),
    end_effector_name=robot.end_effector_name(),
    group_name=robot.MOVE_GROUP_ARM,
    callback_group=callback_group,
)
moveit2.planner_id = (
    node.get_parameter("planner_id").get_parameter_value().string_value
)

# Spin the node in background thread(s) and wait a bit for initialization
executor = rclpy.executors.MultiThreadedExecutor(2)
executor.add_node(node)
executor_thread = Thread(target=executor.spin, daemon=True)
executor_thread.start()
node.create_rate(1.0).sleep()

# Scale down velocity and acceleration of joints (percentage of maximum)
moveit2.max_velocity = 0.5
moveit2.max_acceleration = 0.5

@app.post("/move")
async def move_robot(request: MoveRequest):
    joint_positions = request.joint_positions
    synchronous = request.synchronous
    cancel_after_secs = request.cancel_after_secs

    # Move to joint configuration
    node.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions)}}}")
    moveit2.move_to_configuration(joint_positions)
    if synchronous:
        # Note: the same functionality can be achieved by setting
        # `synchronous:=false` and `cancel_after_secs` to a negative value.
        moveit2.wait_until_executed()
        return {"status": "Movement completed synchronously"}
    else:
        # Wait for the request to get accepted (i.e., for execution to start)
        print("Current State: " + str(moveit2.query_state()))
        rate = node.create_rate(10)
        while moveit2.query_state() != MoveIt2State.EXECUTING:
            rate.sleep()

        # Get the future
        print("Current State: " + str(moveit2.query_state()))
        future = moveit2.get_execution_future()

        # Cancel the goal
        if cancel_after_secs > 0.0:
            # Sleep for the specified time
            sleep_time = node.create_rate(cancel_after_secs)
            sleep_time.sleep()
            # Cancel the goal
            print("Cancelling goal")
            moveit2.cancel_execution()

        # Wait until the future is done
        while not future.done():
            rate.sleep()

        # Print the result
        print("Result status: " + str(future.result().status))
        print("Result error code: " + str(future.result().result.error_code))

        return {
            "status": "Movement completed asynchronously",
            "result_status": future.result().status,
            "result_error_code": future.result().result.error_code
        }

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
    rclpy.shutdown()
    executor_thread.join()
        
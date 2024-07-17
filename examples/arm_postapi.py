#!/usr/bin/env python3

#!/usr/bin/env python3

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import MoveIt2, MoveIt2State
from pymoveit2.robots import ur as robot
from fastapi import FastAPI
from pydantic import BaseModel
from std_srvs.srv import Trigger
import uvicorn

# Create FastAPI app
app = FastAPI()

# Define request model for movement
class MoveRequest(BaseModel):
    joint_positions: list[float]
    synchronous: bool = True
    cancel_after_secs: float = 0.0

# Define request model for gripper control
class GraspRequest(BaseModel):
    service_name: str

# ROS2 Initialization
rclpy.init()

# Create node for this example
node = Node("gra_postapi")

# Declare parameter for joint positions
node.declare_parameter(
    "joint_positions",
    [
        -1.5708,
        -1.5708,
        -2.827,
        1.3521,
        0.0,
        0.0,
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
        moveit2.wait_until_executed()
        return {"status": "Movement completed synchronously"}
    else:
        node.get_logger().info(f"Current State: {moveit2.query_state()}")
        rate = node.create_rate(10)
        while moveit2.query_state() != MoveIt2State.EXECUTING:
            await rate.sleep()

        future = moveit2.get_execution_future()

        if cancel_after_secs > 0.0:
            sleep_time = node.create_rate(cancel_after_secs)
            await sleep_time.sleep()
            node.get_logger().info("Cancelling goal")
            moveit2.cancel_execution()

        while not future.done():
            await rate.sleep()

        result_status = future.result().status
        result_error_code = future.result().result.error_code
        node.get_logger().info(f"Result status: {result_status}")
        node.get_logger().info(f"Result error code: {result_error_code}")

        return {
            "status": "Movement completed asynchronously",
            "result_status": result_status,
            "result_error_code": result_error_code
        }

@app.post("/grasp")
async def control_gripper(request: GraspRequest):
    service_name = request.service_name

    client = node.create_client(Trigger, service_name)
    if not client.wait_for_service(timeout_sec=5.0):
        return {"error": f"Service {service_name} not available"}

    trigger_request = Trigger.Request()
    future = client.call_async(trigger_request)

    while not future.done():
        rclpy.spin_once(node, timeout_sec=1.0)

    response = future.result()
    if response.success:
        return {"status": "Gripper action completed", "message": response.message}
    else:
        return {"status": "Gripper action failed", "message": response.message}

def main():
    # Create an event loop for FastAPI and rclpy
    import asyncio

    async def ros_spin():
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            await asyncio.sleep(0.1)

    loop = asyncio.get_event_loop()
    loop.create_task(ros_spin())

    # Run the FastAPI server in the same event loop
    config = uvicorn.Config(app, host="0.0.0.0", port=8000, loop="asyncio")
    server = uvicorn.Server(config)
    loop.run_until_complete(server.serve())

if __name__ == "__main__":
    main()
    rclpy.shutdown()

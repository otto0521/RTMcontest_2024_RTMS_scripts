import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import subprocess
import time
import asyncio
import math

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')

        self.turtle_publishers = {}

        self.turtle_names = ['turtle1', 'turtle2', 'turtle3']
        for turtle in self.turtle_names:
            topic_name = f'/{turtle}/cmd_vel'
            self.turtle_publishers[turtle] = self.create_publisher(Twist, topic_name, 10)
            self.get_logger().info(f'Publisher created for {topic_name}')

    async def move_turtle_in_circle(self, turtle_name):
        publisher = self.turtle_publishers[turtle_name]
        twist = Twist()
        twist.linear.x = 1.0
        twist.angular.z = 0.5 

        self.get_logger().info(f"{turtle_name} starts moving in a circle.")
        while rclpy.ok():
            publisher.publish(twist)
            await asyncio.sleep(0.5)

    async def move_turtle_vertically(self, turtle_name):
        publisher = self.turtle_publishers[turtle_name]
        twist = Twist()
        moving_up = True  

        while rclpy.ok():
            if moving_up:
                twist.linear.x = 4.0 
            else:
                twist.linear.x = -4.0

            publisher.publish(twist)
            await asyncio.sleep(5.0) 
            moving_up = not moving_up

    async def move_turtle_in_triangle(self, turtle_name):
        publisher = self.turtle_publishers[turtle_name]
        twist = Twist()

        self.get_logger().info(f"{turtle_name} starts moving in a triangle.")

        while rclpy.ok():
            for _ in range(3):
                twist.linear.x = 3.0
                twist.angular.z = 0.0
                publisher.publish(twist)
                await asyncio.sleep(4.0)

                twist.linear.x = 0.0
                twist.angular.z = 2 * math.pi / 3 
                publisher.publish(twist)
                await asyncio.sleep(1.0)

            twist.angular.z = 0.0
            publisher.publish(twist)

    async def start_movements(self):
        await asyncio.gather(
            self.move_turtle_in_circle('turtle1'),
            self.move_turtle_vertically('turtle2'),
            self.move_turtle_in_triangle('turtle3'),
        )

async def main():
    turtles = [
        {"name": "turtle1", "script": "scripts/sample_turtle1_script/ros2_robot_state_publisher.py"},
        {"name": "turtle2", "script": "scripts/sample_turtle2_script/ros2_robot_state_publisher.py"},
        {"name": "turtle3", "script": "scripts/sample_turtle3_script/ros2_robot_state_publisher.py"},
    ]

    processes = []

    try:
        for turtle in turtles:
            process = subprocess.Popen(["python", turtle["script"]])
            processes.append(process)
            print(f"Started state publisher for {turtle['name']}")
            time.sleep(2) 


        rclpy.init()

        controller = TurtleController()

        try:
            print("Starting turtle movements...")
            await controller.start_movements()
        finally:
            controller.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    except KeyboardInterrupt:
        print("Stopping all state publishers...")
    finally:

        for process in processes:
            process.terminate()
        for process in processes:
            process.wait()

if __name__ == '__main__':
    asyncio.run(main())

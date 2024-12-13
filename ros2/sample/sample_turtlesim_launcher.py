import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
import subprocess
import time
import os

class TurtleSpawner(Node):
    def __init__(self):
        super().__init__('turtle_spawner')
        self.turtle_positions = [
            (3.5, 5.5, 90.0),  
            (7.5, 5.5, 0.0),     
        ]

    def spawn_turtles(self):
        for i, (x, y, theta) in enumerate(self.turtle_positions, start=2):  
            self.get_logger().info(f'Spawning turtle{i} at ({x}, {y})')


            client = self.create_client(Spawn, '/spawn')

            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Waiting for /spawn service...')

            request = Spawn.Request()
            request.x = x
            request.y = y
            request.theta = theta
            request.name = f'turtle{i}'

            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future)

            if future.result() is not None:
                self.get_logger().info(f'turtle{i} spawned successfully!')
            else:
                self.get_logger().error(f'Failed to spawn turtle{i}')

def main():
    rclpy.init()

    process = subprocess.Popen(['ros2', 'run', 'turtlesim', 'turtlesim_node'])
    time.sleep(2) 

    os.system('xdotool search --name "Turtlesim" windowsize 300 300')
    spawner = TurtleSpawner()
    try:
        spawner.spawn_turtles()
    finally:
        spawner.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    process.terminate()
    process.wait()

if __name__ == '__main__':
    main()

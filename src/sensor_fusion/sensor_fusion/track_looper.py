import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import sys

def create_pose(navigator, x, y, yaw=0.0):
    """Helper function to generate a PoseStamped object"""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    return pose

def main():
    rclpy.init()
    navigator = BasicNavigator()
    # Wait for Nav2
    navigator.waitUntilNav2Active()

    # Track choice handling
    track_choice = 'inner' 
    if len(sys.argv) > 1:
        track_choice = sys.argv[1].lower()

    waypoints = []

    if track_choice == 'inner':
        print("Starting INNER track loop (Clockwise)...")
        waypoints.append(create_pose(navigator, -2.47, -2.27)) # Bottom Right
        waypoints.append(create_pose(navigator, -2.42, 2.07))  # Bottom Left
        waypoints.append(create_pose(navigator, 2.60, 2.04))   # Top Left
        waypoints.append(create_pose(navigator, 2.72, -2.38))  # Top Right
    else:
        print("Starting OUTER track loop (Couter-Clockwise)...")
        pass

    # Loops infinitely
    while rclpy.ok():
        print(f"Sending {len(waypoints)} waypoints to Nav2...")
        navigator.followWaypoints(waypoints)

        # Wait for the car to finish the loop
        while not navigator.isTaskComplete():
            pass

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print("Lap completed! Restarting loop...")
        elif result == TaskResult.CANCELED:
            print("Task was canceled. Exiting.")
            break
        elif result == TaskResult.FAILED:
            print("Task failed! Retrying in 5 seconds...")

    navigator.lifecycleShutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
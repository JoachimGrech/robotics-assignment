import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SpawnEntity, DeleteEntity
from geometry_msgs.msg import Pose
import time

def make_box_sdf(name, sx=0.4, sy=0.4, sz=0.2, color="Gazebo/Yellow"):
    return f"""<sdf version='1.7'>
  <model name='{name}'>
    <static>false</static>
    <link name='link'>
      <pose>0 0 {sz/2} 0 0 0</pose>
      <visual name='visual'>
        <geometry>
          <box><size>{sx} {sy} {sz}</size></box>
        </geometry>
        <material>
          <ambient>1 0.8 0 1</ambient>
          <diffuse>1 0.8 0 1</diffuse>
        </material>
      </visual>
      <collision name='collision'>
        <geometry>
          <box><size>{sx} {sy} {sz}</size></box>
        </geometry>
      </collision>
      <inertial>
        <mass>1.0</mass>
      </inertial>
    </link>
  </model>
</sdf>"""

def make_bin_sdf(name, sx=0.6, sy=0.6, sz=0.4):
    # hollow-ish bin: represented as single box for simplicity
    return f"""<sdf version='1.7'>
  <model name='{name}'>
    <static>true</static>
    <link name='link'>
      <pose>0 0 {sz/2} 0 0 0</pose>
      <visual name='visual'>
        <geometry>
          <box><size>{sx} {sy} {sz}</size></box>
        </geometry>
        <material>
          <ambient>0.5 0.5 0.5 1</ambient>
          <diffuse>0.7 0.7 0.7 1</diffuse>
        </material>
      </visual>
      <collision name='collision'>
        <geometry>
          <box><size>{sx} {sy} {sz}</size></box>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>"""

def make_robot_sdf(name, color="Blue"):
    color_map = {
        "Blue": "0 0 1",
        "Red": "1 0 0",
        "Green": "0 1 0"
    }
    rgb = color_map.get(color, "0 0 1")
    return f"""<sdf version='1.7'>
  <model name='{name}'>
    <static>false</static>
    <link name='base_link'>
      <pose>0 0 0.12 0 0 0</pose>
      <visual name='visual'>
        <geometry>
          <box><size>0.5 0.4 0.25</size></box>
        </geometry>
        <material>
          <ambient>{rgb} 1</ambient>
          <diffuse>{rgb} 1</diffuse>
        </material>
      </visual>
      <collision name='collision'>
        <geometry>
          <box><size>0.5 0.4 0.25</size></box>
        </geometry>
      </collision>
      <inertial>
        <mass>5.0</mass>
      </inertial>
    </link>
  </model>
</sdf>"""

class WorldManager(Node):
    def __init__(self):
        super().__init__('world_manager')
        self.get_logger().info("WorldManager started, waiting for Gazebo ROS services...")
        
        # Wait longer for Gazebo to fully initialize
        time.sleep(3.0)

        # client for spawn
        self.spawn_cli = self.create_client(SpawnEntity, '/spawn_sdf')

        # wait for spawn service with longer timeout
        self.get_logger().info("Waiting for /spawn_sdf service...")
        for i in range(30):  # Try for 30 seconds
            if self.spawn_cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'/spawn_sdf service available after {i+1} seconds')
                break
            if i % 5 == 0:
                self.get_logger().info(f'Still waiting... ({i+1}/30)')
        else:
            self.get_logger().error('/spawn_sdf service not available after 30s')
            return

        # Give extra time for Gazebo to be ready
        time.sleep(2.0)
        
        # spawn everything
        self.spawn_test_scene()

    def _call_spawn(self, name, sdf, x=0.0, y=0.0, z=0.1, timeout=15.0):
        req = SpawnEntity.Request()
        req.entity_factory.name = name
        req.entity_factory.sdf = sdf
        req.entity_factory.pose.position.x = float(x)
        req.entity_factory.pose.position.y = float(y)
        req.entity_factory.pose.position.z = float(z)
        req.entity_factory.relative_to = "world"

        fut = self.spawn_cli.call_async(req)
        self.get_logger().info(f"Spawn request sent for {name} at ({x}, {y}, {z})")
        
        start_time = time.time()
        while time.time() - start_time < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if fut.done():
                try:
                    res = fut.result()
                    if hasattr(res, 'success') and res.success:
                        self.get_logger().info(f"Ã¢Å“â€œ Successfully spawned {name}")
                        return True
                    else:
                        self.get_logger().warn(f"Spawn response indicates failure for {name}")
                        return False
                except Exception as e:
                    self.get_logger().error(f"Exception in spawn for {name}: {e}")
                    return False
        
        self.get_logger().error(f"Ã¢Å“â€” Spawn call timed out for {name} after {timeout}s")
        return False

    def spawn_test_scene(self):
        self.get_logger().info("=" * 50)
        self.get_logger().info("STARTING TO SPAWN SCENE")
        self.get_logger().info("=" * 50)
        
        # spawn bins (static) FIRST - less likely to fail
        bins = [
            ('bin_0', -1.2, -0.8),
            ('bin_1',  1.2,  0.8),
            ('bin_2',  0.0,  1.2),
            ('bin_3', -0.5, -1.2),
        ]
        for name, x, y in bins:
            sdf = make_bin_sdf(name, 0.6, 0.6, 0.45)
            if self._call_spawn(name, sdf, x, y, 0.0, timeout=10.0):
                time.sleep(0.3)  # Small delay between spawns
            else:
                self.get_logger().warn(f"Failed to spawn {name}, continuing...")

        time.sleep(1.0)
        
        # spawn objects
        objs = [
            ('object_0', -1.0,  0.5),
            ('object_1', -0.6,  0.3),
            ('object_2', -0.2,  0.0),
            ('object_3',  0.2, -0.5),
            ('object_4',  0.7,  0.2),
            ('object_5',  1.0, -0.4),
        ]
        for i, (name, x, y) in enumerate(objs):
            sdf = make_box_sdf(name, 0.3, 0.3, 0.15)
            if self._call_spawn(name, sdf, x, y, 0.08, timeout=10.0):
                time.sleep(0.3)
            else:
                self.get_logger().warn(f"Failed to spawn {name}, continuing...")

        time.sleep(1.0)
        
        # spawn robots LAST
        robots = [
            ('sortabot', make_robot_sdf('sortabot', 'Blue'), 0.0, 0.0),
            ('childabot_0', make_robot_sdf('childabot_0', 'Red'), 0.5, -0.5),
        ]
        
        for name, sdf, x, y in robots:
            if self._call_spawn(name, sdf, x, y, 0.12, timeout=10.0):
                time.sleep(0.5)
            else:
                self.get_logger().warn(f"Failed to spawn {name}, continuing...")

        self.get_logger().info("=" * 50)
        self.get_logger().info("SCENE SPAWNING COMPLETE")
        self.get_logger().info("=" * 50)
        
        # Keep node alive
        self.get_logger().info("WorldManager ready. Press Ctrl+C to exit.")

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WorldManager()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info("Shutting down WorldManager...")
    except Exception as e:
        if node:
            node.get_logger().error(f"WorldManager error: {e}")
        else:
            print(f"WorldManager error: {e}")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

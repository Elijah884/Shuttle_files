import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from zlac8015d import ZLAC8015D

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(
            String, 'motor_control', self.listener_callback, 10)
        self.subscription
        
        self.motors = ZLAC8015D.Controller(port="/dev/ttyUSB0")
        self.motors.clear_alarm()
        self.motors.disable_motor()

        self.motors.set_accel_time(1000,1000)
        self.motors.set_decel_time(1000,1000)

        self.motors.set_mode(3)
        self.motors.enable_motor()

        self.cmds = [0, 0]
        self.motors.set_rpm(self.cmds[0],self.cmds[1])

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')
        speed = int(msg.data)
        self.cmds = [speed, -speed]
        self.motors.set_rpm(self.cmds[0],self.cmds[1])
        fault = self.motors.get_fault_code()
        if fault[0][0] == True or fault[1][0] == True:
            self.get_logger().info(f'Fault: "{fault}"')

def main(args=None):
    rclpy.init(args=args)
    subscriber_node = SubscriberNode()

    try:
        rclpy.spin(subscriber_node)
    except KeyboardInterrupt:
        subscriber_node.get_logger().info('Keyboard interrupt receiver, powering off.')
    finally:
        subscriber_node.motors.disable_motor()
        subscriber_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

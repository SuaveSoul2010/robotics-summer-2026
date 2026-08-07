import rclpy
from rclpy.node import Node
from calculator.srv import Calc


class CalculatorClient(Node):

    def __init__(self):
        super().__init__('calculator_client')
        self.client = self.create_client(Calc, 'calc_service')

    def send_request(self, a, b, op):

        req = Calc.Request()
        req.a = float(a)
        req.b = float(b)
        req.operation = op

        return self.client.call_async(req)


def main():
    rclpy.init()
    node = CalculatorClient()

    while not node.client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("Waiting for service...")

    future = node.send_request(20, 3, "subtract")

    rclpy.spin_until_future_complete(node, future)

    if future.result():
        node.get_logger().info(f"Result: {future.result().result}")

    node.destroy_node()
    rclpy.shutdown()

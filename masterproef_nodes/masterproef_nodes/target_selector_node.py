import rclpy
from std_msgs.msg import String
from target_selector import TargetSelector
#TO BE IMPLEMENTED
# -parse_coordinates(data)
def callback(data):
    coordinates = parse_coordinates(data.data)
    selector = TargetSelector()
    shortest_distance, closest_person = selector.get_shortest_distance(coordinates, 'R', 'P', None)
    
    if shortest_distance is not None and closest_person is not None:
        result = f"Shortest distance: {shortest_distance}, Closest person coordinates: {closest_person}"
        rclpy.loginfo(result)
        pub.publish(result)

def parse_coordinates(data):
    # Implement your logic to parse the incoming data into coordinates
    # Example: [(1, 2, 'R'), (4, 6, 'P'), ...]
    # For simplicity, let's assume the data is a string of tuples
    coordinates = eval(data)
    return coordinates

def target_selector_node():
    global pub
    rclpy.init_node('target_selector_node', anonymous=True)
    rclpy.Subscriber('coordinates_topic', String, callback)
    pub = rclpy.Publisher('target', String, queue_size=10)
    rclpy.spin()

if __name__ == '__main__':
    try:
        target_selector_node()
    except rclpy.ROSInterruptException:
        pass
import math

class TargetSelector:
    def __init__(self):
        self.targets = []

    def add_target(self, x, y, target_id):
        self.targets.append({'x': x, 'y': y, 'id': target_id})

    def calculate_distance(self, id1, id2):
        points_id1 = [target for target in self.targets if target['id'] == id1]
        points_id2 = [target for target in self.targets if target['id'] == id2]

        if not points_id1 or not points_id2:
            return None

        # Assuming we calculate the distance between the first points of each ID
        point1 = points_id1[0]
        point2 = points_id2[0]

        distance = math.sqrt((point2['x'] - point1['x'])**2 + (point2['y'] - point1['y'])**2)
        return distance

    def process_coordinates(self, coordinates, robot_id, person_id, null_id):
        self.targets.clear()  # Clear previous targets
        for coord in coordinates:
            x, y, target_id = coord
            if target_id != null_id:
                self.add_target(x, y, target_id)

        robot_points = [target for target in self.targets if target['id'] == robot_id]
        person_points = [target for target in self.targets if target['id'] == person_id]

        distances = []
        for robot_point in robot_points:
            for person_point in person_points:
                distance = math.sqrt((person_point['x'] - robot_point['x'])**2 + (person_point['y'] - robot_point['y'])**2)
                distances.append((distance, person_point))

        return distances

    def get_shortest_distance(self, coordinates, robot_id, person_id, null_id):
        distances = self.process_coordinates(coordinates, robot_id, person_id, null_id)
        if not distances:
            return None, None

        shortest_distance, closest_person = min(distances, key=lambda x: x[0])
        return shortest_distance, closest_person

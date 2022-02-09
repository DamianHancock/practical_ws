import csv
import os, rospkg

class CoordNav:
    def __init__(self):
        self.rospack = rospkg.RosPack()
        self.status = 0
        self.final_coords = []
        self.text = None
        self.elapsed_time = 0.0
        self.fname = "coordinates.csv"

    def set_goals(self):
        with open(os.path.join(self.rospack.get_path("autonomous_nav"), "resources", self.fname), 'w') as f:
            writer = csv.writer(f)
            writer.writerows([
                [7.0, 8.0, 0.75, 0.66],
                [5.0, 4.0, 0.75, 0.66],
                [1.0, 1.0, 0.75, 0.66]
            ])

    def status_cb(self, msg):
        if msg.status.status >= 3:
            self.status = msg.status.status  
            self.text = msg.status.text

    def pos_cb(self, msg):
        final = msg.feedback.base_position.pose
        self.final_coords = list(map(str, [
            round(final.position.x,3),
            round(final.position.y,3),
            round(final.orientation.z,3),
            round(final.orientation.w,3)
        ]))

    def get_coord(self):
        goals = []
        with open(os.path.join(self.rospack.get_path("autonomous_nav"), "resources", self.fname), 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                goals.append(list(map(float, row)))

        return goals

    def save_coord(self):
        with open(os.path.join(self.rospack.get_path("autonomous_nav"), "resources", self.fname), 'a+') as f:
            writer = csv.writer(f)
            self.final_coords.append(self.text)
            self.final_coords.append(round(self.elapsed_time, 3))
            writer.writerow(self.final_coords)
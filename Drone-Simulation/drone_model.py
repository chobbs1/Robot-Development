class Drone:

    def __init__(self,x,y,theta):
        self.x = x
        self.y = y
        self.theta = theta

    def draw_drone(x,y,theta):
        canvas.delete("all")
        c_theta = math.cos(theta)
        s_theta = math.sin(theta)

        unrotated_vertices = [
            [x - arm_length, y - arm_width],
            [x + arm_length, y - arm_width],
            [x + arm_length, y + arm_width],
            [x - arm_length, y + arm_width],
        ]
        rotated_vertices = []

        for coords in unrotated_vertices:
            x0 = coords[0]
            y0 = coords[1]

            x1 = x0*c_theta - y0*s_theta
            y1 = x0*s_theta + y0*c_theta

            rotated_vertices.append([x1,y1])

        drone = canvas.create_polygon(rotated_vertices, fill="black")

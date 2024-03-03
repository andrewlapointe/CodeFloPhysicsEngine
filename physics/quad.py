class QuadTree():
    MAX_DEPTH = 5
    MAX_OBJECTS = 1
    grid_list = []

    def __init__(self, x:int, y:int, width:int, height:int, level:int = 0) -> None:
        self.objects = []
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.level = level
        self.top_left_node = None
        self.top_right_node = None
        self.bottom_left_node = None
        self.bottom_right_node = None
        self.mid_x = self.width // 2
        self.mid_y = self.height // 2

    def insert(self, particle):
        if particle is None or not self.in_boundary(particle):
            return

        if self.top_left_node is None and len(self.objects) < QuadTree.MAX_OBJECTS:
            self.objects.append(particle)
            return

        if self.top_left_node is None and len(self.objects) >= QuadTree.MAX_OBJECTS:
            if self.level < QuadTree.MAX_DEPTH:
                self._split()

        if self.top_left_node is not None:
            selected_node = self.select_node(particle)
            if selected_node:
                selected_node.insert(particle)
        else:
            self.objects.append(particle)

    def _split(self):
        mid_x = self.mid_x
        mid_y = self.mid_y
        self.top_left_node = QuadTree(self.x, self.y, mid_x, mid_y, self.level+1)
        self.top_right_node = QuadTree(self.x + mid_x, self.y, self.width - mid_x, mid_y, self.level+1)
        self.bottom_left_node = QuadTree(self.x, self.y + mid_y, mid_x, self.height - mid_y, self.level+1)
        self.bottom_right_node = QuadTree(self.x + mid_x, self.y + mid_y, self.width - mid_x, self.height - mid_y, self.level+1)
        # QuadTree.grid_list.append([[self.x+(self.width // 2), self.y], [self.x+(self.width // 2), self.y+self.height]])
        # QuadTree.grid_list.append([[self.x, self.y+(self.height // 2)], [self.x+self.width, self.y+(self.height // 2)]])
        
        temp_objects = self.objects[:]
        self.objects.clear()
        for obj in temp_objects:
            self.insert(obj)

    def select_node(self, particle):
        if particle.position[0] < self.x + self.width / 2:
            if particle.position[1] < self.y + self.height / 2:
                return self.top_left_node
            else:
                return self.bottom_left_node
        else:
            if particle.position[1] < self.y + self.height / 2:
                return self.top_right_node
            else:
                return self.bottom_right_node

    def in_boundary(self, particle):
        return (self.x <= particle.position[0] <= self.x + self.width and
                self.y <= particle.position[1] <= self.y + self.height)

    def search(self, particle):
        if not self.in_boundary(particle):
            raise ValueError("Invalid particle location")

        if particle in self.objects:
            print("found particle")
            return self

        if self.top_left_node is None:
            return None

        return self.select_node(particle).search(particle) if self.select_node(particle) else None

            
    def search_within_radius(self, test_particle):
        radius = test_particle.smoothing_radius
        found_particles = []
        self.__search_within_radius(test_particle, radius, found_particles)
        return found_particles

    def __search_within_radius(self, test_particle, radius, found_particles):
        # If this node's boundaries don't intersect with the search circle, return early
        if not self._intersects_circle(test_particle.position, radius):
            return
        
        # Check each particle in this node
        for particle in self.objects:
            if self._distance(particle.position, test_particle.position) <= radius and particle is not test_particle:
                found_particles.append(particle)
        
        # If this is a leaf node, return
        if self.top_left_node is None:
            return
        
        # Otherwise, recurse into child nodes
        self.top_left_node.__search_within_radius(test_particle, radius, found_particles)
        self.top_right_node.__search_within_radius(test_particle, radius, found_particles)
        self.bottom_left_node.__search_within_radius(test_particle, radius, found_particles)
        self.bottom_right_node.__search_within_radius(test_particle, radius, found_particles)

    def _distance(self, pos1, pos2):
        # Euclidean distance between two points
        return ((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2) ** 0.5

    def _intersects_circle(self, circle_center, radius):
        # Check if the node's rectangle intersects with the search circle
        # This is a simplified check; for more accurate results, you might need a more complex intersection logic
        closest_x = max(self.x, min(circle_center[0], self.x + self.width))
        closest_y = max(self.y, min(circle_center[1], self.y + self.height))
        distance_x = circle_center[0] - closest_x
        distance_y = circle_center[1] - closest_y

        return (distance_x ** 2 + distance_y ** 2) <= radius ** 2
class BlockMapper:
    def __init__(self):
        self.blocks = {}  # Dictionary to store block ID and its coordinates

    def process_tag_data(self, tag_data):
        """
        Process the output of ARTagDetector and update the block information.
        :param tag_data: Output from ARTagDetector, a list of arrays with tag IDs and corner coordinates.
        """
        for tag in tag_data:
            tag_id, corners = tag[0], tag[1]
            self.blocks[tag_id] = self.calculate_center(corners)

    def calculate_center(self, corners):
        """
        Calculate the center point of the tag from its corner coordinates.
        :param corners: List of corner coordinates for a tag.
        :return: Tuple representing the center (x, y) of the tag.
        """
        x_coords = [corners[i] for i in range(0, len(corners), 2)]
        y_coords = [corners[i] for i in range(1, len(corners), 2)]
        center_x = sum(x_coords) / len(x_coords)
        center_y = sum(y_coords) / len(y_coords)
        return center_x, center_y

    def get_block_count(self):
        """
        Get the total number of unique blocks detected.
        :return: Integer representing the number of blocks.
        """
        return len(self.blocks)

    def get_block_locations(self):
        """
        Get the locations of all detected blocks.
        :return: Dictionary with block IDs as keys and their locations as values.
        """
        return self.blocks

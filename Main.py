import cv2
import numpy as np
from ar_tag_detection import ARTagDetector
from block_mapper import BlockMapper

def get_user_input():
    num_columns = int(input("Enter the number of columns: "))
    blocks_per_column = int(input("Enter the number of blocks per column: "))
    return num_columns,   blocks_per_column

def estimate_depth(size_of_marker):
    """
    Simplified depth estimation based on the size of the AR marker.
    In reality, this would require more sophisticated methods or hardware.
    """
    # Assuming a linear relationship for simplicity (not accurate in real-world scenarios)
    return 1000 / size_of_marker  # Example conversion factor

def compute_stacking_plan(mapper, num_columns, blocks_per_column):
    block_locations = mapper.get_block_locations()
    block_ids = list(block_locations.keys())

    # Placeholder for stacking logic: simply groups blocks into columns based on proximity
    # This is a very basic implementation and should be replaced with a more robust algorithm
    columns = [[] for _ in range(num_columns)]
    for i, block_id in enumerate(block_ids):
        column_index = i % num_columns
        columns[column_index].append(block_id)

    # Output the stacking plan
    for i, column in enumerate(columns):
        print(f"Column {i + 1}: Blocks {column}")

class BlockMapper:
    def __init__(self):
        self.blocks = {}  # Dictionary to store block ID and its coordinates

    def process_tag_data(self, tag_data):
        for tag in tag_data:
            tag_id, corners = tag[0], tag[1]
            self.blocks[tag_id] = self.calculate_center(corners)

    def calculate_center(self, corners):
        x_coords = [corners[i] for i in range(0, len(corners), 2)]
        y_coords = [corners[i] for i in range(1, len(corners), 2)]
        center_x = sum(x_coords) / len(x_coords)
        center_y = sum(y_coords) / len(y_coords)
        return center_x, center_y

    def get_block_count(self):
        return len(self.blocks)

    def get_block_locations(self):
        return self.blocks

def main():
    detector = ARTagDetector()
    mapper = BlockMapper()

    num_columns, blocks_per_column = get_user_input()

    try:
        while True:
            frame, tag_info = detector.detect_tags()
            if frame is not None:
                mapper.process_tag_data(tag_info)
                print(f'Total Unique Blocks Detected: {mapper.get_block_count()}')
                cv2.imshow('Frame', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    finally:
        detector.release()
        cv2.destroyAllWindows()

    compute_stacking_plan(mapper, num_columns, blocks_per_column)

if __name__ == "__main__":
    main()

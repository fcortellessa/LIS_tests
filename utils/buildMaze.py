# --------- Puzzle World Class ---------
"""
The PuzzleWorld class is used to build a puzzle world with obstacles and borders. 
To change the position of the objects that build the puzzle world, change the positions in the build method.
"""

import robotic as ry

class PuzzleWorld:
    def __init__(self, C, parent=None, pos_puzzleWorld=[0.0, 0.3, 0.05], q_start=[-4*0.04, 0.0, 0.015], q_goal=[-4*0.04, 4*0.04, 0.005]):
        """
        Initialize the PuzzleWorld class with given parameters.

        Parameters:
        - C: Configuration object
        - parent: Parent frame (optional)
        - pos_puzzleWorld: Position of the puzzle world
        - q_start: Start position
        - q_goal: Goal position
        """
        self.C = C
        self.parent = parent
        self.pos_puzzleWorld = pos_puzzleWorld
        self.q_goal = q_goal
        self.q_start = q_start

    def build(self):
        """
        Build the puzzle world with the specified obstacles and borders.
        """
        cube_positions = [
            [-4.5*0.04, 1.5*0.04, 0.0085],
            [-3.5*0.04, -2.5*0.04, 0.0085],
            [-0.5*0.04, -3.5*0.04, 0.0085]
        ]
        block_positions = [
            ([-4.0*0.04, -3.5*0.04, 0.0085], "horizontal"),
            ([4.0*0.04, -1.5*0.04, 0.0085], "horizontal")
        ]
        longblock_positions = [
            ([-0.5*0.04, 3.0*0.04, 0.0085], "vertical"),
            ([-4.5*0.04, -1.0*0.04, 0.0085], "vertical")
        ]
        corner_position = ([4.0*0.04, -4.5*0.04, 0.0085], 'upper_right')
        
        border_height = 0.05
        border_thickness = 0.1
        border_positions = [
            ([0.2+border_thickness/2, 0, border_height/2], 'vertical'),
            ([-0.2-border_thickness/2, 0, border_height/2], 'vertical'),
            ([0, 0.2+border_thickness/2, border_height/2], 'horizontal'),
            ([0, -0.2-border_thickness/2, border_height/2], 'horizontal')
        ]
    
        self._add_main_frames()
        self._add_border_boxes(border_positions, border_height, border_thickness)
        self._add_cubes(cube_positions)
        self._add_blocks(block_positions)
        self._add_long_blocks(longblock_positions)
        self._add_corner_block(corner_position)
        
        return self.C

    def _add_main_frames(self):
        """
        Add main frames for the start and goal positions.
        """
        self.C.addFrame("puzzle_world").setPosition(self.pos_puzzleWorld)
        self.C.addFrame("start", "puzzle_world") \
            .setShape(ry.ST.marker, size=[.02]) \
            .setRelativePosition(self.q_start) \
            .setColor([0, 1, 0])
        self.C.addFrame("goal", "puzzle_world") \
            .setShape(ry.ST.marker, size=[.02]) \
            .setRelativePosition(self.q_goal) \
            .setColor([1, 0, 1])

    def _add_frame(self, name, parent, shape_type, size, rel_pos, color, contact):
        """
        Add a frame to the configuration.

        Parameters:
        - name: Name of the frame
        - parent: Parent frame
        - shape_type: Shape type of the frame
        - size: Size of the frame (for ssBox: [3 widths, corner radius])
        - rel_pos: Relative position of the frame
        - color: Color of the frame
        - contact: Contact flag
        """
        if parent is None:
            return self.C.addFrame(name=name) \
                .setShape(shape_type, size) \
                .setPosition(rel_pos) \
                .setColor(color) \
                .setContact(contact)
        else:
            return self.C.addFrame(name=name, parent=parent) \
                .setShape(shape_type, size) \
                .setRelativePosition(rel_pos) \
                .setColor(color) \
                .setContact(contact)

    def _add_border_boxes(self, border_positions, border_height, border_thickness):
        """
        Add border boxes around the puzzle world.

        Parameters:
        - border_positions: List of border positions and their orientations ('vertical' or 'horizontal')
        - border_height: Height of the borders
        - border_thickness: Thickness of the borders
        """
        box_name = ['left', 'right', 'top', 'bottom']
        for i, (pos, orientation) in enumerate(border_positions):
            box_size = [border_thickness, 0.6, border_height, 0.001] if orientation == 'vertical' else [0.6, border_thickness, border_height, 0.0]
            border_box_name = f'border_box_{box_name[i]}'
            self.C.addFrame(name=border_box_name, parent='puzzle_world') \
                .setShape(ry.ST.ssBox, box_size) \
                .setRelativePosition(pos) \
                .setColor([0.8, 0.8, 0.8, 0.2]) \
                .setContact(1)


    def _add_cubes(self, cube_positions):
        """
        Add cubes to the puzzle world.

        Parameters:
        - cube_positions: List of cube positions (center of cube)
        """
        for i, pos in enumerate(cube_positions):
            self._add_frame(f'cube{i+1}', 'puzzle_world', ry.ST.ssBox, [0.04, 0.04, 0.017, 0.001], pos, [0.2, 0.2, 0.2, 0.5], 1)


    def _add_blocks(self, block_positions):
        """
        Add blocks to the puzzle world.

        Parameters:
        - block_positions: List of block positions and their orientations [[position], orientation]
        """
        for i, (pos, orientation) in enumerate(block_positions):
            size = [0.08, 0.04, 0.017, 0.001] if orientation == 'horizontal' else [0.04, 0.08, 0.017, 0.001]
            self._add_frame(f'block{i+1}', 'puzzle_world', ry.ST.ssBox, size, pos, [0.2, 0.2, 0.2, 0.5], 1)


    def _add_long_blocks(self, longblock_positions):
        """
        Add long blocks to the puzzle world.

        Parameters:
        - longblock_positions: List of long block positions and their orientations [[position], orientation]
        (position at center of block)
        """
        for i, (pos, orientation) in enumerate(longblock_positions):
            size = [0.16, 0.04, 0.017, 0.001] if orientation == 'horizontal' else [0.04, 0.16, 0.017, 0.001]
            self._add_frame(f'longBlock{i+1}', 'puzzle_world', ry.ST.ssBox, size, pos, [0.2, 0.2, 0.2, 0.5], 1)


    def _add_corner_block(self, pos, color=[0.2, 0.2, 0.2, 0.5]):
        """
        Add a corner block to the puzzle world. The corner block consists of a horizontal 4x8 block and a small 4x4 block.

        Parameters:
        - name: Name of the corner block
        - pos: Position of the corner block (position of center of horizontal 4x8 block)
        - orientation: Orientation of the corner block depending on the position of the small block w.r.t the horizontal block
          ('upper_left', 'upper_right', 'lower_left', 'lower_right')
        - color: Color of the corner block
        """
        pos, orientation = pos[0], pos[1]
        size = [0.08, 0.04, 0.017, 0.001]
        frame = self._add_frame("cornerBlock", 'puzzle_world', ry.ST.ssBox, size, pos, color, 1)
        
        offset_y = -0.04 if 'lower' in orientation else 0.04
        offset_x = -0.02 if 'left' in orientation else 0.02
        self._add_frame(f'cornerBlock_Small', 'puzzle_world', ry.ST.ssBox, [0.04, 0.04, 0.017, 0.001], [pos[0] + offset_x, pos[1] + offset_y, pos[2]], color, 1)

        return frame


    def _add_goal(self, pos, size):
        """
        Add the goal to the puzzle world.

        Parameters:
        - pos: Position of the goal
        - size: Size of the goal
        """
        return self._add_frame('goal', 'puzzle_world', ry.ST.ssBox, [size, size, 0.01, 0.001], pos, [1., 1., 0], 0)


# -------- Usage --------
"""Quick test to see if the puzzle world is built correctly."""


# # 1. build the puzzle world by itsself at the given position q_pWorld
# C = ry.Config()
# q_pWorld = [0.0, 0.3, 0.65]
# q_start = [0.10, .10, .0]
# q_goal = [-0.14, .14, .0]
# puzzle_world = PuzzleWorld(C, None, q_pWorld, q_start, q_goal)
# puzzle_world.build()
# C.view(True)

# # 2. build the puzzle world with the scenario 'pandaSingle.g'
# C = ry.Config()
# C.addFile(ry.raiPath("scenarios/pandaSingle.g"))
# puzzle_world = PuzzleWorld(C, 'None', [0.0, 0.3, 0.65], [-0.12, .0, .0], [0.12, .12, .0])
# puzzle_world.build()
# C.view(True)

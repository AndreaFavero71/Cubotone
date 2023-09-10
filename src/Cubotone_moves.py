#!/usr/bin/env python
# coding: utf-8

"""
# Andrea Favero 10 September 2023
# From Kociemba solution to robot moves

# The cube solver returns the solution that has to be translated to robot movements
# Notations for faces (as per Kocimba notations, or better URF notations...): U=Up, R=Right, F=Front, D=Down, L=Left, B=Back

# Possible moves with this simple robot
# 1) Spins the complete cube ("S") laying on the bottom face: Positive means CW lookig to the bottom face
# 2) Flips the complete cube ("F") by "moving" the Font face to Down face: Only positive values are possible
# 3) Rotates the bottom face ("R") (similarly to Spin) while costraining the 2nd and 3rd layer.

# After cube status recognition the cube is positioned with the Front face facing the viewer, and Upper facing upward
# Robot moves are notated with the 3 letters S, F, R (Spin, Flip, Rotate) followed by a number
# Positive numbers for S ans R identify CW rotation, by loking to the Down face, while negative stand for CCW
# Example S0F2R1 means: Zero cube Spin, 2 times cube Flip, 1 time (90deg) CW rotation of the 1st (Down) layer
"""


# Table to define the cube Spin necessary, based on current (DownFront) cube faces orientation
# and Next_Orientation (DownFront).
# This allows cube Flipping to bring the needed face to the bottom

import numpy as np
spinArr= np.array([('DF', 'DL',  1), ('DF', 'DR', -1), ('DF', 'DB',  2),
                   ('FU', 'FL',  1), ('FU', 'FR', -1), ('FU', 'FD',  2),
                   ('UB', 'UL',  1), ('UB', 'UR', -1), ('UB', 'UF',  2),
                   ('BD', 'BL',  1), ('BD', 'BR', -1), ('BD', 'BU',  2),
                   ('DL', 'DB',  1), ('DL', 'DF', -1), ('DL', 'DR',  2),
                   ('DR', 'DF',  1), ('DR', 'DB', -1), ('DR', 'DL',  2),
                   ('DB', 'DR',  1), ('DB', 'DL', -1), ('DB', 'DF',  2),
                   ('FL', 'FD',  1), ('FL', 'FU', -1), ('FL', 'FR',  2),
                   ('FR', 'FU',  1), ('FR', 'FD', -1), ('FR', 'FL',  2),
                   ('FD', 'FR',  1), ('FD', 'FL', -1), ('FD', 'FU',  2),
                   ('BL', 'BU',  1), ('BL', 'BD', -1), ('BL', 'BR',  2),
                   ('BU', 'BR',  1), ('BU', 'BL', -1), ('BU', 'BD',  2),
                   ('BR', 'BD',  1), ('BR', 'BU', -1), ('BR', 'BL',  2),
                   ('UL', 'UF',  1), ('UL', 'UB', -1), ('UL', 'UR',  2),
                   ('UR', 'UB',  1), ('UR', 'UF', -1), ('UR', 'UL',  2),
                   ('UF', 'UR',  1), ('UF', 'UL', -1), ('UF', 'UB',  2),
                   ('LU', 'LB',  1), ('LU', 'LF', -1), ('LU', 'LD',  2),
                   ('LB', 'LD',  1), ('LB', 'LU', -1), ('LB', 'LF',  2),
                   ('LF', 'LU',  1), ('LF', 'LD', -1), ('LF', 'LB',  2),
                   ('LD', 'LF',  1), ('LD', 'LB', -1), ('LD', 'LU',  2),
                   ('RU', 'RF',  1), ('RU', 'RB', -1), ('RU', 'RD',  2),
                   ('RF', 'RD',  1), ('RF', 'RU', -1), ('RF', 'RB',  2),
                   ('RB', 'RU',  1), ('RB', 'RD', -1), ('RB', 'RF',  2),
                   ('RD', 'RB',  1), ('RD', 'RF', -1), ('RD', 'RU',  2)],
                  dtype=[('Current', '<U2'), ('Next_Orientation', '<U2'), ('Spin', '<i4')])

# "Flip_sequences" means the (6) sequence of faces the cube can be flipped
flip_seq=['DFUBDFUB', 'DLURDLUR', 'DRULDRUL', 'DBUFDBUF', 'FLBRFLBR', 'FRBLFRBL'] # Possible flipping (face) sequences

def spin_func(current, next_orient):      # returns Spin based on Current and desired cube orientation
    try:
        spin = spinArr['Spin'][np.where((spinArr['Current'] == current) & (spinArr['Next_Orientation'] == next_orient))][0]
    except:
        spin = 0
    return spin


def c_bottom_func(solution, move):        # returns the current face on bottom position
    if move == 0:
#         c_bottom = 'D'   # until 20230718, cause the cube was forced to initial pos after scanning
        c_bottom = 'R'     # from 20230718, cause the cube is left on the last scan position
    else:
        c_bottom = 'Error'
    return c_bottom


def c_front_func(solution, move):         # returns the next face required on bottom position
    if move == 0:
#         c_front = 'F'   # until 20230718, cause the cube was forced to initial pos after scanning
        c_front = 'B'     # from 20230718, cause the cube is left on the last scan position
    else:
        c_front = 'Error'
    return c_front


def n_bottom_func(solution):        # returns the next face required on bottom position
    try:
        n_bottom = solution[0]
    except:
        n_bottom = 'Error'
    return n_bottom


def c_orient_func(c_bottom, c_front):     # returns the current cube orintation: DownFront faces of the cube
    return str(c_bottom + c_front)


def n_orient_func(n_bottom, n_front):     # returns the desired cube orientation (DownFront) faces of the cube
    return str(n_bottom + n_front)


def c_n_bottom_func(c_bottom,n_bottom):
    # returns the current bottom and new desired bottom faces of the cube
    # this determines if a cube Spin is requested prior to Flip it
    return str(c_bottom + n_bottom)


def flips_func(c_bot, n_bot, c_orient, flip_sequences):
    # returns the amount of cube Flips (0 to 2) to get the new bottom facing downward
    # return the new face oriented toward the Front
    # current cube orientation is used to check in the new bottom is within the same Flip sequence
    
    
    results=[]
    min_result = 0
    for flip_sequence in flip_sequences:            # check the current cube orientation wrt the 6 possile Flip_sequences
        if c_orient in flip_sequence:
            c_flip_seq = flip_sequence              # return the cFlip sequence or the current cube orientation
            break
    
    if c_bot != n_bot:
        for flip_sequence in flip_sequences:
            # check the current AND new bottom wrt the 6 possile Flip_sequences
            # returns a list with amount of flips on each Flip_sequences
            if (c_bot in flip_sequence) and (n_bot in flip_sequence):  # find the flip_sequence from current to new bottom
                results.append(flip_sequence[flip_sequence.find(c_bot):].find(n_bot)) # amount of flips from current to new bottom
            else:
                results.append(10) # returns 10 (too high value) if not the desired face on such Flip_sequence
        min_result = min(results)                             # min value is the preferred to minimize moves
        if min_result != 2:                                   # result is not 2 when not the opposite face
            n_flip_seq = flip_seq[results.index(min_result)]  # Flip_sequences with less flipping needed
        else:
            n_flip_seq = c_flip_seq                  # result =2 means opposite face, therefore better to don't Flip the cube
    else:
        n_flip_seq = c_flip_seq                  # result =2 means opposite face, therefore better to don't Flip the cube
    
    flips = n_flip_seq[n_flip_seq.find(c_bot):].find(n_bot) # returns the amount of flips from current to new bottom
    n_front = n_flip_seq[n_flip_seq.find(n_bot) +1]         # return wich face will be the next Front
    return flips, n_front 





def robot_moves(solution, solution_Text):
    """
    Solution reppresents the cube movements returned by the Kociemba solver, i.e. U2 F1 R3 etc
    solution_Text contains other type of information, related to the solution, like errors
    
    """
    robot={}                                                # empty dict to store all the robot moves
    robot_tot_moves = 0                                     # counter for all the robot movements
        
    if solution_Text != 'Error':                              # case the solver did not return an error
        moves = int(round(len(solution)/3,0))                 # total amount of blocks of movements
        
        for move in range(moves):                             # iteration over blocks of movements
            
            if move == 0:                                     # case first block of movement
                c_bottom = c_bottom_func(solution, move)      # current bottom face is returned, based on solution string
                c_front = c_front_func(solution, move)        # current front face is returned, based on solution string
            
            c_orient = c_orient_func(c_bottom, c_front)       # current bottom face, based on the "new" cube orientation
            n_bottom = n_bottom_func(solution)                # new bottom face, considering the next move from solution string
            c_n_bottom = c_n_bottom_func(c_bottom, n_bottom)  # current and new bottom face, based on the "new" cube orientation
            
            spin = spin_func(c_orient, c_n_bottom)            # cube spin or spins are returned
            flips, n_front = flips_func(c_bottom, n_bottom, c_orient, flip_seq) # cube flips, and new face in front, is returned

            rotations = int(solution[1:2])  # amount of rotations (for the block) are returned from the solution string
            if rotations == 3:              # case the solver suggested 3 turns             
                rotations = -1              # the notation is changed in "-1"

            robot[move+1] = 'S'+str(spin)+'F'+str(flips)+'R'+str(rotations)   # robot moves dict is populated
            c_bottom = n_bottom             # the new bottom face is now considered the current bottomone
            c_front = n_front               # the new front face is now considered the current front one
            
            solution = solution[3:]         # solution string is sliced, by removing the block of movement just "traslated"
            if spin != 0:                   # case there is/are cube spin needed
                spin = 1                    # multiple spins are done in a single robot action
            if rotations != 0:              # case there is/are cube rotation needed
                rotations = 1               # multiple rotations are done in a single robot action
            
            robot_tot_moves = robot_tot_moves + spin + flips + rotations # total amount of robot movements is updated
            
        #######
#         import time
#         time.sleep(150)
        #######

    return robot, robot_tot_moves    # returns a dict with all the robot moves (or empty one) and total robot movements




if __name__ == "__main__":
    
    print()
    print("Example of robot movements for solver solution: 'B2 R2 L1 U2 R3 B1 U3 R2 L3 U1 B1 D1 F2 L2 D3 L2 U1 F2 R2 L2'")
    print("Robot moves are notated with the 3 letters S, F, R (Spin, Flip, Rotate) followed by a number")
    print("Number '1' for S ans R identifies CW rotation, by loking to the bottom face, while number '3' stands for CCW")
    print("Example 'F1R1S3' means: 1x cube Flip, 1x (90deg) CW rotation of the 1st (bottom) layer, 1x (90deg) CCW cube Spin")
    print()
    
    solution = 'B2 R2 L1 U2 R3 B1 U3 R2 L3 U1 B1 D1 F2 L2 D3 L2 U1 F2 R2 L2'
#     solution = 'U2 D2 R2 L2 F2 B2'
    
    # 20230829 cube status and solution the robot does wrong (after removing the return to start position prior solving
    # on step:1 it does right
    # on step:2 it should do S1F1R2 and not S2F1R2 as the cube is oriented RB
    cube_state = 'URDFUUUBRBRLFRFLBBRUDLFUUFDFRFLDRFBLBLFDLUDBLBDRDBLUDR'
    solution = 'R1 U2 F2 U3 B3 U3 F3 L1 B1 U1 D1 R2 L3 D2 B2 R1 D2 L3 D2 L3'
    ##############

    solution_Text = ""
    robot, robot_tot_moves = robot_moves(solution, solution_Text)
    print(f'\nnumber of robot movements: {robot_tot_moves}')
    print()    
    print(f'robot movements: ')
    for step,moves in robot.items():
        print(f'step:{step}, robot moves:{moves}')


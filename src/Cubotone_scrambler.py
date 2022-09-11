import random
import Cubotone_servos as servo
import time

debug=False
def cube_scrambler(robot_moves='a'):
    """ drives the robot movable parts to scramble the cube;
        Argument is an dict with int starting from zero as key, and the SFL rotations.
        A random dict sequence can be generated with the rand_cube_moves() function
        
        Robot movements includes:
        Spin (= full cube rotation over the laying face, to change its orientation)
        Flip (= cube changing the face where it lays on)
        Rotate (= rotating the first layer on bottom with reference to the two layers above) """


    fix_moves={1:'S0F2R2', 2:'S0F1R1', 3:'S0F2R2', 4:'S-1F1R2', 5:'S0F1R-1', 6:'S-1F1R2', 7:'S2F1R-1', 8:'S2F1R2', 9:'S1F1R2',
               10:'S0F1R2', 11:'S2F1R-1', 12:'S0F2R1', 13:'S-1F1R-1', 14:'S-1F1R-1', 15:'S2F1R2', 16:'S0F1R1', 17:'S0F1R1',
               18:'S-1F1R2', 19:'S1F1R1', 20:'S2F1R2', 21:'S-1F1R0', 22:'S-1F1R0', 23:'S2F0R1'}

    if len(robot_moves)<=1:
        robot_moves=fix_moves

    servo.servo_start_positions(debug)
    time.sleep(0.5)
    
    start=time.time()
    i=1
    for move in robot_moves.values():                           # iterates over the dictionary values (the keys is the amount of Kociemba moves)
        print(f'Robot move {i}: {move}') 
        spins = int(move[move.find('S')+1:move.find('F')])      # amount (and direction) of spins is retrieved
        flips = int(move[move.find('F')+1:move.find('R')])      # amount of flips is retrieved (flips are only positive)
        rotations = int(move[move.find('R')+1:])                # amount (and direction) of rotations is retrieved
        
        if spins!=0:
            servo.spin(spins)                                   # servo package is called for the required spins
#             time.sleep(0.05)
        if flips!=0:
            servo.flip(flips, True, 'open')                     # servo package is called for the required flips
#             time.sleep(0.05)
        if rotations !=0:
            servo.rotate(rotations)                             # servo package is called for the required rotations (lower cube's layer)
#             time.sleep(0.05)
        i+=1
    
    servo.servo_start_positions()
    servo.motor_off()
    stop=time.time()
    print(f'Scramble time: {round(stop-start,1)} seconds')
    
        


def rand_cube_moves(n):
    """ Fuction to generate a dict with random sequence of robot moves
        Argument in an integer (<25) defining the number of random moves the function has to return """
    
    n=int(round(n,0))
    if n > 25:
        n=25
    
    moves={}
    for i in range(n):
        S='S'+str(random.randint(-1, 2))
        F='F'+str(random.randint(0, 3))
        R_options=[-1,1,2]
        R='R'+str(random.choice(R_options))
        moves[i]=S+F+R
        i+=1
    return moves



if __name__ == "__main__":

#     example_01, uncomment next two rows
#     moves=rand_cube_moves(4)              # generates a dict with 4 random moves
#     cube_scrambler(moves)                 # robot scrambles the cube by using the 4 random generated moves


#     example_02, uncomment the last row
#     This example uses a predefined scrambling sequence, called "fixed moves" in cube_scrambler function)
#     This approach is usefuluseful to check the robot overall speed with different set of parameters
#     Starting from a full solved cube, oriented on the robot as per Kociemba, it will end up to be:
#           Cube status:               RBDRUFDRLBRFBRLFBDBUUUFDFBUURLLDLRLBUDLDLFFDRLUBFBFRUD
#           Cube solution (20 moves):  U2 L1 R1 D2 B2 R1 D2 B2 D2 L3 B3 R3 F2 D3 L1 U2 F2 D3 B3 D1
    cube_scrambler()                      # robot scrambles the cube by using a predefined sequence of 23 moves

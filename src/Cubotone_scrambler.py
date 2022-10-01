import random
import Cubotone_servos as servo
import time

debug=False

def cube_scrambler(n):
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
    
    scramble = False
    
    if str(n).isdigit():
        if int(n) == 0:
            print("\nscrambling the cube with a predefined set of 23 movements")
            robot_moves=fix_moves
            scramble = True
        elif int(n) >= 1:
            print("\nscrambling the cube with",n,"random movements")
            robot_moves = rand_cube_moves(int(n))
            scramble = True
        else:
            scramble = False
    else:
        raise Exception("not an integer")
        
    if scramble:
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
                servo.spin(spins, debug)                            # servo package is called for the required spins
            if flips!=0:
                servo.flip(flips, debug, 'open', True)              # servo package is called for the required flips
            if rotations !=0:
                servo.rotate(rotations, debug)                      # servo package is called for the required rotations (lower cube's layer)
            i+=1
        
        servo.servo_start_positions(debug)
        servo.motor_off(debug)
        stop=time.time()
        print(f'Scrambling time: {round(stop-start,1)} seconds')
    
        


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
    """this script has an --moves n argument wherein n is an integer:
        If n is == 0 then the robot applies a fix sequence of 23 cube movements.
        If n >=1 then the robot applies n random cube movements."""
   
    import argparse
    parser = argparse.ArgumentParser(description='number of cube moves')    # argument parser object creation
    parser.add_argument("--moves", type=int, 
                        help="Set the number of cube random  moves, 0 for a predefined set of 23 movements)")      # argument is added to the parser
    args = parser.parse_args()                                              # argument parsed assignement

    if args.moves != None:                                     # case the Cubotino_T_servo.py has been launched with 'set' argument
        servo.init_servo(False, debug)                         # servos and motor settings are uploaded, servos and motor are initialized
        cube_scrambler(args.moves)                             # robot scrambles the cube by using a predefined sequence of 23 movesargs.moves
        
        while True:                                            # infinite loop, to give the chance to play with the servos angles
            target = input('\nenter moves quantity to re-scramble the cube (0 for pre-defined moves position, any letter to escape): ') # input request with proper info
            try:                                               # tentative 
                cube_scrambler(target)                         # robot scrambles the cube by using a predefined sequence of 23 movesargs.moves
            except:                                            # exception, in case non-numbers entered
                print('\nQuitting Cubotone_scrambler.py\n\n')  # feedback is printed to terminal
                break                                          # while loop is interrupted


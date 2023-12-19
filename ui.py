#
#   ui.py
#   
#   This includes the UI thread.
#
import threading
import ctypes
import robot as r
import traceback
import pickle
import ros

def update_pos(bot):
    if bot.pos == None:
        pass
    else:
        ros.x = bot.pos[0]
        ros.y = bot.pos[1]
        ros.heading = bot.heading

def ui():
    userInput = input('Type options start, reset, full reset,  explore, pause, goal, stepping, step, save, show, clear, or quit: ').lower()
    print()
    while userInput not in ['start', 'reset', 'full reset', 'explore', 'pause', 'goal', 'stepping', 'step', 'save', 'show', 'clear', 'quit']:
        print('Illegal command ' + userInput)
        userInput = input('Type options start, reset, full reset, explore, pause, goal, stepping, step, save, show, clear, or quit: ').lower()
        print()
    return userInput

def set_goal(xcoord, ycoord, bot):
    bot.set_mode('pause')
    x = int(xcoord)
    y = int(ycoord)
    bot.set_goal((x,y))
    bot.set_mode('goal')
    print("IN GOAL")

def ROS_explore(bot):
    bot.set_mode('explore')
    print("IN EXPLORE")

def robot_main():
    bot = r.Robot()
    print("Robot ready...")
    rosthread = threading.Thread(name="ROSThread", target=ros.runros, args=(bot,))
    rosthread.start()
    robot_thread = threading.Thread(name="RobotThread", target=bot.run)
    robot_thread.start()

    try:
        while True:
            print(f'start mode? {bot.start}')
            print(f'stepping {bot.stepping}')
            print(f'step {bot.step}')
            print(f'mode {bot.mode}')

            userInput = ui()

            if userInput == 'start':
                bot.start = True
                start_x = int(input('Input start x coordinate: '))
                start_y = int(input('Input start y coordinate: '))
                start_heading = int(input('Input start heading: '))
                bot.start_coord = (start_x, start_y)
                bot.start_heading = start_heading
                bot.set_mode('explore')

            elif userInput == 'explore':
                bot.set_mode('explore')

            elif userInput == 'goal':
                bot.set_mode('pause')
                x = int(input('Input the goal x coordinate: ')) 
                y = int(input('Input the goal y coordinate: '))
                set_goal(x, y, bot)

            elif userInput == 'pause':
                bot.set_mode('pause')

            elif userInput == 'stepping':
                bot.set_stepping()
            elif userInput == 'step':
                if bot.stepping and (bot.mode == 'explore' or bot.mode == 'goal'):
                    bot.set_step(True)

            elif userInput == 'save':
                print('Saving the map to thread_map.pickle')
                with open('thread_map.pickle', 'wb') as file:
                    pickle.dump(bot.map, file) 

            elif userInput == 'show':
                bot.set_mode('pause')
                bot.set_mode('show')
                bot.map.visualize_at_end(bot.pos)

            elif userInput == 'clear':
                for inter in bot.map.intersections.values():
                    inter.clear_obstacles()
                print(bot.map.map_info())
                print('Cleared obstacles!')
            
            elif userInput == 'reset':
                bot.reset(False)

            elif userInput == 'full reset':
                bot.reset(True)

            elif userInput == 'quit':
                bot.set_mode('quit')
                bot.drive.go('stop')
                break
    except BaseException as ex:
        print("Ending Run-UI due to exception: %s" % repr(ex))
        traceback.print_exc()

    ctypes.pythonapi.PyThreadState_SetAsyncExc(
        ctypes.c_long(robot_thread.ident),
        ctypes.py_object(KeyboardInterrupt))
    robot_thread.join()
    print('Robot thread joined')

    # End the ROS thread (send the KeyboardInterrupt exception).
    ctypes.pythonapi.PyThreadState_SetAsyncExc(
        ctypes.c_long(rosthread.ident), 
        ctypes.py_object(KeyboardInterrupt))
    rosthread.join()
    print('ROS thread joined')

    bot.shutdown()
#    
#   Main
#
if __name__ == "__main__":
    robot_main()
    
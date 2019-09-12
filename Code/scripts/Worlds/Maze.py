import numpy as np
import math,time

class Maze(object):
    
    def __init__(self,dimensions = [20,20,2], move_probs = [0.6,0.2,0.1,0.3], python_n_moves = 150):
        # dimensions: [x,y,z]
        self.maze_def_dict = {"dimensions": [20,20,4], "move_probs": {"forward": move_probs[0],"turn_yaw": move_probs[1],"turn_pitch": move_probs[2], "backward": move_probs[3]}, "python_n_moves": python_n_moves}
        self.maze_state = np.zeros(tuple(self.maze_def_dict["dimensions"]))

        initial_pos = np.array(self.maze_def_dict["dimensions"])/2
        initial_pos = np.array([10,10,3])
        self.python_state = {"position":initial_pos, "orientation": np.array([1,0,0])}
        self.update_maze()
        # print(np.array(self.maze_state[:,:,0]))
        self.steps()

        return

    def steps(self):

        for i in range(self.maze_def_dict["python_n_moves"]):
            self.step()

            print(np.array(self.maze_state[:,:,0]))

        print("z = 0")
        print(np.array(self.maze_state[:,:,0]))
        print("z = 1")
        print(np.array(self.maze_state[:,:,1]))
        print("z = 2")
        print(np.array(self.maze_state[:,:,2]))
        print("z = 3")
        print(np.array(self.maze_state[:,:,3]))

        self.gen_boundary_matrix()
        # print(self.boundary_matrix)


    def step(self):
        
        enclosure_cond = False
        while not(enclosure_cond):
            candidate_orientation = self.select_move()
            candidate_position = self.apply_move(self.python_state["position"], candidate_orientation)

            enclosure_cond = self.check_enclosure(candidate_position,[1,1,1])

            time.sleep(0.01)

        self.python_state["position"],self.python_state["orientation"] = candidate_position, candidate_orientation
        
        self.update_maze()


    def select_move(self):

        random_number = np.random.rand()

        probs = self.maze_def_dict["move_probs"]
        if random_number < probs["forward"]:
            new_orientation = self.python_state["orientation"]

        elif random_number < (probs["forward"] + probs["turn_yaw"]):

            theta = np.radians(90*(1-2*(np.random.rand() < 0.5)))
            r = np.array(( (np.cos(theta), -np.sin(theta),0),(np.sin(theta),  np.cos(theta),0),(0,0,1.0) ))
            new_orientation = self.apply_rotation(self.python_state["orientation"],r)
        
        elif random_number < (probs["forward"] + probs["turn_yaw"] + probs["turn_pitch"]):

            theta = np.radians(90*(1-2*(np.random.rand() < 0.5)))
            r = np.array(( (np.cos(theta),0, -np.sin(theta)),(0,1.0,0),(np.sin(theta),0,  np.cos(theta)) ))
            new_orientation = self.apply_rotation(self.python_state["orientation"],r)

        else:

            new_orientation = -self.python_state["orientation"]

        return new_orientation


    def apply_rotation(self,v,r):
        return r.dot(v)


    def apply_move(self,position,orientation):

        new_position = position + orientation.astype('int8')

        return new_position


    def check_enclosure(self,position,shell = [0,0,0]):

        for ind,val in enumerate(position):

            if not((position[ind] >= shell[ind]) and (position[ind] < self.maze_def_dict["dimensions"][ind] - shell[ind])):

                return False

        return True


    def update_maze(self):

        pos = self.python_state["position"]
        self.maze_state[pos[0],pos[1],pos[2]] = 1

        for direction in [[1,0,0],[-1,0,0],[0,1,0],[0,-1,0],[0,0,1],[0,0,-1]]:

            bound_pos = pos + direction

            if self.check_enclosure(bound_pos):

                if self.maze_state[bound_pos[0],bound_pos[1],bound_pos[2]] == 0:

                    self.maze_state[bound_pos[0],bound_pos[1],bound_pos[2]] = 2

    def gen_boundary_matrix(self):

        self.boundary_matrix = self.maze_state == 2


# maze = Maze()
import numpy
import matplotlib
import matplotlib.pyplot as plt
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
user_ip = '192.168.32.1' #'127.0.0.1'

class robot():

    
    def __init__(self, frame_name, motor_names=[], client_id=0):  
        # If there is an existing connection
        if client_id:
                self.client_id = client_id
        else:
            self.client_id = self.open_connection()
            
        self.motors = self._get_handlers(motor_names) 
        
        # Robot frame
        self.frame =  self._get_handler(frame_name)
            
        
    def open_connection(self):
        sim.simxFinish(-1)  # just in case, close all opened connections
        self.client_id = sim.simxStart(user_ip, 19999, True, True, 5000, 5)  # Connect to CoppeliaSim 
        
        if clientID != -1:
            print('Robot connected')
        else:
            print('Connection failed')
        return clientID
        
    def close_connection(self):    
        sim.simxGetPingTime(self.client_id)  # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive.
        sim.simxFinish(self.client_id)  # Now close the connection to CoppeliaSim:
        print('Connection closed')
    
    def isConnected(self):
        c,result = sim.simxGetPingTime(self.client_id)
        # Return true if the robot is connected
        return result > 0         
        
    def _get_handler(self, name):
        err_code, handler = sim.simxGetObjectHandle(self.client_id, name, sim.simx_opmode_blocking)
        return handler
    
    def _get_handlers(self, names):
        handlers = []
        for name in names:
            handler = self._get_handler(name)
            handlers.append(handler)
        
        return handlers

    def send_motor_velocities(self, vels):
        for motor, vel in zip(self.motors, vels):
            err_code = sim.simxSetJointTargetVelocity(self.client_id, 
                                                      motor, vel, sim.simx_opmode_streaming)      
            
    def set_position(self, position, relative_object=-1):
        if relative_object != -1:
            relative_object = self._get_handler(relative_object)        
        sim.simxSetObjectPosition(clientID, self.frame, relative_object, position, sim.simx_opmode_oneshot)
        
    def simtime(self):
        return sim.simxGetLastCmdTime(self.client_id)
    
    def get_position(self, relative_object=-1):
        # Get position relative to an object, -1 for global frame
        if relative_object != -1:
            relative_object = self._get_handler(relative_object)
        res, position = sim.simxGetObjectPosition(self.client_id, self.frame, relative_object, sim.simx_opmode_blocking)        
        return array(position)
    
    def get_object_position(self, object_name):
        # Get Object position in the world frame
        err_code, object_h = sim.simxGetObjectHandle(self.client_id, object_name, sim.simx_opmode_blocking)
        res, position = sim.simxGetObjectPosition(self.client_id, object_h, -1, sim.simx_opmode_blocking)
        return array(position)
    
    def get_object_relative_position(self, object_name):        
        # Get Object position in the robot frame
        err_code, object_h = sim.simxGetObjectHandle(self.client_id, object_name, sim.simx_opmode_blocking)
        res, position = sim.simxGetObjectPosition(self.client_id, object_h, self.frame, sim.simx_opmode_blocking)
        return array(position)

# Plot the grid
def draw_grid(grid_cells):
    n, m = 20,20
    fig, ax = plt.subplots()
    cmap = matplotlib.colors.ListedColormap(['0.9', 'black']) # Colors (0.9 is the almost white in gray scale)
    ax.imshow(grid_cells, cmap=cmap, origin='lower')
    ax.grid(which='major', axis='both', linestyle='-', color='k', linewidth=2)
    ax.set_xticks(numpy.arange(-0.5, m, 1))
    ax.set_yticks(numpy.arange(-0.5, n, 1))

def point_to_point_traj(x1, x2, v1, v2, delta_t):
    t = numpy.linspace(0, delta_t, 100)  
    a0 = x1
    a1 = v1
    a2 = (3*x2 - 3*x1 - 2*v1*delta_t - v2 * delta_t) / (delta_t**2)
    a3 = (2*x1 + (v1 + v2) * delta_t  - 2 * x2) / (delta_t**3)

    polynomial = a0 + a1 * t + a2 * t**2 + a3 * t**3
    derivative = a1 + 2*a2 * t + 3 * a3 * t**2
    #print('a0: (%.4f) '%(a0))
    #print('a1: (%.4f) '%(a1))
    #print('a2: (%.4f) '%(a2))
    #print('a3: (%.4f) '%(a3))
    return polynomial, derivative

def a_pairs(x1, x2, v1, v2, delta_t):
    t = numpy.linspace(0, delta_t, 100)  
    a0 = x1
    a1 = v1
    a2 = (3*x2 - 3*x1 - 2*v1*delta_t - v2 * delta_t) / (delta_t**2)
    a3 = (2*x1 + (v1 + v2) * delta_t  - 2 * x2) / (delta_t**3)

    #print('a0: (%.4f) '%(a0))
    #print('a1: (%.4f) '%(a1))
    #print('a2: (%.4f) '%(a2))
    #print('a3: (%.4f) '%(a3))
    return a0, a1, a2, a3

def piecewise3D (X,Y,Z, Vx, Vy, Vz, T):
    theta_x, theta_y, theta_z, dx, dy, dz = [], [], [], [], [], []
    a0_pairs = []
    a1_pairs = []
    a2_pairs = []
    a3_pairs = []

    
    count = 1
    for i in range(len(X)-1):
       # print(count)
        count = count+1
       # print('x')
        theta_xi, dxi = point_to_point_traj(X[i], X[i+1], Vx[i], Vx[i+1], T[i+1] - T[i])
        xa0, xa1, xa2, xa3 = a_pairs(X[i], X[i+1], Vx[i], Vx[i+1], T[i+1] - T[i])
       # print('y')
        theta_yi, dyi = point_to_point_traj(Y[i], Y[i+1], Vy[i], Vy[i+1], T[i+1] - T[i])
        ya0, ya1, ya2, ya3 = a_pairs(Y[i], Y[i+1], Vy[i], Vy[i+1], T[i+1] - T[i])
       # print('z')
        theta_zi, dzi = point_to_point_traj(Z[i], Z[i+1], Vz[i], Vz[i+1], T[i+1] - T[i])
        za0, za1, za2, za3 = a_pairs(Z[i], Z[i+1], Vz[i], Vz[i+1], T[i+1] - T[i])
       # print('=================')
        
        a0_pairs.append([xa0, ya0, za0])
        a1_pairs.append([xa1, ya1, za1])
        a2_pairs.append([xa2, ya2, za2])
        a3_pairs.append([xa3, ya3, za3])
        
        theta_x += theta_xi.tolist()
        theta_y += theta_yi.tolist()
        theta_z += theta_zi.tolist()
        dx += dxi.tolist()
        dy += dyi.tolist()
        dz += dzi.tolist()

        #plot(theta_xi, theta_yi)
    return theta_x, theta_y, theta_z, dx, dy, dz, a0_pairs, a1_pairs, a2_pairs, a3_pairs

# Plotting
def plot_points():
    plot(X,Y, '--')
    plot(X,Y, 'o')
    quiver(X,Y, Vx, Vy, color='r')

def plot_points_3d():
    plot(X,Y,Z, '--')
    plot(X,Y,Z, 'o')
    
# Speed
def plot_speed():
    speed = numpy.sqrt(numpy.array(dx)**2 + numpy.array(dy)**2)
    plot(speed)

def get_pairs_for_traj(startn,endn,ret_locs):
    n, m = 20,20  # number of rows and columns respectively.
    # to convert to coppeliasim, the origin of the graph needs to be moved up and over 10 by 10
    # Create a matrix to represent the cells of the grid
    grid_cells = numpy.ones((n,m))
    for loc in ret_locs:
        #print(int(loc[0]+10))
        grid_cells[int(loc[0]+10)][int(loc[1]+10)] = 0

    grid = Grid(matrix=grid_cells)
    #Get robot current position
    #start = grid.node(3, 1)
    print(f"Start: {startn}, Goal: {endn}")
    startx = round(startn[1]+10)
    starty = round(startn[0]+10)
    endx = round(endn[1]+10)
    endy = round(endn[0]+10)
    startx = startx if startx<19 else 19
    starty = starty if starty<19 else 19   
    endx = endx if endx<19 else 19
    endy = endy if endy<19 else 19

    start = grid.node(startx,starty)
    #Get robot end position
    #end = grid.node(17, 18)
    end = grid.node(endx,endy)
    #finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
    finder = AStarFinder(diagonal_movement=DiagonalMovement.never)
    path, runs = finder.find_path(start, end, grid)
    
    c_path = []
    for coord in path:
        x = coord[1]-9.5
        z = coord[0]-9.5
        y = .5 #basic height
        c_path.append((x,z,y))
    print(f"Path: {path}")

    ##Making it pretty
    grid_n = numpy.zeros((n,m))
    for x in range(0,20):
        for y in range(0,20):
            if grid_cells[x][y] == 0:
                grid_n[x][y] = 1
            else:
                grid_n[x][y] = 0
    pathp = []
    for nodelet in path:
        pathp.append([nodelet[1], nodelet[0]])
    draw_grid(grid_n)
    printp = numpy.array(path)
    plt.plot(printp[:,0], printp[:,1],'bo')
    plt.show()


    # Velocities
    vs = [[0,0,0]]
    for c_index in range(0,len(c_path)-1):
        rn = c_path[c_index]
        fut = c_path[c_index+1]
        xs = fut[0]-rn[0]
        zs = fut[1]-rn[1]
        if xs != 0:
            xs = xs/8
        if zs != 0:
            zs = zs/8
        vs.append([xs,zs,0])
    #print(vs)

    # Time
    ts = []
    for tim in range(0,len(c_path)):
        ts.append(tim*20)

    # Grouping
    #P = numpy.vstack((p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14, p15, p16, p17))
    #V = numpy.vstack((v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17))
    #T = [t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12, t13, t14, t15, t16, t17]
    P = numpy.vstack(c_path)
    V = numpy.vstack(vs)
    T = ts
    X, Y, Z = P[:,0], P[:,1], P[:,2]
    Vx, Vy, Vz = V[:,0], V[:,1], V[:,2]

    # Plotting
    #plot_points()
    #show()

    # Piecewise function
    theta_x, theta_y, theta_z, dx, dy, dz, a0_pairs, a1_pairs, a2_pairs, a3_pairs = piecewise3D(X,Y,Z, Vx,Vy,Vz, T)
    #if no move return 0s
    if(len(a0_pairs) == 0):
        no_move = [0,0,0]
        for i in range(0,10):
            a0_pairs.append(no_move)
        return a0_pairs, a0_pairs, a0_pairs, a0_pairs
    
    to_ap0 = a0_pairs[len(a0_pairs)-1]
    to_ap1 = a1_pairs[len(a1_pairs)-1]
    to_ap2 = a2_pairs[len(a2_pairs)-1]
    to_ap3 = a3_pairs[len(a3_pairs)-1]
    while(len(a0_pairs) < 10):
        a0_pairs.append(to_ap0)
        a1_pairs.append(to_ap1)
        a2_pairs.append(to_ap2)
        a3_pairs.append(to_ap3)

    return a0_pairs, a1_pairs, a2_pairs, a3_pairs
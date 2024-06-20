"PLOTS A SURFACE PLOT FOR A 3 ST-SR-IA PROBLEM. wOULDN'T PLOT IT FOR MORE THAN 3 EVERY ROBOT IS A DIMENSION"

import sys
import ast
import copy
import random
import parsers

import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
import matplotlib.pyplot as plt
import numpy as np


##*****************
#This is exactly what sir wanted. The meshgrid part was really tricky and took me a few days 15 maybe to ffigure it out
#plots the solution in a 3d surface plot.


#these two are important from where the file is read. the job_count would be a folder inside the Prnt_trnm... folder
FileString = "/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/"
job_count = "50"
const_value = 40
speed_matrix = parsers.read_speed()

#Inside this the two files would be dist_matrix.txt and solution.txt


def main(job_count, const_value, FileString):
    robot_num = 3       # important to change if the number of robots change as this helps in finding out the dist.
    dist_matrix = parsers.read_distmatrix(FileString)
    solution,robot_dist = parsers.read_solution()
    # robot_dist = solution[-robot_num:]
    # solution = solution[:-robot_num]

    print solution
    print dist_matrix
    print robot_dist

    # this is to check if the fitness function is working same as the deap__inti one. the optimal values should be same
    fitness = hetro_fitness_max(solution,dist_matrix,robot_dist,robot_num)
    print fitness

    # The part below this is performing two things.
    # 1. Keeping one robot dist constant and changing the other two and finding fitness one by one.
    # 2. Plotting these fitness distributions one by one. means 1 robot constant, change rest and plot the dist.



    mpl.rcParams['legend.fontsize'] = 10
    fig = plt.figure()
    fig.suptitle(job_count + ' Jobs, EA Best Solution', fontsize=14, fontweight='bold')
    ax = fig.gca(projection='3d')




    clone_robot_dist = copy.deepcopy(robot_dist)      # intelligent indexing ;)
                                                    # const id = 0: 0+1 % 3 = 1 0+2 %3 = 2 addressing the left 2 ids
    data = [[] for i in range(3)]  # empty space for saving the data generated. [id1 value] [id2 value] [fitness]

    nx = ny= (int(job_count)-robot_num)+1
    x = [range(1,nx+1)]     # Since am starting from 1 so I am going till nx+1
    y = [range(1,ny+1)]

    x,y = np.meshgrid(x,y)
    z = copy.deepcopy(y.astype(float))      #This copy is to give the z structure similar to x&y, astype is for typecast
    for i in range(nx):
        for j in range(ny):
            clone_robot_dist[0] = x[i,j]
            clone_robot_dist[1] = y[i,j]        #3rd robot = total-(x+y)
            clone_robot_dist[2] = int(job_count) - (clone_robot_dist[0] + clone_robot_dist[1])
            z[i,j] = function(solution, dist_matrix, clone_robot_dist, robot_num)

    surf = ax.plot_surface(x, y, z, rstride=1, cstride=1, cmap=cm.jet, linewidth=0, antialiased=False)
    ax.set_xlabel('Jobs done by one Robot')
    ax.set_ylabel('Jobs done by another Robot')
    ax.set_zlabel('Fitness')
    plt.title('Original Solution' + str(solution) + str(robot_dist), fontsize=10)
    ax.legend()
    mng = plt.get_current_fig_manager()
    mng.resize(*mng.window.maxsize())
    plt.show()
    fig.savefig(FileString + 'Surface.png')

#this function populates the z matrix against the meshgrid x,y
def function(solution, dist_matrix, clone_robot_dist, robot_num):
    if (clone_robot_dist[0]+clone_robot_dist[1] > ((int(job_count)-robot_num)+1)):
        return const_value          #Just returning a random value for non-existant combinations
    else:
        return hetro_fitness_max(solution, dist_matrix, clone_robot_dist, robot_num)



#This fitness function is a copy from the deap__init one with only a few changes. The robot assign in that one is within chromo
# where as the robot assign in this one is fed seperately to the function. Also the chromo over here is a preprocessed one with only job ids not the ,1 part i.e. only 9 not 9,1
# if there are problems with this one you can copy paste the 3D_plot one as it is, it was working.
def fitness_costmtrx(chromo, cst_mtrx, robots_assign, rbt_count):
    start_matrix = cst_mtrx[0][:]
    # print cst_mtrx
    rbt_job = [[] for i in range(rbt_count)]  # initializing a list of list for the individual robot jobs for each robot
    start = 0
    end = 0
    for i in range(rbt_count):
        end += robots_assign[i]
        rbt_job[i] = chromo[start:end]
        start = end
    start_x, start_y = '0', '0'             #starting location for all the robots. Might have to change this later on
    fitness = 0
    jobz = []
    for i in range(len(rbt_job)):        #for each and every robot
        jobz = rbt_job[i]
        for j in range(len(jobz)):  # for every single job of robot 'i'
            if j == 0:                      #if its the first job then start from base and go till first location
                fitness += start_matrix[jobz[j]] #coz I have appended this extra zero for home-home traveling  +1
            else:
                fitness += cst_mtrx[(jobz[j-1])][(jobz[j])] #+1 +1
        # fitness += start_matrix[jobz[j]]        #This is for completgin the tour. from home to home  +1
        fitness += start_matrix[jobz[-1]]  # adding this line in August 2017 to complete a round tour, so just adding the length of going from last job to home location, coz my CPLEX was performing poorer than my EA
                                            # taking advantage of the fact that this is a symetric problem, distance from 1-3 is same as distance from 3-1 so just adding the value from the start matrix.
                                            # WHEN ATTEMPTING FOR MORE THAN 3 ROBOTS THIS LINE WAS GIVING ERROR.
        jobz = []
    return fitness

def hetro_fitness(chromo, cst_mtrx, robots_assign, rbt_count):
    start_matrix = cst_mtrx[0][:]
    # print cst_mtrx
    rbt_job = [[] for i in range(rbt_count)]  # initializing a list of list for the individual robot jobs for each robot
    start = 0
    end = 0
    for i in range(rbt_count):
        end += robots_assign[i]
        rbt_job[i] = chromo[start:end]
        start = end
    start_x, start_y = '0', '0'             #starting location for all the robots. Might have to change this later on
    fitness = 0
    jobz = []
    for i in range(len(rbt_job)):        #for each and every robot
        jobz = rbt_job[i]                # isolate the jobs for that robot
        for j in range(len(jobz)):      # for every single job of robot 'i'
            if j == 0:                      #if its the first job then start from base and go till first location
                fitness += start_matrix[jobz[j]]/speed_matrix[i] #coz I have appended this extra zero for home-home traveling  +1
            else:
                fitness += cst_mtrx[(jobz[j-1])][(jobz[j])]/speed_matrix[i] #+1 +1
        # fitness += start_matrix[jobz[j]]        #This is for completgin the tour. from home to home  +1
        fitness += start_matrix[jobz[-1]]/speed_matrix[i]  # adding this line in August 2017 to complete a round tour, so just adding the length of going from last job to home location, coz my CPLEX was performing poorer than my EA
                                            # taking advantage of the fact that this is a symetric problem, distance from 1-3 is same as distance from 3-1 so just adding the value from the start matrix.
                                            # WHEN ATTEMPTING FOR MORE THAN 3 ROBOTS THIS LINE WAS GIVING ERROR.
        jobz = []
    return fitness

def hetro_fitness_max(chromo, cst_mtrx, robots_assign, rbt_count):
    start_matrix = cst_mtrx[0][:]
    # print cst_mtrx
    rbt_job = [[] for i in range(rbt_count)]  # initializing a list of list for the individual robot jobs for each robot
    start = 0
    end = 0
    for i in range(rbt_count):
        end += robots_assign[i]
        rbt_job[i] = chromo[start:end]
        start = end
    start_x, start_y = '0', '0'             #starting location for all the robots. Might have to change this later on
    fitness = 0
    fitness_list = []
    jobz = []
    for i in range(len(rbt_job)):        #for each and every robot
        jobz = rbt_job[i]                # isolate the jobs for that robot
        for j in range(len(jobz)):      # for every single job of robot 'i'
            if j == 0:                      #if its the first job then start from base and go till first location
                fitness += start_matrix[jobz[j]]/speed_matrix[i] #coz I have appended this extra zero for home-home traveling  +1
            else:
                fitness += cst_mtrx[(jobz[j-1])][(jobz[j])]/speed_matrix[i] #+1 +1
        # fitness += start_matrix[jobz[j]]        #This is for completgin the tour. from home to home  +1
        fitness += start_matrix[jobz[-1]]/speed_matrix[i]  # adding this line in August 2017 to complete a round tour, so just adding the length of going from last job to home location, coz my CPLEX was performing poorer than my EA
                                            # taking advantage of the fact that this is a symetric problem, distance from 1-3 is same as distance from 3-1 so just adding the value from the start matrix.
                                            # WHEN ATTEMPTING FOR MORE THAN 3 ROBOTS THIS LINE WAS GIVING ERROR.
        fitness_list.append(fitness)
        fitness = 0
        jobz = []
    return max(fitness_list)

if __name__ == "__main__":
    main()
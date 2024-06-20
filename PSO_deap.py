import sys
import getopt
import os
import time

import random
from deap import base, creator, tools, benchmarks
import matplotlib.pyplot as plt
import pickle
import cPickle
import Operators
import operator
import numpy
import gen_job
import copy
import parsers
import plot_TSP
import auction_traderbots
import collections

#Adding my routines made before switching to deap
sys.path.append('/home/usman/research_gaz_ros/src/turtlebot_research/src/charm/iterative_execute_ST-SR_MR-IA_TA')
# import Evolutionary.selection
import reader_classbased_dict
import fitness_dict

FileString = "/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/"
# at this particular location there should be a folder with the name of number of tasks e.g. '50'
# inside that folder there should be a folder with the number of iteration e.g. '0'
# inside that the whole process will take place so our destination would be ...../txt_files/ST_SR_TA/50/0/
# if you are running predeveloped jobs then keep them inside the folders according to the iterations 0, 1, 2, 3, 4, 5.

parameters = pickle.load(open("/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/parameters.pkl" , 'rb'))
print parameters
NGEN = parameters['NGEN']             # number of jobs x 80 for MR Homo
POP_SIZE = parameters['POP_SIZE']
map_dim = parameters['map_dim']   # basically its for the job generation, a 9 means jobs generated in space of -9 -- 9 in x and y both
robot_num = parameters['robot_num']
Jobs = parameters['jobs']
iteration = parameters['iteration']
job_type = parameters['jobs_type']
speed_matrix = parameters['speed_matrix']
# for large difference in speed of robots i observed that the solution was very skewed towards the faster robots.
# so much so that all the tasks were assigned to the faster one.

start_location = parameters['start_location']#[0, 0, 0, 0, 0, 0, 0]
# this is not being used right now but have added to allow robots to start from different location
# The information will be passed on to hetro_fitness_max_2 ruotine which allows robot to start form non depot locaitons
# The ending on depot can simply be commented out of the routine. A review of all other schemes
# i.e. Darrah, SSI, auction will be needed if this is implemented.

# Parameters Execution.py would pick from here
c1 = 2      # movement in favour of pbest
c2 = 2      # movement in favour of gbest


FileString = parameters['Complete_file_String']    #FileString + iteration_string
# Display input and output file name passed as the args
print ("PSO printing robots : %i Generation: %i and Population: %i Jobs: %i Iteration: %i" % (robot_num,NGEN, POP_SIZE, Jobs, iteration) )



# Display input and output file name passed as the args
print ("robots : %i Generation: %i and Population: %i Jobs: %i Iteration: %i" % (robot_num,NGEN, POP_SIZE, Jobs, iteration) )


#initializing a few empty list for graph construction at the end
maxm = list()
minm = list()
avge = list()
stde = list()
beste = list()

#pickling the logbook at the end, Have a look at how to view it, coz if you pickle it it wouldn't be normal txt.
logbook = tools.Logbook()
log_file = open(FileString + 'log_file.txt', 'a')

#************************************************* Reading job dictionary and all **************************************
# readinbg some older generated scheme and giving a kick start to the whole thing
job_dict = pickle.load(open(FileString + 'job_dict.pkl', 'rb'))
sub_job_dict = pickle.load(open(FileString + 'sub_job_dict.pkl', 'rb'))
dist_matrix = pickle.load(open(FileString + 'dist_matrix.pkl', 'rb'))
start_matrix = pickle.load(open(FileString + 'start_matrix.pkl', 'rb'))

IND_SIZE = len(sub_job_dict)                              #what should be the size of each individual in the generation
# **********************************************************************************************************************


#putting in the fitness and robots_assign attribute in to the individual
creator.create("FitnessMin", base.Fitness, weights=(-1.0,))
creator.create("Particle", list, fitness=creator.FitnessMin, speed=list, smin=None, smax=None, best=None)

def generate(ind_size, robot_number, pmin, pmax, smin, smax):
    # The chromosome strucutre being used has the location in chromosome as task ID, then the decimal number has the robot number and the task
    # execution order. The whole number part represents the robot to execute that task and the decimal part has the order of task execution.
    part = creator.Particle(random.uniform(pmin, pmax) for _ in range(ind_size))
    part.speed = [random.uniform(smin, smax) for _ in range(ind_size)]
    part.smin = smin
    part.smax = smax
    return part

def updateParticle(part, best, phi1, phi2):
    print 'this is the input particle', part
    u1 = (c1*random.uniform(0, phi1) for _ in range(len(part)))
    u2 = (c2*random.uniform(0, phi2) for _ in range(len(part)))
    v_u1 = map(operator.mul, u1, map(operator.sub, part.best, part))
    v_u2 = map(operator.mul, u2, map(operator.sub, best, part))
    part.speed = list(map(operator.add, part.speed, map(operator.add, v_u1, v_u2)))
    print 'these are the particle speeds received', part.speed
    speeds = part.speed
    for i, speed in enumerate(part.speed):
        if speed < part.smin:
            part.speed[i] = part.smin
        elif speed > part.smax:
            part.speed[i] = part.smax
    part[:] = list(map(operator.add, part, part.speed))
    for ind, x in enumerate(part):
        if x < 1:
            y = abs(x)%1
            part[ind] = abs(1+y)
        if x > robot_num:
            y = abs(x)%1
            part[ind] = abs(robot_num + y)
    print 'this is the updated particle', part


def geno2pheno(particle):
    permutation = []
    number_list = []
    job_list = []
    assignment = []
    for i in range(1, robot_num+1):     #goint through for each number, for robot 1 the number will be between 1.0 & 1.99
        count = 0
        for ind,x in enumerate(particle):
            if x >= i and x <= i+1:     # checking the range of the continuous number to figure out which robot attempts what task
                number_list.append(ind+1)   #indexing for the list starts from 0 but our job count starts from 1
                job_list.append(x)
                count += 1              # Keeping the total count of tasks each robot has to attempt
        assignment.append(count)        # count of tasks to be attempted by this robot
    permutation = [x for _,x in sorted(zip(job_list,number_list))]  # ordering the tasks based on the magnitude of numbers
    return permutation, assignment


def evaluate(particle):
    permutation, assignment = geno2pheno(particle)

    return hetro_fitness_max(permutation, assignment, dist_matrix, robot_num)


toolbox = base.Toolbox()
stats = tools.Statistics(key=lambda ind: ind.fitness.values)
toolbox.register("particle", generate, ind_size=IND_SIZE, robot_number=robot_num, pmin=1, pmax=robot_num+1, smin=-0.9, smax=0.9)
toolbox.register("population", tools.initRepeat, list, toolbox.particle)
toolbox.register("update", updateParticle, phi1=2.0, phi2=2.0)
toolbox.register("evaluate", evaluate)

#registering some stats for recording as the algo goes on
stats.register("avg", numpy.mean)
stats.register("std", numpy.std)
stats.register("min", numpy.min)
stats.register("max", numpy.max)


def main():
    # getting the solution file and the statistics file ready for writing on them later on
    solution_file = open(
        FileString + str(NGEN) + 'G_' + str(POP_SIZE) + 'P_' + str(Jobs) + 'J_' + str(robot_num) + 'R_' + str(
            iteration) + 'I_PSO_jobs_solutions.txt', 'a')
    solution_file.seek(0)
    solution_file.truncate()
    stats_file = open(
        FileString + str(NGEN) + 'G_' + str(POP_SIZE) + 'P_' + str(Jobs) + 'J_' + str(robot_num) + 'R_' + str(
            iteration) + 'I_PSO_jobs_stats.txt', 'a')
    stats_file.seek(0)
    stats_file.truncate()

    pop = toolbox.population(n=POP_SIZE)
    best = None

    for g in range(NGEN):
        for part in pop:
            fit = evaluate(part)
            part.fitness.values = fit,
            # print fit
            if not part.best or part.best.fitness < part.fitness:
                part.best = creator.Particle(part)
                part.best.fitness.values = part.fitness.values

            if not best or best.fitness < part.fitness:
                best = creator.Particle(part)
                best.fitness.values = part.fitness.values
        print g,'***************************best so far****************************:',best.fitness
        perm, assign = geno2pheno(best)
        solution_file.write((str(perm)) + (str(assign)) + str(best.fitness.values) + '\n')
        for part in pop:
            toolbox.update(part, best)



        # ****************** RECORDING STATISTICS ****************************
        record = stats.compile(pop)
        minm.append(record['min'])
        maxm.append(record['max'])
        avge.append(record['avg'])
        stde.append(record['std'])
        beste.append(best.fitness.values)
        stats_file.write(str(record) + str(best.fitness.values) + (str(g)) + '\n')
        # ********************************************************************

    permutation, assignment = geno2pheno(best)
    print permutation
    print assignment

    print evaluate(best)
    print best.fitness.values
    solution_file2 = open(FileString + 'solution_PSO.txt', "wb")
    solution_file2.write((str(permutation)) + "\n" + (str(assignment)) + "\n" + (str(best.fitness.values)) + "\n")
    solution_file2.close()
    print beste

    full_execution = open('/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/' + 'PSO_solution.txt', 'a')
    full_execution.write(str(best.fitness.values[0]) + '\n')

    # # ****************************** GRAPH PLOTTING ****************************
    # fig = plt.figure(figsize=(20, 20))
    # # x = list(range(0,NGEN))
    # x = list(range(0, generation_count))
    # plt.plot(x, minm, '-g', label='Best so Far(Minimum)')
    # # plt.plot(x,maxm, 'r', label='Maximum')
    # plt.plot(x, avge, 'b', label='Average')
    # plt.grid(True)
    # plt.legend(loc='upper left', fontsize=20)
    # plt.title("Graph for" + str(Jobs) + 'Jobs ' + str(robot_num) + 'robots', fontsize=40)
    # plt.ylabel('Fitness (minimum)', fontsize=30)
    # # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # # this is a huge jugaar, m 100% sure that there are better ways of adjusting the tick size and gridding.
    # # m just in a hurry to plot my graphs
    # plt.xticks(range(0, generation_count, 100), fontsize=20)
    # plt.yticks(range(int(min(avge))-10, int(max(avge))+10, 5), fontsize=20)
    # # plt.yticks(range(35, int(max(avge)), 5), fontsize=20)
    # # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # plt.xlabel('Generation', fontsize=30)



#  #*********************************** Time based Fitness functions  **************************************************
#very minor changes into the hetro_fitness function, now saving each robots total time into a list as fitness and returning
# the max out of it. This is coz sir asked me to treat the maximum time consumed by a robot (distance/speed) as my fitness
def hetro_fitness_max(chromo, robots_assign, cst_mtrx, rbt_count):
    start_matrix = cst_mtrx[0][:]
    rbt_job = [[] for i in range(rbt_count)]  # initializing a list of list for the individual robot jobs for each robot
    start = 0
    end = 0
    for i in range(rbt_count):          #robot count is plain number of robots
        end += robots_assign[i]
        rbt_job[i] = chromo.__getslice__(start,end)
        start = end

    fitness_list = []                       #a list of fitness values to return the max from
    fitness = 0

    for i in range(len(rbt_job)):        #for each and every robot
        for j in range(len(rbt_job[i])):  # for every single job of robot 'i'
            if j == 0:                      #if its the first job then start from base and go till first location
                fitness += start_matrix[rbt_job[i][j]]/speed_matrix[i] #coz I have appended this extra zero for home-home traveling  +1
            else:
                fitness += cst_mtrx[(rbt_job[i][j-1])][(rbt_job[i][j])]/speed_matrix[i] #+1 +1
        if len(rbt_job[i]) > 0:                                           # this is added coz without this the route to home was added for robots doing nothing (assign = 0), which was giving indexing errors
            fitness += start_matrix[rbt_job[i][j]]/speed_matrix[i]        #This is for completgin the tour. from home to home  +1
        fitness_list.append(fitness)                            #saving each robots fitness value into the list
        fitness = 0

    return max(fitness_list)                                    #returning the max
#  #********************************************************************************************************************


if __name__ == "__main__":
    main()
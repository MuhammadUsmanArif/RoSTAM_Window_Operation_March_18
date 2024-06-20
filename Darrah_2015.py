import sys
import getopt
import os
import time
import random
from deap import base, creator, tools
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
import Selection
import Seed_creation
import math
from collections import deque

#Adding my routines made before switching to deap
sys.path.append('/home/usman/research_gaz_ros/src/turtlebot_research/src/charm/iterative_execute_ST-SR_MR-IA_TA')

FileString = "/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/"
# at this particular location there should be a folder with the name of number of tasks e.g. '50'
# inside that folder there should be a folder with the number of iteration e.g. '0'
# inside that the whole process will take place so our destination would be ...../txt_files/ST_SR_TA/50/0/
# if you are running pre-developed jobs then keep them inside the folders according to the iterations 0, 1, 2, 3, 4, 5.

#Evolutioany parameters.
CXPB = 1.0              # 0.9 MR homo
MUTPB = 0.005           #MUTPB for gene based mutations is working with relatively lower values (0.005) compared to normal mutation (0.1)
NGEN = 5000             # number of jobs x 80 for MR Homo
POP_SIZE = 50
# Elitest_percentage = 0.05 # have turned the elitest down to 1 or 2 individuals in the generation and not any percentage
                        # for Homo MR 0.02 (the crossover was too greedy) giving error on job distributions such as 60 70, working with 50
                        # , hetroMR 0.4 the greedy crossover wasn't that effective.
                        # the crossover "carter" acts too greedy and the quick conversion rate of the generations is not
                        # coz of anything else but it is coz of the crossover operator.

# Other details for Team and Environment
map_dim = 9   # basically its for the job generation, a 9 means jobs generated in space of -9 -- 9 in x and y both
robot_num = 5
Jobs = 50
iteration = 3
job_type = 1

## Speeds
# speed_matrix = [1, 1, 1]
speed_matrix = [0.7, 0.68, 0.66]
# speed_matrix = [.7, .68, .66, .64, .62]
# speed_matrix = [1, 1, 1, 1, 1]
# for large difference in speed of robots i observed that the solution was very skewed towards the faster robots.
# so much so that all the tasks were assigned to the faster one.
#*********************************************************************************************************

## Input parameters for calling deap__init.py from someother file
## o == option
## a == argument passed to the o
## http://www.cyberciti.biz/faq/python-command-line-arguments-argv-example/  got the code below from here.
try:
    myopts, args = getopt.getopt(sys.argv[1:],"r:g:p:j:i:")
except getopt.GetoptError as e:
    print (str(e))
    print("Usage: %s -r 3 -g 100 -p 50" % sys.argv[0])
    sys.exit(2)

for o, a in myopts:
    if o == '-r':
        robot_num=int(a)
    elif o == '-g':
        NGEN=int(a)
    elif o == '-p':
        POP_SIZE = int(a)
    elif o == '-j':         #jobs are not implemented right now. will do this sometime else.
        Jobs = int(a)
    elif o == '-i':         # The i is basically for the iterative version implementation of things.
        iteration = int(a)

iteration_string = (str(Jobs) + "/" + str(iteration) + "/")
FileString = FileString + iteration_string

# Display input and output file name passed as the args
print ("robots : %i Generation: %i and Population: %i Jobs: %i Iteration: %i" % (robot_num,NGEN, POP_SIZE, Jobs, iteration) )


# **********************************************************************************************************************
# this is the portion you uncomment when reading some older generated scheme and giving a kick start to the whole thing
# the new task generation portion is non-existant over here coz that work is doen in RoSTAM (GA_solo.py part), Darrah
# since is always run as a follow up so no need for all those portions.

job_dict = pickle.load(open(FileString + 'job_dict.pkl', 'rb'))
sub_job_dict = pickle.load(open(FileString + 'sub_job_dict.pkl', 'rb'))
dist_matrix = pickle.load(open(FileString + 'dist_matrix.pkl', 'rb'))
start_matrix = pickle.load(open(FileString + 'start_matrix.pkl', 'rb'))

IND_SIZE = len(sub_job_dict)            #Size of each individual in the generation

# **********************************************************************************************************************

#initializing a few empty list for graph construction at the end
maxm = list()
minm = list()
avge = list()
stde = list()
off_avge = list()
off_min = list()

# initializing this FIFO queue to implement the panelty function in Michalewicz Paper panelty scheme 3.2.4
# used while calling the panelty_adaptive function in panelty.py
last_k_bests = deque(20*[1], 20)

#pickling the logbook at the end, Have a look at how to view it, coz if you pickle it it wouldn't be normal txt.
logbook = tools.Logbook()
log_file = open(FileString + 'log_file.txt', 'a')

#************************************ Declaring and initializing chromosomes in deap ***********************************
#putting in the fitness and robots_assign attribute in to the individual
creator.create("FitnessMin", base.Fitness, weights=(-1.0,))     # marking to deap that its a minimization problem with one fitness function
creator.create("Individual", list, fitness=creator.FitnessMin, robots_assign=None) # defining the specs of an individual (chromosome)

## initiating the individual (deap format)
def initIndividual(icls, ind_size, robot_number):
    ind = icls(random.sample(sub_job_dict.keys(), ind_size))    # random sampling from the task_id vlaues without replacement
    ind.robots_assign = [[None] for _ in xrange(ind_size)]  # defining robot_assign as a list of robot ids
    init_assignment_random(ind_size, ind, robot_num)  # initialize the robot assignments (2nd part of the chromo) for each chromosome
    return ind


def evaluate(individual):
    # Time based fitness evaluation
    #This one is working when I am using the distance matrix
    # I wouldn't be needing the dictionary coz I dont want x,y in cost matrix just the job index
    return hetro_fitness_max(individual, dist_matrix, robot_num) # for time based fitness function which supports hetro as well as homo team


#initiating the toolbox for registering deap procedures as needed
toolbox = base.Toolbox()    # initiating the toolbox and defining different routines under it for later use
stats = tools.Statistics(key=lambda ind: ind.fitness.values)    # for handling the statistics
#********************** Setting for random initializtion of population *************************************************
toolbox.register("individual", initIndividual, creator.Individual, ind_size= IND_SIZE, robot_number=robot_num)
toolbox.register("population", tools.initRepeat, list, toolbox.individual)          #registering how the population shoud look like. And it calls the generation of individuals inside it.
#********************** Setting for seeded initializtion of population *************************************************

#registering some stats for recording as the algo goes on
stats.register("avg", numpy.mean)
stats.register("std", numpy.std)
stats.register("min", numpy.min)
stats.register("max", numpy.max)


def main():
    start_time = time.time()
    # Starting a text file to save all the fitness values of a single run onto it. (for later record keeping)
    # for the longer runs from execute.py the RoSTAM_Results file below is used for keeping values of multiple runs and then averaged and dumped in the average recording file
    full_execution = open('/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/' + 'RoSTAM_Results.txt','a')
    full_valid_invalid = open('/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/' + 'invalid.txt','a')


    # *********************************** Population Initialization *******************************************
    #Doesn't apply the repair operator but applies the penalty function instead.
    pop, fitness = init_random_penalty()    #performing poorer than init_random_repair
    # assigning the fitness to corresponding individuals
    for ind, fit in zip(pop, fitness):
        ind.fitness.values = fit,

    #getting the solution file and the statistics file ready for writing on them later on
    solution_file = open(FileString + str(NGEN) + 'G_' + str(POP_SIZE) + 'P_' + str(Jobs)
                         + 'J_' + str(robot_num) + 'R_' + str(iteration) + 'I_jobs_solutions.txt', 'a')
    solution_file.seek(0)
    solution_file.truncate()
    stats_file = open(FileString + str(NGEN) + 'G_' + str(POP_SIZE) + 'P_' + str(Jobs)
                      + 'J_' + str(robot_num) + 'R_' + str(iteration) + 'I_jobs_stats.txt', 'a')
    stats_file.seek(0)
    stats_file.truncate()

    generation_count = 0

    #Running the EA generations
    for g in range(NGEN):
        #picking the best individual from the generation and writing it to the solution file
        best = max(pop, key=operator.attrgetter('fitness')) # This will be max, tried the min thing, it was returning the worst value

        ## This portion is for recording the FIFO list initiated at top for penalty_adaptive function
        if check_invalid_darrah(len(best),best,robot_num):
            print '***************best is invalid*******************'
            last_k_bests.appendleft(1)
        else:
            last_k_bests.appendleft(0)

        solution_file.write(str(best) + (str(best.robots_assign)) + (str(best.fitness)) +'\n' )
        offspring = copy.deepcopy(pop)

        MUTPB = 0.5 / len(best)         #this basically translates to a 50% chance of mutation in the whole chromosome

        for i in range(0, POP_SIZE, 2):
            parent1, parent2 = Selection.roulette_select_FPS_ind(pop, 2)
            # parent1, parent2 = Selection.roulette_select_RBS_ind(pop, 2)
            par1 = child1 = copy.deepcopy(parent1)
            par2 = child2 = copy.deepcopy(parent2)
            # print 'parent fitness:', parent1.fitness.values[0], parent2.fitness.values[0]


            ## *********************************** Crossovers **********************************************************
            if random.random() < CXPB:
                ##***************************** MY ALGO Crossovers *****************************************************
                child1, child2 = Operators.permutation_Carter_2006(par1, par2) #used this crossover for GA-II results
                # child1, child2 = Operators.TCX_yuan2013(par1, par2)
                # child1, child2 = Operators.permutation_crossover2(par1, par2)

                # **************************** Darrah's own crossovers *************************************************
                # if g % 2 == 0: # used this even odd thing for GA-I results in paper
                #     child1, child2 = crossover_darrah(par1, par2)
                # else:
                #     child1 = crossover_inversion_darrah(par1)
                #     child2 = crossover_inversion_darrah(par2)
            # print 'child after crossover:', child1, child2

            ## *********************************** Mutation ************************************************************
            # if random.random() < .1:
            #     child1 = Operators.shuffle_mutation(child1)
            # if random.random() < .1:
            #     child2 = Operators.shuffle_mutation(child2)
            # child1 = Operators.gene_swap_mut(child1, MUTPB)
            # child2 = Operators.gene_swap_mut(child2, MUTPB)
            # child1 = Operators.gene_inverse_mut(child1, MUTPB)
            # child2 = Operators.gene_inverse_mut(child2, MUTPB)
            # child1 = Operators.gene_swap_mut(child1, MUTPB)
            # child2 = Operators.gene_swap_mut(child2, MUTPB)
            # child1 = Operators.gene_inverse_mut_II(child1, MUTPB, g + 1)
            # child2 = Operators.gene_inverse_mut_II(child2, MUTPB, g + 1)

            ## inventing this heuristic coz the gene_inverse_mut_II tends to be a bit slow at the end of the show.
            if g < NGEN*.8:
                child1 = Operators.gene_inverse_mut_II(child1, MUTPB, g + 1)
                child2 = Operators.gene_inverse_mut_II(child2, MUTPB, g + 1)
            else:
                child1 = Operators.gene_swap_mut(child1, MUTPB)
                child2 = Operators.gene_swap_mut(child2, MUTPB)
            # **************************** Darrah's own mutation *************************************************
            # if random.random() < 0.5:
            #     child1 = mutation_darrah(child1)
            #     child2 = mutation_darrah(child2)

            ##**********************************************************************************************************


            ##************************************ Penalty functions ***************************************************
            # penalty1 = penalty_dynamic_darrah(len(child1), child1, robot_num, generation_count)
            # penalty2 = penalty_dynamic_darrah(len(child2), child2, robot_num, generation_count)

            ## This penalty_fix function is used in all the population initialization routines where ever the penalty is applicable.
            penalty1 = penalty_fix_darrah(len(child1), child1, robot_num, generation_count)
            penalty2 = penalty_fix_darrah(len(child2), child2, robot_num, generation_count)

            # penalty1 = penalty_adaptive_darrah(len(child1), child1, robot_num, generation_count, last_k_bests)
            # penalty2 = penalty_adaptive_darrah(len(child2), child2, robot_num, generation_count, last_k_bests)
            # print 'child penalties:', penalty1, penalty2

            ##************************************ Fitness evaluations and allocations *********************************
            fitness1 = evaluate(child1)
            fitness2 = evaluate(child2)
            # print 'fitness without penalty:', fitness1, fitness2
            child1.fitness.values = (fitness1 + penalty1),
            child2.fitness.values = (fitness2 + penalty2),
            # print 'fitness after penalty:', child1.fitness.values[0], child2.fitness.values[0]

            ## Putting them into the pool.
            offspring[i] = child1
            offspring[i+1] = child2         #if you get an index out of range error at this point its coz of odd numbered for loop not matching your allowed number of solutions

        #************************* SURVIVAL SELECTION *********************
        combined_pop = copy.deepcopy(pop+offspring)
        pop = Selection.truncation(combined_pop, int(1))#POP_SIZE * Elitest_percentage))
        pop += Selection.Binary_Tournament(combined_pop, int(POP_SIZE -1),2)# * (1-Elitest_percentage)), 2)
        # pop += Selection.roulette_select_FPS(combined_pop, int(POP_SIZE -1))#* (1-Elitest_percentage)))

        #****************** RECORDING STATISTICS ****************************

        #   Record keeping
        record = stats.compile(pop)
        minm.append(record['min'])
        maxm.append(record['max'])
        avge.append(record['avg'])
        stde.append(record['std'])
        print(record)
        stats_file.write(str(record) + (str(g)) + '\n')
        logbook.record(gen=0, **record)
        generation_count += 1
        print('generation count' , generation_count)


    #writing the last best value in the best value record.
    best = max(pop, key=operator.attrgetter('fitness'))
    solution_file.write((str(best)) + (str(best.robots_assign)) + (str(best.fitness)) + '\n')
    stats_file.close()
    solution_file.close()

    #writing the last best value in a seperate file as a single entry with individual robot solutions seperated.
    solution_file_ind = open(FileString + 'solution_individual.txt', "wb")

    #identifying each robot's tours.
    rbt_job = [[] for x in range(robot_num)]
    indices = []
    print rbt_job
    for p in range(robot_num):
        indices = [index for index, value in enumerate(best.robots_assign) if value == (p + 1)]
        for job in indices:
            rbt_job[p].append(best[int(job)])
    # printing and saving each robot's identifed tours.
    for robot in rbt_job:
        print robot
        for entry in robot:
            solution_file_ind.write(entry + '  '),
        solution_file_ind.write('\n')

    # Writing the solution in another file wihout isolating each robot's subtour.
    solution_file2 = open(FileString + 'solution.txt', "wb")
    solution_file2.write((str(best)) + "\n" + (str(best.robots_assign)) + "\n" + (str(best.fitness)) + "\n")
    solution_file2.close()
    solution_file_ind.close()
    speed_file = open(FileString + 'speed.txt', "wb")
    speed_file.write(str(speed_matrix))
    speed_file.close()

    best_fitness = str(best.fitness)
    best_fitness = best_fitness.replace(")", "")
    best_fitness = best_fitness.replace("(", "")
    best_fitness = best_fitness.replace(",", "")
    full_execution.write(best_fitness + '\n')  # making a single file for all the results
    if check_invalid_darrah(len(best), best, robot_num):
        full_valid_invalid.write('invalid'+'\n')
    else:
        full_valid_invalid.write('valid'+'\n')

    pickle.dump(logbook, log_file)
    #Only plotting the convergence graph for darrha. Not plotting the robot tour as it's very dependent on my chromosome structure.
    plot_TSP.graph_plot(minm, avge, off_avge, off_min, generation_count, Jobs, robot_num, NGEN, POP_SIZE, iteration) #ASF and BSF curves
    # plot_TSP.main(FileString, (map_dim *-1)-2, map_dim+2)   # plotting the route map for the final solution
    print (time.time() - start_time)

# 2nd part of the chromosome initialization with random robot numbers.
# remember the darrah representation is two chromosome representation so for every task in one chromosome
# there has to be a robot declared to attempt it in the 2nd chromosome
def init_assignment_random(IND_SIZE, IND, robot_num):
    for id in range(IND_SIZE):     # looping through each task in one chromosome
        value = random.randint(1,robot_num)        # pick a robot to attempt that task
        IND.robots_assign[id] = value               # allocating the selected robot (value) to that task in 2nd chromosome


#  #*********************************** Time based Fitness functions  **************************************************
#very minor changes into the hetro_fitness function, now saving each robots total time into a list as fitness and returning
# the max out of it. This is coz sir asked me to treat the maximum time consumed by a robot (distance/speed) as my fitness
def hetro_fitness_max(chromo, cst_mtrx, rbt_count):
    start_matrix = cst_mtrx[0][:]             # initializing the start matrix, which is just the first line of cost matrix
    rbt_job = [[] for x in range(rbt_count)]  # initializing a list of list for the individual robot jobs for each robot
    fitness_list = []  # a list of fitness values to return the max from the times of each robot
    fitness = 0
    previous_task = task = None

    #looping through each individual robot.
    for i in range(rbt_count):
        #isolating the ids in the solution for each individual robot
        indices = [index for index, value in enumerate(chromo.robots_assign) if value == (i+1)] #i+1 coz in solution the robot count starts from 1
        #picking the jobs against the ids for each robot identified above.
        for value in indices:       # populating subtours for each robot
            rbt_job[i].append(chromo[int(value)])

    # Now I have the individual tour lists of each robot. So its just like my routine fitness evaluation.
    for i in range(rbt_count):        #for each and every robot
        for j in range(len(rbt_job[i])):
            task_id = rbt_job[i][j].split(',')
            task = int(task_id[0])
            if j == 0:                      #if its the first job then start from base and go till first location
                fitness += start_matrix[task]/speed_matrix[i] #coz I have appended this extra zero for home-home traveling  +1
            else:
                fitness += cst_mtrx[previous_task][task]/speed_matrix[i] #+1 +1
            previous_task = task
        if len(rbt_job[i]) > 0:   # checking if this particular robot has no jobs then there is no point completing the tour
            fitness += start_matrix[task]/speed_matrix[i]        #This is for completing the tour. from home to home  +1
        fitness_list.append(fitness)                            #saving each robots fitness value into the list
        # for a normal distance based fitness just return the sum of this list with speed values set to 1 for all robots
        fitness = 0
    return max(fitness_list)                                    #returning the max
#  #********************************************************************************************************************

# This routine initializes the population of solutions while applying penalty on them for their invalidness in ST-MR-TA
def init_random_penalty():
    pop = toolbox.population(n=POP_SIZE)        # for random initialization of population
    total_penalty = [None] * POP_SIZE           # empty list of penalty values for each individual solution
    for i, ind in enumerate(pop):  # for every crossovered and mutated child check if its a valid solution or not
        total_penalty[i] = penalty_fix_darrah(len(ind), ind, robot_num,1)
    fitness = map(evaluate, pop)              #evaluate each individual of the population by mapping the evaluate function over the whole population
    return pop, [x + y for x, y in zip(total_penalty, fitness)] #merging the fitness and penalty values to return as the population and its respective fitness with penalty

def penalty_fix_darrah(ind_size, ind, rbt_count, generation_count):
    penalty = 0
    invalid = True              # don't think this flag is of any use for this routine.
    rbt_job = [[] for i in range(rbt_count)]  # initializing a list of list for the individual robot jobs for each robot
    ind_split = [i.split(",")[0] for i in ind]  # reading only the first part of the task ids in the chromosome, I ain't really concerned about the subtask while checking invalid

    for i in range(rbt_count):  #for each robot in the chromosome
        # isolating the ids in the solution for each individual robot
        indices = [index for index, value in enumerate(ind.robots_assign) if value == (i+1)] #if value in robot chromosome matches the robot we are looping over then read the task id (later used to read tasks from other chromosome)
        # picking the jobs against the ids for each robot identified above.
        for value in indices:
            rbt_job[i].append(ind_split[int(value)]) #reading the task-id for this particular robot. using the ind_split isolated above and the ids extracted within this for loop.

    #for each robots tour identified above count the number of job repetations happening and penalize against that
    for robot in rbt_job:
        already_counted = []
        for job in robot:
            count = robot.count(job)
            if count > 1 and (job not in already_counted):  #if a task is already counted within this robot for violations then shouldn't be counted again.
                penalty += count * 5        #number of violations *5 the fixed penalty
            already_counted.append(job)
    return penalty

# This is an adaptive penalty implementation for Darrah
# Implementation of the Michalewicz Paper panelty scheme 3.2.4
# For extensive commenting for majority of stuff in here read the fixed penalty routine. AS this is almost same as that till extracting the count.
def penalty_adaptive_darrah(ind_size, ind, rbt_count, generation_count, last_K_results):
    penalty = total_count = 0
    invalid = True  # Simple flag used all along to check where the loops are returning from
    rbt_job = [[] for i in range(rbt_count)]  # initializing a list of list for the individual robot jobs for each robot
    ind_split = [i.split(",")[0] for i in ind]

    for i in range(rbt_count):
        # isolating the ids in the solution for each individual robot
        indices = [index for index, value in enumerate(ind.robots_assign) if value == (i + 1)]
        # picking the jobs against the ids for each robot identified above.
        for value in indices:
            rbt_job[i].append(ind_split[int(value)])
    # for each robots tour identified above count the number of job repetations happening and store the count for that
    for robot in rbt_job:
        already_counted = []
        for job in robot:
            count = robot.count(job)
            if count > 1 and (job not in already_counted):
                total_count += count
            already_counted.append(job)
    # use the count for the adaptive penalty evaluation.
    if 0 in last_K_results:
        if 1 in last_K_results:
            lamda = 5
        else:
            lamda = 1
    else:
        lamda = 20

    penalty = lamda * (total_count)
    # print lamda
    return penalty

# This is an dynamic penalty implementation for Darrah
# Implementation of the Michalewicz Paper panelty scheme 3.2.2
# For extensive commenting for majority of stuff in here read the fixed penalty routine. AS this is almost same as that till extracting the count.
def penalty_dynamic_darrah(ind_size, ind, rbt_count, generation_count):
    penalty = total_count = 0
    invalid = True  # Simple flag used all along to check where the loops are returning from
    rbt_job = [[] for i in range(rbt_count)]  # initializing a list of list for the individual robot jobs for each robot
    ind_split = [i.split(",")[0] for i in ind]

    for i in range(rbt_count):
        # isolating the ids in the solution for each individual robot
        indices = [index for index, value in enumerate(ind.robots_assign) if value == (i + 1)]
        # picking the jobs against the ids for each robot identified above.
        for value in indices:
            rbt_job[i].append(ind_split[int(value)])
    # for each robots tour identified above count the number of job repetations happening and store the count for that
    for robot in rbt_job:
        already_counted = []
        for job in robot:
            count = robot.count(job)
            if count > 1 and (job not in already_counted):
                total_count += count
            already_counted.append(job)
    # use the count for the adaptive penalty evaluation.
    penalty = pow(0.3 * generation_count, 0.4) * pow(total_count,1)
    # print penalty
    return penalty

# this routine simply checks if a chromosome is invalid or not. return a valid/invalid flag after the check
# many of the things in this routine are same as the fix penalty. So read that for understading the working
def check_invalid_darrah(ind_size, ind, rbt_count):
    invalid = False  # Simple flag used all along to check where the loops are returning from
    rbt_job = [[] for i in range(rbt_count)]  # initializing a list of list for the individual robot jobs for each robot
    ind_split = [i.split(",")[0] for i in ind]

    for i in range(rbt_count):
        indices = [index for index, value in enumerate(ind.robots_assign) if value == (i + 1)]
        for value in indices:
            rbt_job[i].append(ind_split[int(value)])

    for robot in rbt_job:
        if invalid:
            break
        already_counted = []
        for job in robot:
            count = robot.count(job)
            if count > 1 and (job not in already_counted):
                invalid = True
                break
    return invalid

# This is crossover from Darrah_Willhelm_2015 for the vehicles
# Its a single point crossover, select a random point. left of that point parent one. right of that point from parent 2
# this top layer combined with the as-it-is 2nd layer(2nd chromosome) of both parents produces two offsprings
def crossover_darrah(cl1, cl2):
    temp1 = []
    CX_Point = random.randint(0,len(cl1))
    for_validation1 = cl1.robots_assign
    for_validation2 = cl2.robots_assign
    temp1[:CX_Point] = cl1.robots_assign[:CX_Point]
    temp1[CX_Point:] = cl2.robots_assign[CX_Point:]
    # replacing the altered in both parents. Not altering the chromosome with the tasks at all.
    cl1.robots_assign = temp1
    cl2.robots_assign = temp1
    return cl1, cl2

# Simple inversion crossover. Select a substring from the chromosome, invert it and put it back.
def crossover_inversion_darrah(cl1):
    length = len(cl1)
    id1, id2 = random.sample(xrange(length),2)  # x range generates a list of numbers till the given range and then there are two random samples picked from that
    if id1 > id2:
        id1, id2 = id2, id1  # making sure that id1 is smaller than id2
    subarray = cl1[id1:id2]  # plugging out the substring
    subarray.reverse()  # reversing it
    cl1[id1:id2] = subarray
    return cl1

def mutation_darrah(cl1):
    index = random.randint(0,len(cl1.robots_assign)-1)
    robot_id = random.randint(1,robot_num)
    cl1.robots_assign[index] = robot_id
    return cl1

if __name__ == "__main__":
    main()
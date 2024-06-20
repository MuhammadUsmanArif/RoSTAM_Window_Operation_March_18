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

#Evolutioany parameters.
CXPB = 1
MUTPB = 0.005           #MUTPB for gene based mutations is working with relatively lower values (0.005) compared to normal mutation (0.1)
NGEN = 1000
POP_SIZE = 10
Elitest_percentage = 0.4 # for Homo MR 0.05 (the crossover was too greedy), hetroMR 0.4 the greedy crossover wasn't that effective.
                         # the crossover "carter" acts too greedy and the quick conversion rate of the generations is not
                         # coz of anything else but it is coz of the crossover operator.

# Other details for Team and Environment
map_dim = 9   # basically its for the job generation, a 9 means jobs generated in space of -9 -- 9 in x and y both
robot_num = 5
Jobs = 10
iteration = 0
job_type = 4
# speed_matrix = [1, 1, 1]
# speed_matrix = [0.7, 0.68, 0.66]
# speed_matrix = [.7, .68, .66, .64, .62]
speed_matrix = [1, 1, 1, 1, 1]
# for large difference in speed of robots i observed that the solution was very skewed towards the faster robots.
# so much so that all the tasks were assigned to the faster one.
#*********************************************************************************************************
# Input parameters for calling deap__init.py from someother file
# o == option
# a == argument passed to the o
# http://www.cyberciti.biz/faq/python-command-line-arguments-argv-example/  got the code below from here.
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


# Generate new jobs for every iteration. I don't think am using this call anymore. I use the call in the portion below
# os.system("python /home/usman/research_gaz_ros/src/turtlebot_research/src/gen_job.py -j " + str(Jobs) + " ")          #this was the older way of generating jobs till ST-SR-IA and ST-MR-IA. for TA I have put the gen_job routine inside the project

#*****************************Generating jobs and starting gazebo if this iteration is the 0th iteration****************
##This generate job line is to be commented when reading some old generated jobs. Otherwise it stays on
# gen_job.main(Jobs, job_type, FileString, map_dim*-1, map_dim) #calling the gen_job routine residing inside the project. the older versions had a general gen_job file

##This portion of code goes to start all the ROS and gazebo nodes for the first iteration of generating jobs
## For the iterative version The gazebo and ros nodes are started only on first iteration and then are not terminated
## For all the coming iterations they stay on. Don't forget to remove the kill commands at the end of the main routine
# if (iteration == 0): # this condition is important when i run the code through execute coz then this portion would run every time if I don't put the condidtion
#     os.system("roslaunch turtlebot_research mylaunch_gazebo.launch world_file:=/opt/ros/indigo/share/turtlebot_gazebo/worlds/empty.world &")
#     time.sleep(20)
#     for i in range(1, robot_num+1): # starting the indexing from 1 and taking it to robot#+1 coz these number will be used as names in robot launches, don't want a robot0
#         os.system("roslaunch turtlebot_research mylaunch_turtlebot_0.2_multiplebots.launch robot_name:='robot" +str(i)+"' robot_frame:='robot" +str(i)+ "_tf' robot_loc:='-z 3 -x 3 -y 0.0' frame_publisher:='3 0 0 0 0 0 map robot" + str(i) + "_tf/odom 100' &")
#     time.sleep(20)
#     print ('mylaunch file launched & jobs generated')
#     os.system("roslaunch turtlebot_research multiple_nav_mapONLY.launch map_file:=/home/usman/research_gaz_ros/src/turtlebot_research/navigation/empty.yaml &")
#     for i in range(1, robot_num+1):
#         os.system("roslaunch turtlebot_research multiple_nav_rbtONLY.launch  robot_name:='robot" +str(i)+"' robot_frame:='robot" +str(i)+ "_tf' robot_x:='0' robot_y:='0' &")
#     print ('navigation with empty map also launched')
#     time.sleep(20)
#************************************************************************************************************************

#initializing a few empty list for graph construction at the end
maxm = list()
minm = list()
avge = list()
stde = list()
off_avge = list()
off_min = list()

#pickling the logbook at the end, Have a look at how to view it, coz if you pickle it it wouldn't be normal txt.
logbook = tools.Logbook()
log_file = open(FileString + 'log_file.txt', 'a')

#************************************************* Reading/Writing job dictionary and all **************************************
# # This is the portion which you comment if you are reading some older generated sequence.
# # Coz its related to writing the job dictionaries and all for future use.
# reader_classbased_dict.job_representation(FileString)     #the jobs are converted from the generation version to the representation version over here.
# job_dict, sub_job_dict = reader_classbased_dict.get_job_dict(FileString)        #create a job dictionary out of the job representation
# print(job_dict)
# print(sub_job_dict)
# pickle.dump(job_dict, open(FileString + "job_dict.pkl", "wb"))
# pickle.dump(sub_job_dict, open(FileString + "sub_job_dict.pkl", "wb"))
# dist_matrix, start_matrix = reader_classbased_dict.get_costmatrix(job_dict, FileString)   #generate a costmatrix out of the job_dictionary
# pickle.dump(dist_matrix, open(FileString + "dist_matrix.pkl", "wb"))
# pickle.dump(start_matrix, open(FileString + "start_matrix.pkl", "wb"))

# this is the portion you uncomment when reading some older generated scheme and giving a kick start to the whole thing
# also comment out the job generation and all the gazebo stuff at the starting.
job_dict = pickle.load(open(FileString + 'job_dict.pkl', 'rb'))
sub_job_dict = pickle.load(open(FileString + 'sub_job_dict.pkl', 'rb'))
dist_matrix = pickle.load(open(FileString + 'dist_matrix.pkl', 'rb'))
start_matrix = pickle.load(open(FileString + 'start_matrix.pkl', 'rb'))

IND_SIZE = len(sub_job_dict)            #what should be the size of each individual in the generation

# **********************************************************************************************************************

#putting in the fitness and robots_assign attribute in to the individual
creator.create("FitnessMin", base.Fitness, weights=(-1.0,))     # marking to deap that its a minimization problem with one fitness function
creator.create("Individual", list, fitness=creator.FitnessMin, robots_assign=None) # defining the specs of an individual (chromosome)

## initiating the individual (deap format)
def initIndividual(icls, ind_size, robot_number):
    # ind = icls(random.sample(range(0,ind_size), ind_size) )#for _ in xrange(size))
    ind = icls(random.sample(sub_job_dict.keys(), ind_size))    # random sampling from the task_id vlaues without replacement
    ind.robots_assign = [[None] for _ in xrange(robot_number)]  # defining robot_assign as a list values defining sub-tour length for each robot
    init_assignment_random(ind_size, ind, robot_num)  # initialize the robot assignments (2nd part of the chromo) for each chromosome
    # ind_valid_random(ind_size, ind, robot_num)             # check if the generated chromosome is valid or not, if not valid then repair.
    return ind

## initialize a population of individuals with some seed value. This seed value should be saved in a seed.pkl file in the
## appropriate folder in order to call this function (filename).
def initIndividual_seed(icls, filename, ind_size, robot_number):
    with open(filename, 'rb') as seed_file: # opening the pickle file at the desired location and reading the seed
        pop_file = pickle.load(seed_file)   # seed is a single chromosome with both parts of the chromosome representation

    # doing this sampling for khana-puree coz couldn't figure a way around this
    ind = icls(random.sample(sub_job_dict.keys(), ind_size))  # random sampling from the task_id vlaues without replacement
    for i in range(len(ind)):
        ind[i] = pop_file[0][i]    # replacing those randomly sampled values with the seed string loaded. (first part only)
    ind.robots_assign = [[None] for _ in xrange(robot_number)]  # defining robot_assign as a list values defining sub-tour length for each robot
    init_assignment_random(ind_size, ind, robot_num)  # initialize the robot assignments (2nd part of the chromo) for each chromosome
    return ind


def evaluate(individual):
    # This one is working when I am not using the distance matrix
    # I would be needing the job_dict everytime coz i don't know the x,y of jobs
    # return fitness_dict.fitness_deap(individual, job_dict, robot_num)

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
toolbox.register("individual_seed", initIndividual_seed, creator.Individual, ind_size= IND_SIZE, robot_number=robot_num, filename = FileString + "seed.pkl")
toolbox.register("population_seed", tools.initRepeat, list, toolbox.individual_seed)

#registering some stats for recording as the algo goes on
stats.register("avg", numpy.mean)
stats.register("std", numpy.std)
stats.register("min", numpy.min)
stats.register("max", numpy.max)


def main():
    start_time = time.time()
    # Starting a text file to save all the fitness values of a single run onto it. (for later record keeping)
    full_execution = open('/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/'
                          + 'RoSTAM__STsrIA_Homo_PCX_genSWAP_1_.005.txt','a')

    # *********************************** Population Initialization Options *******************************************
    # this routine seeks a seed value from the files and creates a population having one seed value and rest all mutated
    # chromosomes of that seed value. Will apply the panelty function to all those mutants and add those panelties to their
    # actual fitnesses before returning the population and their respective fitnesses.
    # pop, fitness = init_seedNvariants(FileString, 'Auction', robot_num, POP_SIZE) # Auction Re-Auction Random Allocation SSI_auction
    # pop, fitness = init_seedNrandom(FileString, 'Auction', robot_num,POP_SIZE)  # Auction Re-Auction Random Allocation SSI_auction

    # this routine generates a random population without any bias and applies the repair operator to all of them to make
    # them feasible solutions. And there is another one below that which doesn't apply the repair operator but applies
    # the panelty function instead.
    pop, fitness = init_random_repair()
    # pop, fitness = init_random_panelty()    #performing poorer than init_random_repair

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
        solution_file.write(str(best) + (str(best.robots_assign)) + (str(best.fitness)) +'\n' )


        #*********************** OFFSPRING SELECTION *************************
        #   # Select the next generation individuals
        # offspring = pop
        # MUTPB = 1.0/len(best)
        # print MUTPB
        offspring = Selection.roulette_select_FPS(pop, POP_SIZE)
        # offspring = Selection.roulette_select_RBS(pop, POP_SIZE)
        # offspring = Selection.Binary_Tournament(pop, POP_SIZE, 2)
        offspring = list(map(toolbox.clone, offspring))   # Clone the selected individuals as the select function returns only a reference to the individuals and a copy must be made to avoid confusion b/w parents and childre

        #******************************* CROSSOVER ***************************
        # Apply crossover on the offspring
        for child1, child2 in zip(offspring[::2], offspring[1::2]): #these two lists [::2], [1::2] are picking even and odd entreis from this list of chromosome (clones/children/offsprings)
            if random.random() < CXPB:                              #first #is starting point and last is step size [1::2] strt from 1 and step with 2 indexes each
                # child1, child2 = Operators.TCX_yuan2013(child1, child2)
                child1, child2 = Operators.permutation_crossover2(child1, child2)
                # child1, child2 = Operators.permutation_Carter_2006(child1, child2)
                del child1.fitness.values           #clean the old fitness values as new ones would come in after the operations are complete
                del child2.fitness.values

        #******************************** MUTATION *****************************
        # Apply mutation on the offspring
        for mutant in offspring:    #This for loop will stay for both gene based mutation or chromo based mutation
            # # this portion is to be initiated for chromo based mutations
            # if random.random() < MUTPB:
            #     mutant = Operators.reverse_mutation(mutant)

            ## This portion activated for gene based mutation, so that every chromo goes into mutation and MUTPB thing done in mutation routine
            # mutant = Operators.gene_inverse_mut(mutant, MUTPB)
            mutant = Operators.gene_swap_mut(mutant, MUTPB)
            #   #not deleting the fitness of the mutant coz all the fitnesses for offsprings are already deleted in crossover

    #*************************** Validity check and re-evaluation of offsprings ************************************
        total_panelty = [None] * POP_SIZE
        for i, ind in enumerate(offspring):     #for every crossovered and mutated child check if its a valid solution or not
            # ind_valid_random(len(ind), ind, robot_num) # check if the mutated and crossoverd ind. are valid or not.
                                                    # I am doing this for both ST-SR and ST-MR where it really isn't needed for ST-SR
                                                    # will effect my running time.
            total_panelty[i] = ind_valid_panelty(len(ind), ind, robot_num)
        # print total_panelty
        fitnesses = map(evaluate, offspring)  # re-evaluated
        fitnesses = [x + y for x, y in zip(total_panelty, fitnesses)]
        for ind, fit in zip(offspring, fitnesses):        #Assign the re-evaluated fitness values to the right offsprings
            ind.fitness.values = fit,                       # zipping the fitnesses with individuals
        offspring_stats = stats.compile(offspring)
        off_avge.append(offspring_stats.get('avg'))
        off_min.append(offspring_stats.get('min'))

        #************************* SURVIVAL SELECTION *********************
        # pop[:] = offspring    # The population is entirely replaced by the offspring
        combined_pop = pop+offspring
        pop = Selection.truncation(combined_pop, int(POP_SIZE * Elitest_percentage))
        pop += Selection.Binary_Tournament(combined_pop, int(POP_SIZE * (1-Elitest_percentage)), 2)


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
    #writing the last best value in a seperate file as a single entry
    solution_file_ind = open(FileString + 'solution_individual.txt', "wb")
    start = end = 0
    for robot in best.robots_assign:
        end += robot
        solution_file_ind.write(str(best[start:end]) + '\n')
        print (str(best[start:end]) + '\n')
        start += robot
    solution_file_ind.write(str(best.fitness) + str(best.robots_assign))
    solution_file_ind.close()
    solution_file2 = open(FileString + 'solution.txt', "wb")
    solution_file2.write((str(best)) + "\n" + (str(best.robots_assign)) + "\n" + (str(best.fitness)) + "\n")
    solution_file2.close()
    speed_file = open(FileString + 'speed.txt', "wb")
    speed_file.write(str(speed_matrix))
    speed_file.close()

    best_fitness = str(best.fitness)
    best_fitness = best_fitness.replace(")", "")
    best_fitness = best_fitness.replace("(", "")
    best_fitness = best_fitness.replace(",", "")
    full_execution.write(best_fitness + '\n')  # making a single file for all the results

    pickle.dump(logbook, log_file)


    ## Comment the kill stuff so that gazebo runs throughout all the iterations (for iterative operations).
    # os.system("killall roslaunch")
    # os.system("killall python")
    # time.sleep(10)

    #Adding this in the TA phase of things to save the last population, to be
    # started from for next run of EA when new jobs arrive.
    pop_file = open(FileString + 'population_file.pkl', 'wb')
    pickle.dump(pop, pop_file)
    pop_file.close()

    # parsers.for_ampl(robot_num, Jobs, FileString)         # saving in proper format for AMPL to work later on
    plot_TSP.graph_plot(minm, avge, off_avge, off_min, generation_count, Jobs, robot_num, NGEN, POP_SIZE, iteration) #ASF and BSF curves
    plot_TSP.main(FileString, (map_dim *-1)-2, map_dim+2)   # plotting the route map for the final solution
    # auction_traderbots.main(FileString, robot_num, Jobs, speed_matrix)    # running the auction based schemes. (for iterative execution).



    # this is the text file based writing of population for cases where "Individual" is not defined in the place the
    #file is being read from. This problem happend when I was trying to load the pickle in python file where the deap
    #individuals wheren't defined so it wasn't able to recognize the individuals when it was reading them.
    # with open(FileString + 'population_file.txt', 'wb') as pop_file:
    #     pop_file.seek(0)
    #     pop_file.truncate()
    #     for i in pop:
    #         pop_file.write(str(i) + '\n')
    # pop_file.close()
    #
    #
    # with open(FileString + 'population_dist.txt', 'wb') as dist_file:
    #     dist_file.seek(0)
    #     dist_file.truncate()
    #     for i in pop:
    #         dist_file.write(str(i.robots_assign) + '\n')
    # dist_file.close()
    print (time.time() - start_time)

# this initializes the robot_assign part (2nd part of the 2part chromosome) for an indvidual chromosome.One case at a time
#  this distributes the jobs evenly between all the robots. if 15 jobs and 3 robots then 5,5,5 will be the initialization
# the init_assignment routines below does it for the whole population in a single go.
def ind_init_assignment(IND_SIZE, IND,  robot_num):
    fraction = IND_SIZE/float(robot_num)
    ## The way I have assigned tasks for ST-SR-IA things this part of assigning equal capacities to robots has almost become useless. But still keeping this part as it might be useful in the coming problems
    if (fraction).is_integer():
        for j in range(robot_num):                      #if the fraction of jobs for every robot is a whole number then assign it.
                IND.robots_assign[j] = int(fraction)
    else:                                                               #if not a whole number then assign the remainder number one at a time to each robot.
        remainder = IND_SIZE%robot_num
        for j in range(robot_num):
                IND.robots_assign[j] = (int(fraction))
        for j in range(remainder):
                IND.robots_assign[j] += 1

# initializing the 2nd part of the 2part chromo for the whole population in a single go.
# if it is to be used will be used after the initial initialization of whole population is complete and before fitness evaluation
# going for even allocation of jobs if divisible between robots     15 jobs giving 5,5,5 between 3 robots
# if not divisible then add the remainder of jobs at the end one by one
def init_assignment(IND_SIZE, robot_num, pop):
    fraction = IND_SIZE / float(robot_num)
    robot_assignment = [None] * robot_num
    remainder = IND_SIZE % robot_num
    for i in range(robot_num):
        robot_assignment[i] = int(fraction)
    for j in range(remainder):
        robot_assignment[j] += 1  # create a generic list for all robots and assign at the end
    for entry in pop:  # assigning at the end
        entry.robots_assign = robot_assignment


# 2nd part of the chromosome initialization with random numbers.
# the above 2 routines were giving even distributions between robots
def init_assignment_random(IND_SIZE, IND, robot_num):
    tasks_left = IND_SIZE           # keeping track of how many tasks are left to be assigned
    for id in range(robot_num):     # assignments are done for every robot
        if id == robot_num-1:       # if its the last robot to be assigned then assign the left over to it
            IND.robots_assign[id] = tasks_left
        else:                               # for all the robots except the last one.
            value = random.randint(0,tasks_left)        # pick a random number from the range of left over tasks
            IND.robots_assign[id] = value               # assign it to the current robot
            tasks_left -= value                         # subtract the assigned value from the leftover tasks number

##  #******************************* Distance based Fitness functions  **************************************************

# def fitness_costmtrx(chromo, cst_mtrx, rbt_count):
#     start_matrix = cst_mtrx[0][:]
#     rbt_job = [[] for i in range(rbt_count)]  # initializing a list of list for the individual robot jobs for each robot
#     start = 0
#     end = 0
#     for i in range(rbt_count):
#         end += chromo.robots_assign[i]
#         rbt_job[i] = chromo[start:end]
#         start = end
#     start_x, start_y = '0', '0'             #starting location for all the robots. Might have to change this later on
#     fitness = 0
#     for i in range(len(rbt_job)):        #for each and every robot
#         for j in range(len(rbt_job[i])):  # for every single job of robot 'i'
#             if j == 0:                      #if its the first job then start from base and go till first location
#                 job_number = rbt_job[i][j]
#                 fitness += start_matrix[job_number+1] #coz I have appended this extra zero for home-home traveling  +1
#             else:
#                 fitness += cst_mtrx[(rbt_job[i][j-1])+1][(rbt_job[i][j])+1] #+1 +1
#         fitness += start_matrix[rbt_job[i][j]+1]        #This is for completgin the tour. from home to home  +1
#     return fitness


# this is my distance based fitness function. it simply indexes to the right location within the distance matrix and
# picks out the right distance values. Summing them all up provides the total distance travelled for a given solution.
def fitness_costmtrx(chromo, cst_mtrx, rbt_count):
    start_matrix = cst_mtrx[0][:]
    # print cst_mtrx
    rbt_job = [[] for i in range(rbt_count)]  # initializing a list of list for the individual robot jobs for each robot
    start = 0
    end = 0
    for i in range(rbt_count):
        end += chromo.robots_assign[i]
        rbt_job[i] = chromo[start:end]
        start = end
    start_x, start_y = '0', '0'             #starting location for all the robots. Might have to change this later on
    fitness = 0
    jobz = []
    for i in range(len(rbt_job)):        #for each and every robot
        for job in rbt_job[i]:
            data = job.split(",")
            jobz.append(int(data[0]))
            # print ("***************************************************")
            # print jobz
        for j in range(len(jobz)):  # for every single job of robot 'i'
            if j == 0:                      #if its the first job then start from base and go till first location
                # job_number = jobz[j]
                fitness += start_matrix[jobz[j]] #coz I have appended this extra zero for home-home traveling  +1
            else:
                fitness += cst_mtrx[(jobz[j-1])][(jobz[j])] #+1 +1
        fitness += start_matrix[jobz[j]]        #This is for completgin the tour. from home to home  +1
        jobz = []
    return fitness

#few changes in the original fitness_costmtrx function for making it hetro robots.
# just adding a few things like fitness being distance/speed rather than plane distance.
def hetro_fitness(chromo, cst_mtrx, rbt_count):
    start_matrix = cst_mtrx[0][:]
    # print cst_mtrx
    rbt_job = [[] for i in range(rbt_count)]  # initializing a list of list for the individual robot jobs for each robot
    start = 0
    end = 0
    for i in range(rbt_count):
        end += chromo.robots_assign[i]
        rbt_job[i] = chromo[start:end]
        start = end
    start_x, start_y = '0', '0'             #starting location for all the robots. Might have to change this later on
    fitness = 0
    jobz = []
    for i in range(len(rbt_job)):        #for each and every robot
        for job in rbt_job[i]:
            data = job.split(",")
            jobz.append(int(data[0]))
            # print ("***************************************************")
            # print jobz
        for j in range(len(jobz)):  # for every single job of robot 'i'
            if j == 0:                      #if its the first job then start from base and go till first location
                # job_number = jobz[j]
                fitness += start_matrix[jobz[j]]/speed_matrix[i] #coz I have appended this extra zero for home-home traveling  +1
            else:
                fitness += cst_mtrx[(jobz[j-1])][(jobz[j])]/speed_matrix[i] #+1 +1
        fitness += start_matrix[jobz[j]]/speed_matrix[i]        #This is for completgin the tour. from home to home  +1
        jobz = []
    return fitness
#  #********************************************************************************************************************

#  #*********************************** Time based Fitness functions  **************************************************
#very minor changes into the hetro_fitness function, now saving each robots total time into a list as fitness and returning
# the max out of it. This is coz sir asked me to treat the maximum time consumed by a robot (distance/speed) as my fitness
def hetro_fitness_max(chromo, cst_mtrx, rbt_count):
    start_matrix = cst_mtrx[0][:]             # initializing the start matrix, which is just the first line of cost matrix
    rbt_job = [[None]] * rbt_count  # initializing a list of list for the individual robot jobs for each robot
    fitness_list = []  # a list of fitness values to return the max from
    fitness = 0
    previous_task = task = None
    start = end = 0
    for i in range(rbt_count):                # seperating robot sub tours for each robot
        end += chromo.robots_assign[i]
        rbt_job[i] = chromo[start:end]
        start = end
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

# This routine is for ST-MR representation. would locate repetition of a primary job in a robot's substring
# If a single robot is assigned more than one instance of a single job, then assign that instance to another robot
# This one works for the initial population and after crossover or mutation
# The structure of job representation should be "1,2" type otherwise this routine fails.
# this routine looks at the input chromo and decides if its valid or not.
# looks at each individual robot plan one by one if there is repetation of tasks then assigns the repetation to
# another robot (randomly) who doesn't have that particular task assigned already.
def ind_valid_random(ind_size, ind, rbt_count):
    invalid = True              # Simple flag used all along to check where the loops are returning from
    rbt_job = [[] for i in range(rbt_count)]  # initializing a list of list for the individual robot jobs for each robot
    start = end = 0
    for i in range(rbt_count):      # populating each robots sub tour
        end += ind.robots_assign[i]
        rbt_job[i] = ind[start:end]
        start = end

    while(invalid):
        invalid = False
        for i in range(len(rbt_job)):        #for each and every robot
            temp = "-".join(rbt_job[i])  # creating a string out of the job list for a single robot
            temp = "-" + temp  # adding a - to the first task, as the loop on top misses first task
            for job in (rbt_job[i]):         # for every single job of robot 'i'
                job2 = copy.deepcopy(job)    # create a copy of job for later alterations
                data = job.split(",")
                if (temp.count("-" + data[0] + ",")) > 1:     #counting the occurances of jobs
                    adjusted = True
                    while(adjusted):
                        rand_id = random.randint(0, len(rbt_job)-1)
                        temp2 = "-".join(rbt_job[rand_id])
                        temp2 = "-" + temp2
                        if (temp2.count("-" + data[0] + ",")) == 0: # if have zero instances of task in question then assign
                            adjusted = False
                            ind.robots_assign[rand_id] += 1           # increase the robot_assign of this robot by 1
                            ind.remove(job)                     # remove the job from the chromosome
                            rbt_job[i].remove(job)              # remove job from initial robot's sub-tour array
                            rbt_job[rand_id].insert(0,job)           # add it to new robot's sub-tour array
                            # if new robot comes after initial robot then insertion is a bit different than the other case
                            if rand_id > i:   # due to "ind.remove(job)" 2 lines up chromosome's indexing is now a bit jumbled up
                                ind.insert(sum(ind.robots_assign[0:rand_id])-1, job)
                            else:       # if the new robot is before the problem facing robot then the indexing in first
                                ind.insert(sum(ind.robots_assign[0:rand_id]), job)    # part of the chromosome isn't really effected.
                            break
                    ind.robots_assign[i] -= 1
                    invalid = True
                    break
            if invalid:
                break

def ind_valid_panelty(ind_size, ind, rbt_count):
    panelty = 0
    invalid = True              # Simple flag used all along to check where the loops are returning from
    rbt_job = [[] for i in range(rbt_count)]  # initializing a list of list for the individual robot jobs for each robot
    start = end = 0
    ind_split = [i.split(",")[0] for i in ind]
    for i in range(rbt_count):      # populating each robots sub tour
        end += ind.robots_assign[i]
        rbt_job[i] = ind_split[start:end]
        start = end
    for robot in rbt_job:
        for job in robot:
            count = robot.count(job)
            if count > 1:
                panelty += count * 5
    # print panelty
    return panelty

def init_random_repair():
    pop = toolbox.population(n=POP_SIZE)        # for random initialization of population
    for ind in pop:
        ind_valid_random(len(ind), ind,robot_num)  # check if the generated chromosome is valid or not, if not valid then repair.
    fitness = map(evaluate, pop)              #evaluate each individual of the population by mapping the evaluate function over the whole population
    return pop, fitness

def init_random_panelty():
    pop = toolbox.population(n=POP_SIZE)        # for random initialization of population
    total_panelty = [None] * POP_SIZE
    for i, ind in enumerate(pop):  # for every crossovered and mutated child check if its a valid solution or not
        # ind_valid_random(len(ind), ind, robot_num) # check if the mutated and crossoverd ind. are valid or not.
        # I am doing this for both ST-SR and ST-MR where it really isn't needed for ST-SR
        # will effect my running time.
        total_panelty[i] = ind_valid_panelty(len(ind), ind, robot_num)
    fitness = map(evaluate, pop)              #evaluate each individual of the population by mapping the evaluate function over the whole population
    return pop, [x + y for x, y in zip(total_panelty, fitness)]

def init_seedNrandom(FileString, selection_string, robot_num, POP_SIZE):
    Seed_creation.auc_resutl_2_file(FileString, selection_string, robot_num)
    Seed_creation.auction_2_GA_random(FileString, robot_num)
    pop = toolbox.population_seed(n=POP_SIZE)  # for seed based initialization which automatically reads from seed.pkl
    with open(FileString + "seed.pkl", 'rb') as seed_file: # reading seed.pkl again to asign the robot assigns to one
        pickle_file = pickle.load(seed_file)               # individual in the population making it exactly the same as
        pop[0].robots_assign = pickle_file[1]               # the seed value. Rest all pop have random robot_assign
    pop2 = toolbox.population(n=POP_SIZE-1)
    for i in range(1,POP_SIZE):                         # as sir ordered, applying inverse mutation to rest of the population
        pop[i] = pop2[i-1]   # apart from the first one.
    total_panelty = [None] * POP_SIZE
    for i, ind in enumerate(pop):  # if there are any invalid solutions in the population apply panelty to those ind.
        total_panelty[i] = ind_valid_panelty(len(ind), ind, robot_num)
    print total_panelty
    fitness = map(evaluate, pop)  # re-evaluated        # find the fitness of all the individuals
    return pop, [x + y for x, y in zip(total_panelty, fitness)] # merge fitness and panelty and return them as a single fitness value

def init_seedNvariants(FileString, selection_string, robot_num, POP_SIZE):
    Seed_creation.auc_resutl_2_file(FileString, selection_string, robot_num)
    Seed_creation.auction_2_GA_random(FileString, robot_num)
    pop = toolbox.population_seed(n=POP_SIZE)  # for seed based initialization which automatically reads from seed.pkl
    with open(FileString + "seed.pkl", 'rb') as seed_file: # reading seed.pkl again to asign the robot assigns to one
        pickle_file = pickle.load(seed_file)               # individual in the population making it exactly the same as
        pop[0].robots_assign = pickle_file[1]               # the seed value. Rest all pop have random robot_assign
    for i in range(1,POP_SIZE):                         # as sir ordered, applying inverse mutation to rest of the population
        pop[i] = Operators.gene_inverse_mut(pop[i], 0.01)   # apart from the first one.
    total_panelty = [None] * POP_SIZE
    for i, ind in enumerate(pop):  # if there are any invalid solutions in the population apply panelty to those ind.
        total_panelty[i] = ind_valid_panelty(len(ind), ind, robot_num)
    print total_panelty
    fitness = map(evaluate, pop)  # re-evaluated        # find the fitness of all the individuals
    return pop, [x + y for x, y in zip(total_panelty, fitness)] # merge fitness and panelty and return them as a single fitness value

if __name__ == "__main__":
    main()
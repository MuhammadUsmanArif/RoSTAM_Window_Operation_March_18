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
import pprint
import math
from collections import deque

#Adding my routines made before switching to deap
sys.path.append('/home/usman/research_gaz_ros/src/turtlebot_research/src/charm/iterative_execute_ST-SR_MR-IA_TA')
import population_diversity
import penalties

# FileString = "/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/"
# at this particular location there should be a folder with the name of number of tasks e.g. '50'
# inside that folder there should be a folder with the number of iteration e.g. '0'
# inside that the whole process will take place so our destination would be ...../txt_files/ST_SR_TA/50/0/
# if you are running predeveloped jobs then keep them inside the folders according to the iterations 0, 1, 2, 3, 4, 5.+-

parameters = pickle.load(open("/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/parameters.pkl" , 'rb'))
#Evolutioany parameters.
CXPB = parameters['CXPB']              # 0.9 MR homo
# Mutation is altered down in the code (within the epoch) no point setting up here.
MUTPB_overall = parameters['MUTPB']           #MUTPB for gene based mutations is working with relatively lower values (0.005) compared to normal mutation (0.1)
NGEN = parameters['NGEN']             # number of jobs x 80 for MR Homo
POP_SIZE = parameters['POP_SIZE']
after_followup = parameters['after_followup'] #if after followup will help in changing names for the run before adding tasks for the run after adding tasks.

## Other details for Team and Environment
map_dim = parameters['map_dim']   # basically its for the job generation, a 9 means jobs generated in space of -9 -- 9 in x and y both
robot_num = parameters['robot_num']
Jobs = parameters['jobs']
iteration = parameters['iteration']
job_type = parameters['jobs_type']
speed_matrix = parameters['speed_matrix'] #large diff in speed was causing plans to heavily load the faster robots.
start_location = parameters['start_location']#[0, 0, 0, 0, 0, 0, 0]
# this is not being used right now but have added to allow robots to start from different location. The information will
# be passed on to hetro_fitness_max_2 (I guess this one is now outdated plz check) routine which allows robot to start
# form non depot locations # The ending on depot can simply be commented out of the routine. A review of all other schemes
# i.e. Darrah, SSI, auction will be needed if this is implemented.
percentage_to_drop = parameters['hidden_task_percentage']


FileString = parameters['Complete_file_String']    #FileString + iteration_string
# Display input and output file name passed as the args
print ("GA PRINTING ROBOTS : %i GENERATION: %i and POPULATION: %i JOBS: %i ITERATION: %i" % (robot_num,NGEN, POP_SIZE, Jobs, iteration) )


#*****************************Generating jobs and starting gazebo if this iteration is the 0th iteration****************
##This generate job line is to be commented when reading some old generated jobs. Otherwise it stays on
# gen_job.main(Jobs, job_type, FileString, map_dim*-1, map_dim) #calling the gen_job routine residing inside the project. the older versions had a general gen_job file

##This portion of code goes to start all the ROS and gazebo nodes for the first iteration of generating jobs
## For the iterative version The gazebo and ros nodes are started only on first iteration and then are not terminated
## For all the coming iterations they stay on. Don't forget to remove the kill commands at the end of the main routine
# if (iteration == 0): # this condition is important when i run the code through execute coz then this portion would run every time if I don't put the condidtion
#     os.system("roslaunch turtlebot_research mylaunch_gazebo.launch world_file:=/opt/ros/indigo/share/turtlebot_gazebo/worlds/empty.world &")
#     time.sleep(12)
#     ## commenting this thing out and adding only one robot to this coz at the time of cost matrix generation I only need one robot
#     ## there is no point adding 3 or 5 robots. Maybe this will reduce the load on my machine.
#     # for i in range(1, robot_num+1): # starting the indexing from 1 and taking it to robot#+1 coz these number will be used as names in robot launches, don't want a robot0
#     #     os.system("roslaunch turtlebot_research mylaunch_turtlebot_0.2_multiplebots.launch robot_name:='robot" +str(i)+"' robot_frame:='robot" +str(i)+ "_tf' robot_loc:='-z 3 -x 3 -y 0.0' frame_publisher:='3 0 0 0 0 0 map robot" + str(i) + "_tf/odom 100' &")
#     os.system("roslaunch turtlebot_research mylaunch_turtlebot_0.2_multiplebots.launch robot_name:='robot1" + "' robot_frame:='robot1" + "_tf' robot_loc:='-z 3 -x 3 -y 0.0' frame_publisher:='3 0 0 0 0 0 map robot1" + "_tf/odom 100' &")
#     time.sleep(12)
#     print ('mylaunch file launched & jobs generated')
#     os.system("roslaunch turtlebot_research multiple_nav_mapONLY.launch map_file:=/home/usman/research_gaz_ros/src/turtlebot_research/navigation/empty.yaml &")
#     ## Again commenting coz of the 1 robot thing. No need to initiate more than one robot.
#     # for i in range(1, robot_num+1):
#     #     os.system("roslaunch turtlebot_research multiple_nav_rbtONLY.launch  robot_name:='robot" +str(i)+"' robot_frame:='robot" +str(i)+ "_tf' robot_x:='0' robot_y:='0' &")
#     os.system("roslaunch turtlebot_research multiple_nav_rbtONLY.launch  robot_name:='robot1" + "' robot_frame:='robot1" + "_tf' robot_x:='0' robot_y:='0' &")
#     print ('navigation with empty map also launched')
#     time.sleep(12)
#***********************************************************************************************************************

#***************************************** Reading/Writing job dictionary and all **************************************
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
# **********************************************************************************************************************


# **********************************************************************************************************************
## this is the portion you uncomment when reading some older generated scheme and giving a kick start to the whole thing
## also comment out the job generation and all the gazebo stuff at the starting.
job_dict = pickle.load(open(FileString + 'job_dict.pkl', 'rb'))
sub_job_dict = pickle.load(open(FileString + 'sub_job_dict.pkl', 'rb'))
dist_matrix = pickle.load(open(FileString + 'dist_matrix.pkl', 'rb'))
start_matrix = pickle.load(open(FileString + 'start_matrix.pkl', 'rb'))

## **************************** NEW TASK INTRODUCITON FOR DYNAMIC IMPLEMENTAITON **************************************
## Over here we are hiding a few tasks for later introduction into the system
# left_tasks = random.sample(sub_job_dict, int(len(sub_job_dict)*percentage_to_drop)) # removing random tasks (difficult to get the same tasks for SSI)
left_tasks = sub_job_dict.keys()[int(len(sub_job_dict)*(1-percentage_to_drop)):] #hiding last fex tasks in the dict. sounds lame but easier to implement with SSI as well
print 'The total number of tasks for this case are', len(sub_job_dict)
for task in left_tasks:
    del sub_job_dict[task]
print 'the new sub job dict with tasks removed', len(sub_job_dict), sub_job_dict.keys()
print 'the removed tasks are', len(left_tasks), left_tasks

parameters['left_over_tasks'] = left_tasks
pickle.dump(parameters, open(parameters['FileString'] + "parameters.pkl", "wb"))
# **********************************************************************************************************************
IND_SIZE = len(sub_job_dict)            #Size of each individual in the generation

## Adaptive penalty function
#initializing this FIFO queue to implement the panelty function in Michalewicz Paper panelty scheme 3.2.4
# used while calling the panelty_adaptive function in penalties.py
last_k_bests = deque(10*[1], 10)    #This checks if the last few (10) best were valid or not
last_k_improve = deque(10*[0], 10)  # This checks if there is an improvement in the last few gens then don't change the
# penalty value (there is already convergence in the population). Not using this in current version


#pickling the logbook at the end, Have a look at how to view it, coz if you pickle it it wouldn't be normal txt.
logbook = tools.Logbook()
log_file = open(FileString + 'log_file.txt', 'a')

#************************************ Declaring and initializing chromosomes in deap ***********************************
#putting in the fitness and robots_assign attribute in to the individual
creator.create("FitnessMin", base.Fitness, weights=(-1.0,))     # marking to deap that its a minimization problem with one fitness function
creator.create("Individual", list, fitness=creator.FitnessMin, robots_assign=None) # defining the specs of an individual (chromosome)

## initiating the individual (deap format)
def initIndividual(icls, ind_size, robot_number):
    ind = icls(random.sample(sub_job_dict.keys(), ind_size))  # random sampling from the task_id values without replacement
    # ** This is the part where the 2nd part of the chromosome is created and populated
    ind.robots_assign = [[None] for _ in xrange(robot_number)]  # defining robot_assign as a list values defining sub-tour length for each robot
    ind_init_assignment(ind_size, ind, robot_num)  # Even distribution of tasks for each robot
    # random was giving more tasks to starting robots and less to ending, but now its somewhat fixed and now it can give
    # a high number of tasks to any robot in the list and a low number to any other robot.
    # init_assignment_random(ind_size, ind, robot_num)  # initialize the robot assignments randomly
    # ind_valid_random(ind_size, ind, robot_num)        # check if the generated chromosome is valid or not, if not valid then repair. Not a very good idea
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
##**********************************************************************************************************************

def evaluate(individual):
    # This one is working when I am not using the distance matrix
    # I would be needing the job_dict everytime coz i don't know the x,y of jobs
    # return fitness_dict.fitness_deap(individual, job_dict, robot_num)

    # Time based fitness evaluation
    #This one is working when I am using the distance matrix
    # I wouldn't be needing the dictionary coz I dont want x,y in cost matrix just the job index
    return hetro_fitness_max(individual, dist_matrix, robot_num) # for time based fitness function which supports hetro as well as homo team
    #this is used for initial population evaluation. Later generation wise evaluations are done by another routine which
    #combines fitness evaluation and penalty evaluation for better timings

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
    # initializing a few empty list for graph construction at the end
    maxm = list()
    minm = list()
    avge = list()
    stde = list()
    off_avge = list()
    off_min = list()
    start_time = time.time()
    # Starting a text file to save all the fitness values of a single run onto it. (for later record keeping)
    # full_execution = open('/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/' + 'RoSTAM_Results.txt','a')
    full_valid_invalid = open('/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/' + 'invalid.txt','a')
    ## these are two files which I introduced for ASF and BSF recording over multiple runs. The files will then later be
    ## compiled into the average ASF and average BSF version using the file_consolidation.py file
    # BSF = open('/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/' + 'BSF_gaSOLO.txt', "a")
    # ASF = open('/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/' + 'ASF_gaSOLO.txt', "a")


    # *********************************** Population Initialization Options *******************************************
    # this routine seeks a seed value from the files and creates a population having one seed value and rest all mutated
    # chromosomes of that seed value. Will apply the penalty function to all those mutants and add those penalties to their
    # actual fitnesses before returning the population and their respective fitnesses.
    # pop, fitness = init_seedNvariants(FileString, 'SSI_auction', robot_num, POP_SIZE) # Auction Re-Auction Random Allocation SSI_auction
    # pop, fitness = init_seedNrandom(FileString, 'Random Allocation', robot_num,POP_SIZE)  # Auction Re-Auction Random Allocation SSI_auction

    # this routine generates a random population without any bias and applies the repair operator to all of them to make
    # them feasible solutions. And there is another one below that which doesn't apply the repair operator but applies
    # the penalty function instead.
    # pop, fitness = init_random_repair()
    pop, fitness = init_random_penalty()    #unlike the name this one will go for even initialization of robots
    # assigning the fitness to corresponding individuals
    for ind, fit in zip(pop, fitness):     #allocationg the fitness values returned from the top
        ind.fitness.values = fit,
    ##******************************************************************************************************************

    #getting the solution file and the statistics file ready for writing on them later on
    solution_file = open(FileString + str(NGEN) + 'G_' + str(POP_SIZE) + 'P_' + str(Jobs) + 'J_' + str(robot_num) + 'R_' + str(iteration) + 'I_jobs_solutions.txt', 'a')
    solution_file.seek(0)
    solution_file.truncate()
    stats_file = open(FileString + str(NGEN) + 'G_' + str(POP_SIZE) + 'P_' + str(Jobs) + 'J_' + str(robot_num) + 'R_' + str(iteration) + 'I_jobs_stats.txt', 'a')
    stats_file.seek(0)
    stats_file.truncate()


    generation_count = 0
    penalty_value = 1       #This is the starting penalty value. The algorithm will adjust its penalty automatically
    best_valid_ind = pop[0] #This variable holds the best valid solution found throughout the runs as the algo could loose the valid best

   #Running the EA generations
    for g in range(NGEN):
        #picking the best individual from the generation and writing it to the solution file
        best = max(pop, key=operator.attrgetter('fitness')) # This will be max, tried the min thing, it was returning the worst value

        solution_file.write(str(best) + (str(best.robots_assign)) + (str(best.fitness)) +'\n' )
        # ****************** RECORDING STATISTICS ****************************
        #   Record keeping
        record = stats.compile(pop)
        minm.append(record['min'])
        maxm.append(record['max'])
        avge.append(record['avg'])
        stde.append(record['std'])
        # BSF.write(str(record['min'])+'\n')
        # ASF.write(str(record['avg'])+'\n')
        stats_file.write(str(record) + (str(g)) + '\n')

        # Print details on the display every 100th generation
        if generation_count%100 == 0:
            print(record)
            print('generation count', generation_count)
            if penalties.check_invalid(len(best), best, robot_num):
                print '***************best is invalid*******************'

        # forming a few new groups for offspring. One is for the normal mu + lambda formation. The other is for crowding
        # offspring = copy.deepcopy(pop)  #keeping a copy of current population for offspring creation
        offspring = []
        offspring_new = []

        ## developing this sorted pop for the stochastic universal parent selection. as a sorted list is passed on to it.
        pop_sorted = copy.deepcopy(pop)
        pop_sorted.sort(key=lambda x: x.fitness.values[0])

        MUTPB = 0.3#MUTPB_overall / len(best)         #this if MUTPB_overall is 0.5 basically translates to a 50% chance of mutation in the whole chromosome
        # hamming_matrix = population_diversity.compute_hamming_matrix(IND_SIZE, POP_SIZE, pop)   #computing the hamming matrix once every generation

        for i in range(0, POP_SIZE, 2):

            # ***************************** Routine parent selection ***************************************************
            parent1, parent2 = Selection.roulette_select_FPS_ind(pop, 2)
            # parent1, parent2 = Selection.roulette_select_RBS_ind(pop, 2)
            # parent1, parent2 = Selection.stochastic_universal_ind(pop_sorted, 2)


            # # ***************************** Incest for population diversity *********************************************
            # selection_OK = True  # just a flag to check if the selected parents are diverse enough
            # count = 0
            # chromo_size = len(pop[0])  # could read individual size from any chromosome within the population
            # percentage_closeness = parameters['percentage_closeness_1']
            # while selection_OK:  # Until the perfect pair of parents is figured out
            #     # parent1, parent2 = Selection.roulette_select_FPS_ind(pop, 2)
            #     parent1, parent2 = Selection.roulette_select_RBS_ind(pop, 2)
            #     parent1_index = pop.index(parent1)
            #     parent2_index = pop.index(parent2)
            #     distance_hamming = hamming_matrix[parent1_index][parent2_index]
            #     count += 1  # keeping the count, so that if you have gone through the population once without finding parents then reduce your closeness measure
            #     if distance_hamming > (chromo_size * percentage_closeness):  # if they are far enough then terminate the while loop
            #         selection_OK = False
            #     if count > len(
            #             pop) and percentage_closeness > 0:  # if not able to find parents after a complete pass then reduce closeness. a complete pass doesn't necessarily mean that you have checked all individual chromosomes.
            #         percentage_closeness -= (percentage_closeness * .05)
            #         if percentage_closeness < 0.01:  # there is no point going below 0.01, coz chromosome size of 200 * .001 translates into .2 which is useless, will never have value below 1 for binary hamming
            #             selection_OK = False
            #         count = 0

            par1 = child1 = copy.deepcopy(parent1)
            par2 = child2 = copy.deepcopy(parent2)
            frequency = [0] * (IND_SIZE)  # garbage array for the hamming distance thing
            # print 'this is the status between parent 1 and child1 and parent 2 and child 2', population_diversity.hamming_dist(parent1, child1, frequency), population_diversity.hamming_dist(parent2, child2, frequency)

            ## *********************************** Crossovers **********************************************************
            if random.random() < CXPB:
                # child1, child2 = Operators.robot_based_BOX(par1, par2, best_valid_ind)
                # child1, child2 = Operators.BOX(par1, par2, best_valid_ind)
                child1, child2 = Operators.permutation_Carter_2006(par1, par2)
                # child1, child2 = Operators.ordered_crossover(par1, par2)
                # child1, child2 = Operators.TCX_yuan2013(par1, par2)
                # child1, child2 = Operators.permutation_crossover2(par1, par2)
            # print 'relation after crossover', population_diversity.hamming_dist(parent1, child1, frequency), population_diversity.hamming_dist(parent2, child2, frequency)

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

            ## inventing this even odd heuristic coz the gene_inverse_mut_II tends to be a bit slow at the end of the show.
            # if g % 2 == 0:
            if random.random() < 0.5:
                child1 = Operators.gene_swap_mut(child1, MUTPB)
                child2 = Operators.gene_swap_mut(child2, MUTPB)
            else:  # For each odd generation apply inverse mutation
            #     ## child1 = Operators.gene_inverse_mut_II(child1, MUTPB, g + 1)
            #     ## child2 = Operators.gene_inverse_mut_II(child2, MUTPB, g + 1)
                child1 = Operators.gene_inverse_mut(child1, MUTPB)
                child2 = Operators.gene_inverse_mut(child2, MUTPB)
                # child1 = Operators.gene_insert_mut(child1, MUTPB)
                # child2 = Operators.gene_insert_mut(child2, MUTPB)
            # print 'relation after mutation', population_diversity.hamming_dist(parent1, child1,frequency), population_diversity.hamming_dist(parent2, child2, frequency)
            ##**********************************************************************************************************


            ## This portion basically makes the children feasible after applying the operations.
            # if (generation_count == 0):             # make the children feasible for only the first generation
            #     child1 = ind_valid_random(len(child1), child1, robot_num)
            #     child2 = ind_valid_random(len(child2), child2, robot_num)


            ##*********************************** Penalty functions ***************************************************
            # penalty1 = penalties.penalty_dynamic(len(child1), child1, robot_num, generation_count)
            # penalty2 = penalties.penalty_dynamic(len(child2), child2, robot_num, generation_count)

            ## This penalty_fix function is used in all the population initialization routines where ever the penalty is applicable.
            # penalty1 = penalties.penalty_fix(len(child1), child1, robot_num, generation_count)
            # penalty2 = penalties.penalty_fix(len(child2), child2, robot_num, generation_count)

            # penalty1 = penalties.penalty_adaptive(len(child1), child1, robot_num, generation_count, last_k_bests)
            # penalty2 = penalties.penalty_adaptive(len(child2), child2, robot_num, generation_count, last_k_bests)
            # print 'child penalties:', penalty1, penalty2

            ## ********************************** Fitness evaluations and allocations *********************************
            # fitness1 = evaluate(child1)
            # fitness2 = evaluate(child2)
            # print 'fitnesses old',fitness1 + penalty1, fitness2 + penalty2
            # print 'fitness new', fitness_maxNpenalty(child1, dist_matrix, robot_num),fitness_maxNpenalty(child2, dist_matrix, robot_num)
            # child1.fitness.values = (fitness1 + penalty1),
            # child2.fitness.values = (fitness2 + penalty2),
            ## I have merged the fitness and penlaty function thats why the top penalty function portion is totally commented.
            child1.fitness.values = fitness_maxNpenalty(child1, dist_matrix, robot_num, penalty_value),
            child2.fitness.values = fitness_maxNpenalty(child2, dist_matrix, robot_num, penalty_value),

            ## *********************************** Population Diversity ************************************************
            ## This is suggested by Dr.Sajjad to not apply population diversity measure to the whole population but to
            ## apply it to a part of the population. If the below value is %4 then it is applied to 50% population as the
            ## for loop is running with a step size of 2.
            if i % 10 == 0:  #Since the i is looping with step size 2, half of the population will go for mu + lambda
                combined_list = [parent1, parent2, child1, child2]  #These 4 to the next generation without any survival
                min_fit_ind = min(combined_list, key=operator.attrgetter('fitness.values')) # pick the fittest amongst the 4
                offspring_new.append(min_fit_ind)
                combined_list.remove(min_fit_ind)   # remove the fittest from the list
                frequency = [0] * (IND_SIZE)  # garbage array for the hamming distance thing

                ## Variety of distance measures, hamming dist, edit distance, sequ_match robot tour based, and seq match whole solution based
                minimum_dist = max(combined_list,
                                   key=lambda x: population_diversity.hamming_dist(x, min_fit_ind, frequency))
                # hamming dist returns 0 for completely different chromos so the min() will pick the most dissimilar one
                # minimum_dist = min(combined_list,
                #                    key=lambda x: population_diversity.edit_dist(x, min_fit_ind))
                # minimum_dist = max(combined_list,
                #                    key=lambda x: population_diversity.seq_match_ratio(x, min_fit_ind))
                # minimum_dist = max(combined_list,
                #                      key=lambda x: population_diversity.seq_match_ratio_full(x, min_fit_ind))
                # print population_diversity.hamming_dist(minimum_dist, min_fit_ind, frequency)
                offspring_new.append(minimum_dist) #send the most dissimilar (to the best) to the next gen.
            else:
                offspring.append(child1)
                offspring.append(child2)

            ## this is Deterministic crowding. Make a few compares of which child is close to what parent.
            ## hold a competation amongst the close parent child pair to decide which one will proceed.
            # frequency = [0] * (IND_SIZE)  # measure how many times a particular gnene is repeated within the pop
            # distance_parent1 = population_diversity.hamming_dist(parent1, child1, frequency)
            # distance_parent2 = population_diversity.hamming_dist(parent2,child1, frequency)
            # if distance_parent1 > distance_parent2:
            #     if parent1.fitness.values[0] > child1.fitness.values[0] and parent1 in pop:
            #         parent1_index = pop.index(parent1)
            #         pop[parent1_index] = child1
            #
            # else:
            #     if parent2.fitness.values[0] > child1.fitness.values[0] and parent2 in pop:
            #         parent2_index = pop.index(parent2)
            #         pop[parent2_index] = child1
            #
            # frequency = [0] * (IND_SIZE)  # measure how many times a particular gnene is repeated within the pop
            # distance_parent1 = population_diversity.hamming_dist(parent1, child2, frequency)
            # distance_parent2 = population_diversity.hamming_dist(parent2, child2, frequency)
            # if distance_parent1 > distance_parent2:
            #     if parent1.fitness.values[0] > child2.fitness.values[0] and parent1 in pop:
            #         parent1_index = pop.index(parent1)
            #         pop[parent1_index] = child2
            #
            # else:
            #     if parent2.fitness.values[0] > child2.fitness.values[0] and parent2 in pop:
            #         parent2_index = pop.index(parent2)
            #         pop[parent2_index] = child2

            ## this was for usual survival selection where the offspring pool is maintained seperately and then sent for the
            ## competation.
            ## Putting them into the pool overwriting the existing members
            # offspring[i] = child1
            # offspring[i+1] = child2         #if you get an index out of range error at this point its coz of odd numbered for loop not matching your allowed number of solutions

        # ************************* SURVIVAL SELECTION *********************
        # pop[:] = offspring_new    # The population is entirely replaced by the offspring
        for ind in pop:
            ind.fitness.values = fitness_maxNpenalty(ind, dist_matrix, robot_num, penalty_value),
        combined_pop = copy.deepcopy(pop + offspring)   #this is for the 50% of the population seprated on the top.

        # ************************* Diversity Measures Fitness Sharing *********************
        # This is the part I added at the time of GA testing Where this computes the Hamming distance b/w chromosomes
        # hamming_matrix = population_diversity.compute_hamming_matrix(IND_SIZE, POP_SIZE, combined_pop)
        # combined_pop = population_diversity.fitness_sharing(hamming_matrix, combined_pop)

        # ************************* SURVIVAL SELECTION *********************
        pop = Selection.truncation(combined_pop, int(1))  # picking the best as an elite
        # for ind in pop:
        #     combined_pop.remove(ind)
        pop += Selection.Binary_Tournament(combined_pop, int(POP_SIZE - 1) - len(offspring_new), 2)  # * (1-Elitest_percentage)), 2)
        # pop += Selection.roulette_select_FPS(combined_pop, int(POP_SIZE -1))#* (1-Elitest_    percentage)))

        ## merging the population portion implemented through crowding with the population which has already undergone survival selection
        # pop = Selection.truncation(pop, len(offspring_new))
        pop += offspring_new
        best = max(pop, key=operator.attrgetter('fitness'))  # This will be max, tried the min thing, it was returning the worst value

        # ************************************* Adaptive Penalty ********************************************************
        ## Since I am implementing a variable penalty function so I don't want the best solution found so far to be lost
        ## This is where at the end of each generation we check if there is a new valid best solution and save it.
        ## Also update the arrays which are maintaining the record for the previous 10 generations convergence
        if not penalties.check_invalid(len(best), best, robot_num) and best.fitness.values[0] < best_valid_ind.fitness.values[0]:
            # print 'changing the best valid it fitness before and after', best_valid_ind.fitness.values[0], best.fitness.values[0]
            best_valid_ind = copy.deepcopy(best)   #save the latest valid best individual found
            # print best is best_valid_ind
            last_k_improve.appendleft(1)    #register that there is some convergece in this generation (valid convergence)
        else:
            last_k_improve.appendleft(0)    #register that there is no valid convergence in this generation.

        ## This portion is for recording the FIFO list initiated at top for penalty_adaptive function
        if penalties.check_invalid(len(best),best,robot_num):
            # print '***************best is invalid*******************
            last_k_bests.appendleft(1)  #if the best is invalid in this gen then append 1
        else:
            last_k_bests.appendleft(0)  # else append a 0
        if g%10 == 0:   # at the end of every 10 generations
            if last_k_bests.count(1) == 10 and penalty_value < 20: #if all previous 10 have been invalid then increase penalty
                penalty_value += penalty_value*.15
                # print 'penalty value changed to', penalty_value
            elif last_k_bests.count(0) == 10: # and last_k_improve.count(0) == 10: #and if all have been valid then decrease penalty
                penalty_value -= penalty_value*.15
                # print 'penalty value changed to', penalty_value

        #***************************************** AIS based Population diversity measure ******************************
        # # I am tinkering with this routine while commented, if doesn't work please have a look at last backup before 9/10/2018
        if g % 10 == 0:
            pop = Selection.truncation(pop, 80)   #dropping the worst 5 solutions
            pop_AIS = toolbox.population(20)       # generating 5 random solutions
            for i, ind in enumerate(pop_AIS):  # for every new random ind check the amount of penalty
               ind.fitness.values = fitness_maxNpenalty(ind, dist_matrix, robot_num, 0),
            pop += pop_AIS

        logbook.record(gen=0, **record)
        generation_count += 1
        # print('generation count' , generation_count)

    ##********************************************* Record Keeping *****************************************************
    #writing the last best value in the best value record.
    best = best_valid_ind
    solution_file.write((str(best)) + (str(best.robots_assign)) + (str(best.fitness)) + '\n')
    solution_file.write(str(hetro_fitness_list(best, dist_matrix, robot_num)))
    iteration_solution_file = open(FileString + 'iteration_solution', 'a')
    print('*************************This is the fitness for the final solution', best.fitness)
    iteration_solution_file.writelines(str(best.fitness.values[0]) + '\n')
    iteration_solution_file.close()
    stats_file.close()
    solution_file.close()

    solution_file_ind = open(FileString + 'solution_individual.txt', "wb") #after tasks are added (iterative)

    start = end = 0
    #printing individual tours for each robot.
    for robot in best.robots_assign:
        end += robot
        solution_file_ind.write(str(best[start:end]) + '\n')
        print (str(best[start:end]) + '\n')
        start += robot

    solution_file_ind.write(str(best.fitness) + str(best.robots_assign))
    solution_file_ind.write(str(hetro_fitness_list(best, dist_matrix, robot_num)))
    solution_file_ind.close()
    solution_file2 = open(FileString + 'solution.txt', "wb")
    solution_file2.write((str(best)) + "\n" + (str(best.robots_assign)) + "\n" + (str(best.fitness)) + "\n")
    solution_file2.write('left over tasks' + str(left_tasks))
    solution_file2.close()
    speed_file = open(FileString + 'speed.txt', "wb")
    speed_file.write(str(speed_matrix))
    speed_file.close()

    best_fitness = str(best.fitness)
    best_fitness = best_fitness.replace(")", "")
    best_fitness = best_fitness.replace("(", "")
    best_fitness = best_fitness.replace(",", "")

    # full_execution.write(best_fitness + '\n')  # making a single file for all the results, RoSTAM_Results.txt
    # full_execution.close()

    # keeping a record if the best of last generation was a valid chromo or not.
    if penalties.check_invalid(len(best_valid_ind), best_valid_ind, robot_num):
        full_valid_invalid.write('invalid'+'\n')
    else:
        full_valid_invalid.write('valid'+'\n')

    pickle.dump(logbook, log_file)

    # print "solution from previous fitness function", hetro_fitness_max(best, dist_matrix, robot_num)
    # print 'solutoin from new fitness function', hetro_fitness_max_2(best, dist_matrix, robot_num, start_location)

    ## Comment the kill stuff so that gazebo runs throughout all the iterations (for iterative operations).
    # os.system("killall roslaunch")
    # os.system("killall python")
    # time.sleep(10)

    #Adding this in the TA phase of things to save the last population, to be
    # started from for next run of EA when new jobs arrive.
    pop_file = open(FileString + 'population_file.pkl', 'wb')
    pickle.dump(pop, pop_file)
    pop_file.close()

    # The best solution also saved for the same purpose so that on the next run we know what was the solution previously followed.
    best_file = open(FileString + 'best_solution.pkl', 'wb')
    pickle.dump(best, best_file)
    best_file.close()
    ##*******************************************************************************************************************

    # parsers.for_ampl(robot_num, Jobs, FileString)         # saving in proper format for AMPL to work later on
    plot_TSP.graph_plot(minm, avge, off_avge, off_min, generation_count, Jobs, robot_num, NGEN, POP_SIZE, iteration) #ASF and BSF curves
    # plot_TSP.main(FileString, (map_dim *-1)-2, map_dim+2, 1)   # plotting the route map for the final solution
    # auction_traderbots.main(FileString, robot_num, Jobs, speed_matrix)    # running the auction based schemes. (for iterative execution).

    # BSF.close()
    # ASF.close()

    print (time.time() - start_time)

# this initializes the robot_assign part (2nd part of the 2part chromosome) for an indvidual chromosome.One case at a time
#  this distributes the jobs evenly between all the robots. if 15 jobs and 3 robots then 5,5,5 will be the initialization
# This one does it for one solution at a time. The init_assignment routines below does it for the whole population in a single go.
def ind_init_assignment(IND_SIZE, IND,  robot_num):
    fraction = IND_SIZE/float(robot_num)
    IND.robots_assign = [int(fraction)] * robot_num
    ## The way I have assigned tasks for ST-SR-IA things this part of assigning equal capacities to robots
    ## has almost become useless. But still keeping this part as it might be useful in the coming problems
    if (fraction).is_integer():
        None
    else:  #if not a whole number then assign the remainder number randomly to the robots.
        # No need to consider failed robots coz there are no failed robots in the first iteration.
        remainder = IND_SIZE%robot_num
        for j in range(remainder):
                IND.robots_assign[random.randint(0,robot_num-1)] += 1

# initializing the 2nd part of the 2part chromo for the whole population in a single go.
# if it is to be used will be used after the initial initialization of whole population is complete and before fitness evaluation
# going for even allocation of jobs if divisible between robots 15 jobs giving 5,5,5 between 3 robots
# if not divisible then add the remainder of jobs at the end one by one
def init_assignment(IND_SIZE, robot_num, pop):
    fraction = IND_SIZE / float(robot_num)
    robot_assignment = [None] * robot_num
    remainder = IND_SIZE % robot_num
    for i in range(robot_num):
        robot_assignment[i] = int(fraction)
    for j in range(remainder):
        robot_assignment[j] += 1  # This will allocate remainder to only starting few robots
    for entry in pop:  # assigning at the end
        entry.robots_assign = robot_assignment


# 2nd part of the chromosome initialization with random numbers.
# the above 2 routines were giving even distributions between robots
# This was giving high weightage to starting few robots and low to the ending few
def init_assignment_random(IND_SIZE, IND, robot_num):
    tasks_left = IND_SIZE           # keeping track of how many tasks are left to be assigned
    for id in range(robot_num):     # assignments are done for every robot
        if id == robot_num-1:       # if its the last robot to be assigned then assign the left over to it
            IND.robots_assign[id] = tasks_left
        else:                               # for all the robots except the last one.
            value = random.randint(0,tasks_left)        # pick a random number from the range of left over tasks
            IND.robots_assign[id] = value               # assign it to the current robot
            tasks_left -= value                         # subtract the assigned value from the leftover tasks number
    list_assign = IND.robots_assign
    IND.robots_assign = random.sample(list_assign, robot_num)


#  #*********************************** Time based Fitness functions  **************************************************

# Exactly same as hetro_fitness_max, will just return the whole list rather than just the max. making to write this list into the saved .txt solution file at the end
def hetro_fitness_list(chromo, cst_mtrx, rbt_count):
    start_matrix = cst_mtrx[0][:]             # initializing the start matrix, which is just the first line of cost matrix
    rbt_job = [[None]] * rbt_count  # initializing a list of list for the individual robot jobs for each robot
    fitness_list = []  # a list of fitness values to return the max from
    fitness = 0
    previous_task = task = None
    start = end = 0
    for i in range(rbt_count):  # for each and every robot
        end += chromo.robots_assign[i]
        rbt_job[i] = chromo[start:end]
        start = end
        for j in range(len(rbt_job[i])):
            task = int(rbt_job[i][j].split(',')[0])
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
    return fitness_list                                    #returning the max

# This routine works same as hetro_fitness_max combined with the variable penalty so that both can be computed
# inside a single routine and returned.
def fitness_maxNpenalty(chromo, cst_mtrx, rbt_count, penalty_value):
    start_matrix = cst_mtrx[0][:]   # initializing the start matrix, which is just the first line of cost matrix
    rbt_job = [[None]] * rbt_count  # initializing a list of list for the individual robot jobs for each robot
    fitness_list = []               # a list of fitness values to return the max from
    fitness = 0
    penalty = 0
    previous_task = None
    task = None
    start = end = 0
    chromo_split = [int(i.split(",")[0]) for i in chromo]   #getting rid of the subtasks and having only task ids as int
    for i in range(rbt_count):        #for each and every robot populate the tour for that robot
        end += chromo.robots_assign[i]
        rbt_job[i] = chromo_split[start:end]
        start = end
        already_counted = []
        for j in range(len(rbt_job[i])):    #for each task within the tour
            #********************* the fitness computation part ********************************************
            task = rbt_job[i][j]
            if j == 0:   #if its the first job then start from base and go till first location
                fitness += start_matrix[task]/speed_matrix[i]
            else:
                fitness += cst_mtrx[previous_task][task]/speed_matrix[i]
            previous_task = task
            #****************************** The penalty part ***********************************************
            count = rbt_job[i].count(task)
            if task not in already_counted: #if task attempted more than once and is already not counted for then
                penalty += (count-1)*penalty_value
            already_counted.append(task)    #record keeping so that the task is not recounted

        # # going back to the home depot if the robot ever attempted any task
        # if len(rbt_job[i]) > 0:   # checking if this particular robot has no jobs then there is no point completing the tour
        #     fitness += start_matrix[task]/speed_matrix[i]        #This is for completing the tour. from home to home  +1
        fitness_list.append(fitness)                            #saving each robots fitness value into the list
        # for a normal distance based fitness just return the sum of this list with speed values set to 1 for all robots
        fitness = 0
    fitness = max(fitness_list)     #since the same penalty will be applied to each robots tour the realative max min
    return fitness+penalty          # will remain the same so adding penalty at the end

# #very minor changes into the hetro_fitness function, now saving each robots total time into a list as fitness and returning
# # the max out of it. This is coz sir asked me to treat the maximum time consumed by a robot (distance/speed) as my fitness
def hetro_fitness_max(chromo, cst_mtrx, rbt_count):
    start_matrix = cst_mtrx[0][:]             # initializing the start matrix, which is just the first line of cost matrix
    rbt_job = [[None]] * rbt_count  # initializing a list of list for the individual robot jobs for each robot
    fitness_list = []  # a list of fitness values to return the max from
    fitness = 0
    previous_task = None
    task = None
    start = end = 0
    for i in range(rbt_count):        #for each and every robot
        end += chromo.robots_assign[i]
        rbt_job[i] = chromo[start:end]
        start = end
        for j in range(len(rbt_job[i])):
            task = int(rbt_job[i][j].split(',')[0])
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
#
# # This routine will work same as hetro_fitness_max but with variable starting locations than the depot.
# # if not working then check the latest version from deap__followup.
# def hetro_fitness_max_2(chromo, cst_mtrx, rbt_count, start_locations):
#     start_matrix = cst_mtrx[0][:]             # initializing the start matrix, which is just the first line of cost matrix
#     rbt_job = [[None]] * rbt_count  # initializing a list of list for the individual robot jobs for each robot
#     fitness_list = []  # a list of fitness values to return the max from
#     fitness = 0
#     previous_task = None
#     task = None
#     start = 0
#     end = 0
#     for i in range(rbt_count):        #for each and every robot
#         end += chromo.robots_assign[i]
#         rbt_job[i] = chromo[start:end]
#         start = end
#         for j in range(len(rbt_job[i])):
#             task = int(rbt_job[i][j].split(',')[0])
#             if j == 0:                      #if its the first job then read the starting location from start_location and then find the distance.
#                 fitness += cst_mtrx[start_locations[i]][task]/speed_matrix[i] #So basically the previous task is the starting task read from start_location and till the first task of this particular robot.
#             else:
#                 fitness += cst_mtrx[previous_task][task]/speed_matrix[i] #+1 +1
#             previous_task = task
#         if len(rbt_job[i]) > 0:   # checking if this particular robot has no jobs then there is no point completing the tour
#             fitness += start_matrix[task]/speed_matrix[i]        #This is for completing the tour. from home to home  +1
#         fitness_list.append(fitness)                            #saving each robots fitness value into the list
#         # for a normal distance based fitness just return the sum of this list with speed values set to 1 for all robots
#         fitness = 0
#     return max(fitness_list)                                    #returning the max

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
    return ind

## These initialization routines are called at the start of the algorithm. Where they further call the population
## initialization and then based on their core logic either apply a penalty on that random initialization or repair them
def init_random_repair():
    pop = toolbox.population(n=POP_SIZE)        # for random initialization of population
    for ind in pop:
        ind_valid_random(len(ind), ind,robot_num)  # check if the generated chromosome is valid or not, if not valid then repair.
    fitness = map(evaluate, pop)              #evaluate each individual of the population by mapping the evaluate function over the whole population
    return pop, fitness

def init_random_penalty():
    pop = toolbox.population(n=POP_SIZE)        # for random initialization of population
    total_penalty = [None] * POP_SIZE
    for i, ind in enumerate(pop):  # for every crossovered and mutated child check if its a valid solution or not
        # ind_valid_random(len(ind), ind, robot_num) # check if the mutated and crossoverd ind. are valid or not.
        # I am doing this for both ST-SR and ST-MR where it really isn't needed for ST-SR
        # will effect my running time.
        total_penalty[i] = penalties.penalty_fix(len(ind), ind, robot_num,1)
    fitness = map(evaluate, pop)              #evaluate each individual of the population by mapping the evaluate function over the whole population
    return pop, [x + y for x, y in zip(total_penalty, fitness)]

# use the auction_solution.txt to create a seed and then use that seed and other random solutions to form the whole population.
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
    total_penalty = [None] * POP_SIZE
    for i, ind in enumerate(pop):  # if there are any invalid solutions in the population apply penalty to those ind.
        total_penalty[i] = penalties.penalty_fix(len(ind), ind, robot_num, 1)
    print total_penalty
    fitness = map(evaluate, pop)  # re-evaluated        # find the fitness of all the individuals
    return pop, [x + y for x, y in zip(total_penalty, fitness)] # merge fitness and penalty and return them as a single fitness value

def init_seedNvariants(FileString, selection_string, robot_num, POP_SIZE):
    Seed_creation.auc_resutl_2_file(FileString, selection_string, robot_num)
    Seed_creation.auction_2_GA_random(FileString, robot_num)
    pop = toolbox.population_seed(n=POP_SIZE)  # for seed based initialization which automatically reads from seed.pkl
    with open(FileString + "seed.pkl", 'rb') as seed_file: # reading seed.pkl again to asign the robot assigns to one
        pickle_file = pickle.load(seed_file)               # individual in the population making it exactly the same as
        pop[0].robots_assign = pickle_file[1]               # the seed value. Rest all pop have random robot_assign
    for i in range(1,POP_SIZE):                         # as sir ordered, applying inverse mutation to rest of the population
        pop[i] = Operators.gene_inverse_mut(pop[i], 0.01)   # apart from the first one.
    total_penalty = [None] * POP_SIZE
    for i, ind in enumerate(pop):  # if there are any invalid solutions in the population apply penalty to those ind.
        total_penalty[i] = penalties.penalty_fix(len(ind), ind, robot_num, 1)
    print total_penalty
    fitness = map(evaluate, pop)  # re-evaluated        # find the fitness of all the individuals
    return pop, [x + y for x, y in zip(total_penalty, fitness)] # merge fitness and penalty and return them as a single fitness value


if __name__ == "__main__":
    main()
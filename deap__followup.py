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
import copy
from collections import deque
#Adding my routines made before switching to deap
sys.path.append('/home/usman/research_gaz_ros/src/turtlebot_research/src/charm/execute_ST-SR_MR-IA_TA')
# import Evolutionary.selection
import reader_classbased_dict
import fitness_dict
import penalties
import Selection
import population_diversity
import plot_TSP

parameters = pickle.load(open("/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/parameters.pkl" , 'rb'))

map_dim = parameters['map_dim']   # basically its for the job generation, a 9 means jobs generated in space of -9 -- 9 in x and y both
robot_num = parameters['robot_num_new']
failed_robots = parameters['failed_robots'] #updated in Execution_followup by the failed robot list of list I am providing as input

#Evolutioany parameters.
CXPB = parameters['CXPB_2']
MUTPB_overall = parameters['MUTPB_2']
NGEN = parameters['NGEN_2']
POP_SIZE = parameters['POP_SIZE']

Jobs = parameters['jobs']
iteration = parameters['iteration']
execution_number = parameters['execution_number']
speed_matrix = parameters['speed_matrix_2']
attempted_tasks = parameters['attempted_tasks']
start_location = parameters['start_location_2']  # so these are job ids against every individual robot where it would start from.

# The information will be passed on to hetro_fitness_max_2 ruotine which allows robot to start form non depot locaitons
# The ending on depot can simply be commented out of the routine. A review of all other schemes
# i.e. Darrah, SSI, auction will be needed if this is implemented.
FileString = parameters['Complete_file_String']
print ("Deap Followup : %i Generation: %i and Population: %i Jobs: %i Iteration: %i" % (robot_num,NGEN, POP_SIZE, Jobs, iteration) )

###############################
# o == option
# a == argument passed to the o
###############################
# Cache an error with try..except
# Note: options is the string of option letters that the script wants to recognize, with
# options that require an argument followed by a colon (':') i.e. -i fileName
# http://www.cyberciti.biz/faq/python-command-line-arguments-argv-example/  got the code below from here.
# try:
#     myopts, args = getopt.getopt(sys.argv[1:],"r:g:p:j:i:q:")
# except getopt.GetoptError as e:
#     print (str(e))
#     print("Usage: %s -r 3 -g 100 -p 50 -j 10 -i 0 -n 13" % sys.argv[0])
#     sys.exit(2)
#
# for o, a in myopts:
#     if o == '-r':
#         robot_num= int(a)
#     elif o == '-g':
#         NGEN=int(a)
#     elif o == '-p':
#         POP_SIZE = int(a)
#     elif o == '-j':         #jobs are not implemented right now. will do this sometime else.
#         Jobs = int(a)
#     elif o == '-i':
#         iteration = int(a)
#     elif o == '-q':         # jobs after the addition in jobs. the jobs gives previous jobs, could be useful for folder addressing
#         Jobs_now = int(a)  # jobs now gives the current status of the jobs.



# reading the old jobs dictionary generated in GA_solo using reader_classbased_dict, this remain original and is never altered
# reading the old jobs dictionary generated in GA_solo using reader_classbased_dict, this remain original and is never altered
job_dict = pickle.load(open(FileString + "job_dict.pkl", 'rb'))
sub_job_dict = pickle.load(open(FileString + 'sub_job_dict.pkl', 'rb'))  #again remains original and is never altered by followup
dist_matrix = pickle.load(open(FileString + 'dist_matrix.pkl', 'rb'))

left_over_tasks = parameters['left_over_tasks']     #initiated in GA_solo then edited on each iteration by execution_Followup
# number_to_release = random.randint(0,len(left_over_tasks)) #randomly picking the number of tasks to be released
# # if the number os tasks left is less than the size of the window OR the normal tasks are all attempted and there are only left over tasks then release all remaining tasks.
# if len(left_over_tasks) <= parameters['window_size'] or len(sub_job_dict) == len(attempted_tasks)+len(left_over_tasks):
#         number_to_release = len(left_over_tasks)
# new_tasks = left_over_tasks[:number_to_release]
# for task in new_tasks:
#     left_over_tasks.remove(task)
# the sub_job_dict originally contains all the task, in the start I remove a few tasks from it to be intruduced later on
# over here if the tasks (for later introduction) are not to be introduced in this iteration then remove them again from
# sub_job_dict.
for task in left_over_tasks:
    del sub_job_dict[task]
# Tasks which are already attempted by the robots should also be removed from the sub_job_dict
for task in attempted_tasks:
    del sub_job_dict[task]
new_tasks = parameters['new_tasks_released']    #updated in each iteration by Execution_followup

# Fetching the distance matrices
dist_matrix = pickle.load(open(FileString + 'dist_matrix.pkl', 'rb'))
start_matrix = pickle.load(open(FileString + 'start_matrix.pkl', 'rb'))

## this was used for incest prevention
# parameters['percentage_closeness_1_Onwards'] -= parameters['percentage_closeness_1_Onwards']*0.2
# print 'PERCENTAGE CLOSENESS FOR THIS ITERATION', parameters['percentage_closeness_1_Onwards']

# Saving and printing a few things.
pickle.dump(parameters, open(parameters['FileString'] + "parameters.pkl", "wb"))
print 'the sub job dict for the new operation', len(sub_job_dict), sub_job_dict.keys()
print 'these are the new tasks', new_tasks
print 'thse are the tasks which are still left to be done', left_over_tasks

IND_SIZE = len(sub_job_dict)    #what should be the size of each individual in the generation

#initializing this FIFO queue to implement the panelty function in Michalewicz Paper panelty scheme 3.2.4
# used while calling the panelty_adaptive function in penalties.py
last_k_bests = deque(10*[1], 10)    #This checks if the last few (10) best were valid or not
last_k_improve = deque(10*[0], 10)  # This checks if there is an improvement in the last few gens then don't change the penalty value (there is already convergence in the population)

# creating a sub_job_dictionary of tasks that are to be done now (after deleting the tasks already attempted).
# sub_job_dict_followup = copy.deepcopy(sub_job_dict)
# for entry in attempted_tasks:
#     if entry in new_tasks:
#         del sub_job_dict_followup[entry]

#putting in the fitness and robots_assign attribute in to the individual
creator.create("FitnessMin", base.Fitness, weights=(-1.0,))
creator.create("Individual", list, fitness=creator.FitnessMin, robots_assign=None)


#initiating the individual (deap format) with previous influence. the content thing brings in the last population of
# the previous run from initPopulation, where as the random.sample part randomly initializes the jobs that are new.
# from the new list of subjobs which is formed above by subtracting old subjobs from the total list of subjobs.
def initIndividual(icls, content, robot_number):
    ind = icls(content)     #bringing in the records from the previous run (the final population of previous run).
    ind.robots_assign = copy.deepcopy(content.robots_assign) #copying the previous robot assigns as it is
    ind = remove_attempted(ind, attempted_tasks)    #remove the tasks from allocations which are already attempted
    ind = check_failedNnew(ind, failed_robots)  #check if there are failed or new robots which don't have allocations made to them
    ind = add_new_task(ind, new_tasks, failed_robots)    #if there are new tasks introduced in this iteration then add them
    return ind

# Bringing in the older generation influence from the saved file. File name passed as argument.
def initPopulation(pcls, ind_init, filename):
    with open(filename, "r") as pop_file:
        contents = pickle.load(pop_file)
    return pcls(ind_init(c, robot_num) for c in contents)

# This is the initialization where no previous solutions are used only random assignments are made
# Here for the AIS part of initializations
def initIndividual_simple(icls, ind_size, robot_number):
    ind = icls(random.sample(sub_job_dict.keys(), len(sub_job_dict)))
    ind.robots_assign = [0 for _ in xrange(robot_number)] #creating these dummy lists of lists to be send to check_failedNnew
    ind = check_failedNnew(ind, failed_robots) #Since allocations will not be there for any of the robot, calling this routine populates "robots_assign" a fresh
    return ind

def evaluate(individual):
    return hetro_fitnessNpenalty_max_iterative(individual, dist_matrix, robot_num, parameters['robot_solutions_windowed'],5) # for time based fitness function which supports hetro as well as homo team
    # return hetro_fitness_max_2(individual, dist_matrix, robot_num, start_location, parameters['robot_solutions_windowed'])  # for time based fitness function which supports hetro as well as homo team


toolbox = base.Toolbox()
# registering the initializations for individuals using previous populations
toolbox.register("individual_guess", initIndividual, creator.Individual)
toolbox.register("population_guess", initPopulation, list, toolbox.individual_guess, FileString + "population_file.pkl")
#initializing for individuals not using previous populaitons and starting a fresh (AIS)
toolbox.register("individual", initIndividual_simple, creator.Individual, ind_size= IND_SIZE, robot_number=robot_num)
toolbox.register("population", tools.initRepeat, list, toolbox.individual)          #registering how the population shoud look like. And it calls the generation of individuals inside it.
# Registering the evaluation routine.
toolbox.register("evaluate", evaluate)


#registering some stats for recording as the algo goes on
stats = tools.Statistics(key=lambda ind: ind.fitness.values)
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
    logbook = tools.Logbook()  # writing the log book using pickle but not really using it as yet.
    log_file = open(FileString + 'log_file.txt', 'a')

    # full_execution = open('/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/' + 'RoSTAM_Results.txt', 'a')
    full_valid_invalid = open('/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/' + 'invalid.txt', 'a')
    ## these are two files which I introduced for ASF and BSF recording over multiple runs. The files will then later be
    ## compiled into the average ASF and average BSF version using the file_consolidation.py file
    # BSF = open('/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/' + 'BSF_followup' + str(len(attempted_tasks)) + '.txt', "a")
    # ASF = open('/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/' + 'ASF_followup' + str(len(attempted_tasks)) + '.txt', "a")

    # Reading the previous best which is technically the solution which was followed
    with open(FileString+ 'best_solution.pkl', "r") as pop_file:
        last_best = pickle.load(pop_file)
    print '***********************************************************************************'
    print 'This is the last best solution from which I read the starting locations', last_best


    print 'these are the starting locations', start_location #start locations are created and extracted in Execution_followup
    # Evaluating the pop of solution after deletion of already attempted tasks and addition of any new introduced tasks
    pop, fitness = init_random_penalty()    #performing poorer than init_random_repair
    for ind, fit in zip(pop, fitness):      #assigning the fitness to corresponding individuals
        ind.fitness.values = fit,

    #getting the solution file and the statistics file ready for writing on them later on
    solution_file = open(FileString + str(NGEN) + 'G_' + str(POP_SIZE) + 'P_' + str(len(new_tasks)) + 'J_' + str(robot_num) + 'R_' + str(iteration) + 'I_jobs_solutions.txt', 'a')
    solution_file.seek(0)
    solution_file.truncate()
    stats_file = open(FileString + str(NGEN) + 'G_' + str(POP_SIZE) + 'P_' + str(len(new_tasks)) + 'J_' + str(robot_num) + 'R_' + str(iteration) + 'I_jobs_stats.txt', 'a')
    stats_file.seek(0)
    stats_file.truncate()

    #The generation count and the fitest solution thing is for the while loop
    generation_count = 0
    fitest_solution = 5000
    penalty_value = 1       #This is the starting penalty value. The algorithm will adjust its penalty automatically
    best_valid_ind = pop[0]     #This variable holds the best valid solution found throughout the runs.

    #Running the EA generations
    for g in range(NGEN):

        ## AIS based new solution injection for adding some diversity into the population from previous iteration
        ## This you wouldn't find in GA_solo as that already starts with a fresh population.
        # if g == 0:
        #     pop = Selection.truncation(pop, 80)  # dropping the worst 5 solutions
        #     pop_AIS = toolbox.population(20)  # generating 5 random solutions
        #     for i, ind in enumerate(pop_AIS):  # for every new random ind check the amount of penalty
        #         ind.fitness.values = hetro_fitnessNpenalty_max_iterative(ind, dist_matrix, robot_num, parameters['robot_solutions_windowed'],0),
        #     pop += pop_AIS


        # picking the best individual from the generation and writing it to the solution file
        best = max(pop, key=operator.attrgetter('fitness'))  # This will be max, tried the min thing, it was returning the worst value



        solution_file.write(str(best) + (str(best.robots_assign)) + (str(best.fitness)) + '\n')
        # ****************** RECORDING STATISTICS ****************************
        record = stats.compile(pop)
        minm.append(record['min'])
        maxm.append(record['max'])
        avge.append(record['avg'])
        stde.append(record['std'])
        stats_file.write(str(record) + (str(g)) + '\n')

        ## for every 100th generation pring a few details.
        if generation_count%100 == 0:
            print(record)
            print ('generation count', generation_count)
            if penalties.check_invalid_window(len(best), best, robot_num, parameters['robot_solutions_windowed']):
                print '***************best is invalid*******************'
        # BSF.write(str(record['min'])+'\n')
        # ASF.write(str(record['avg'])+'\n')

        # forming a few new groups for offspring. One is for the normal mu + lambda formation. The other is for crowding
        # offspring = copy.deepcopy(pop)
        offspring_new = []  # used for crowding
        offspring = []      # used for mu + lambda

        ## developing this sorted pop for the stochastic universal parent selection. as a sorted list is passed on to it.
        pop_sorted = copy.deepcopy(pop)
        pop_sorted.sort(key=lambda x: x.fitness.values[0])

        MUTPB = 0.3 #MUTPB_overall / len(best)  # this basically translates to a 50% chance of mutation in the whole chromosome
        # hamming_matrix = population_diversity.compute_hamming_matrix(IND_SIZE, POP_SIZE, pop)

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

            par1 = copy.deepcopy(parent1)
            child1 = copy.deepcopy(parent1)
            par2 = copy.deepcopy(parent2)
            child2 = copy.deepcopy(parent2)

            ## *********************************** Crossovers **********************************************************
            if random.random() < CXPB:
                # child1, child2 = Operators.robot_based_BOX(par1, par2, best_valid_ind)
                # child1, child2 = Operators.BOX(par1, par2, best_valid_ind)
                child1, child2 = Operators.permutation_Carter_2006(par1, par2)
                # child1, child2 = Operators.ordered_crossover(par1, par2)
                # child1, child2 = Operators.TCX_yuan2013(par1, par2)
                # child1, child2 = Operators.permutation_crossover2(par1, par2)
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

            ## inventing this even odd heuristic coz the gene_inverse_mut_II tends to be a bit slow at the end of the show.
            # if g % 2 == 0:        #apply swap mutation for every even numbered generation
            if random.random() < 0.5:
                child1 = Operators.gene_swap_mut(child1, MUTPB)
                child2 = Operators.gene_swap_mut(child2, MUTPB)
            else:               # For each odd generation apply inverse mutation
            #     ## child1 = Operators.gene_inverse_mut_II(child1, MUTPB, g + 1)
            #     ## child2 = Operators.gene_inverse_mut_II(child2, MUTPB, g + 1)
                child1 = Operators.gene_inverse_mut(child1, MUTPB)
                child2 = Operators.gene_inverse_mut(child2, MUTPB)
                # child1 = Operators.gene_insert_mut(child1, MUTPB)
                # child2 = Operators.gene_insert_mut(child2, MUTPB)
            ##**********************************************************************************************************


            ## This portion basically makes the children feasible after applying the operations.
            # if (generation_count%100) == 0:       # make the children feasible for every 100th generation

            # if (generation_count == 0):  # make the children feasible for only the first generation
            #     child1 = ind_valid_random(len(child1), child1, robot_num)
            #     child2 = ind_valid_random(len(child2), child2, robot_num)

            ##*********************************** Penalty functions ***************************************************
            ## have developed a combined function for fitness and penalty thats why these are no longer needed.
            # penalty1 = penalties.penalty_dynamic(len(child1), child1, robot_num, generation_count)
            # penalty2 = penalties.penalty_dynamic(len(child2), child2, robot_num, generation_count)

            ## This penalty_fix function is used in all the population initialization routines where ever the penalty is applicable.
            # penalty1 = penalties.penalty_fix(len(child1), child1, robot_num, generation_count)
            # penalty2 = penalties.penalty_fix(len(child2), child2, robot_num, generation_count)

            # Altering the penalty_fix in the window operation to cater for tasks attempted in one window not
            # attempted in the 2nd window.
            # penalty1 = penalties.penalty_fix_window(len(child1), child1, robot_num, generation_count, parameters['robot_solutions_windowed'])
            # penalty2 = penalties.penalty_fix_window(len(child2), child2, robot_num, generation_count, parameters['robot_solutions_windowed'])

            # penalty1 = penalties.penalty_adaptive(len(child1), child1, robot_num, generation_count, last_k_bests)
            # penalty2 = penalties.penalty_adaptive(len(child2), child2, robot_num, generation_count, last_k_bests)
            # print 'child penalties:', penalty1, penalty2

            ## ********************************** Fitness evaluations and allocations *********************************
            # fitness1 = evaluate(child1)
            # fitness2 = evaluate(child2)
            # print 'fitness without penalty:', fitness1, fitness2
            child1.fitness.values = hetro_fitnessNpenalty_max_iterative(child1, dist_matrix, robot_num, parameters['robot_solutions_windowed'], penalty_value),
            child2.fitness.values = hetro_fitnessNpenalty_max_iterative(child2, dist_matrix, robot_num, parameters['robot_solutions_windowed'], penalty_value),

            if i % 10 == 0:      #Since the i is looping with step size 2, half of the population will go for mu + lambda
                combined_list = [parent1, parent2, child1, child2]  #create a list of parent and children for competiton
                min_fit_ind = min(combined_list, key=operator.attrgetter('fitness.values'))
                offspring_new.append(min_fit_ind)   #find the best individual out of the 4, it will survive
                combined_list.remove(min_fit_ind)
                frequency = [0] * (IND_SIZE)  # measure how many times a particular gnene is repeated within the pop
                # x, y, z = (population_diversity.hamming_dist(min_fit_ind, v, frequency) for v in combined_list)
                minimum_dist = max(combined_list,
                                   key=lambda x: population_diversity.hamming_dist(x, min_fit_ind, frequency)) ##find the one farthest from the best
                ## other distance measures
                # minimum_dist = min(combined_list,
                #                    key=lambda x: population_diversity.edit_dist(x, min_fit_ind))
                # minimum_dist = max(combined_list,
                #                    key=lambda x: population_diversity.seq_match_ratio(x, min_fit_ind))
                # minimum_dist = max(combined_list,
                #                    key=lambda x: population_diversity.seq_match_ratio_full(x, min_fit_ind))
                # print population_diversity.hamming_dist(minimum_dist, min_fit_ind, frequency)
                offspring_new.append(minimum_dist)  # the most dissimilar from the best also survives to the next generation
            else:               #rest of the population will go for crowding. (correlative family crowding)
                offspring.append(child1)
                offspring.append(child2)



            ## these are other schemes from crowding. competing between parents and child (individually) to figure out which would survive
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

            ## Putting them into the pool overwriting the existing members
            # offspring[i] = child1
            # offspring[i+1] = child2         #if you get an index out of range error at this point its coz of odd numbered for loop not matching your allowed number of solutions

        # ************************* SURVIVAL SELECTION *********************
        # pop[:] = offspring_new    # The population is entirely replaced by the offspring
        # for the mu + lambda scheme create a combined pool and undergo survival selection
        for ind in pop:
            ind.fitness.values = hetro_fitnessNpenalty_max_iterative(ind, dist_matrix, robot_num, parameters['robot_solutions_windowed'],penalty_value),
        combined_pop = copy.deepcopy(pop + offspring)

        # ************************* Diversity Measures Fitness Sharing *********************
        ## if implementing fitness sharing then this is where sharing should be applied to the fitter individuals
        # This is the part I added at the time of GA testing Where this computes the Hamming distance b/w chromosomes
        # hamming_matrix = population_diversity.compute_hamming_matrix(IND_SIZE, POP_SIZE, combined_pop)
        # combined_pop = population_diversity.fitness_sharing(hamming_matrix, combined_pop)

        # ************************* SURVIVAL SELECTION *********************
        pop = Selection.truncation(combined_pop, int(1))  # picking the best as an elite
        # for ind in pop:
        #     combined_pop.remove(ind)
        pop += Selection.Binary_Tournament(combined_pop, int(POP_SIZE - 1)-len(offspring_new), 2)  # * (1-Elitest_percentage)), 2)
        # pop += Selection.roulette_select_FPS(combined_pop, int(POP_SIZE -1))#* (1-Elitest_    percentage)))

        ## merging the population portion implemented through crowding with the population which has already undergone survival selection
        # pop = Selection.truncation(pop, len(offspring_new))
        pop += offspring_new
        best = max(pop, key=operator.attrgetter('fitness'))  # This will be max, tried the min thing, it was returning the worst value

        # ************************************* Adaptive Penalty ********************************************************
        ## Since I am implementing a variable penalty function so I don't want the best solution found so far to be lost
        ## This is where at the end of each generation we check if there is a new valid best solution and save it.
        ## Also update the arrays which are maintaining the record for the previous 10 generations convergence
        if not penalties.check_invalid_window(len(best), best, robot_num, parameters['robot_solutions_windowed']) and best.fitness.values[0] < best_valid_ind.fitness.values[0]:
            best_valid_ind = copy.deepcopy(best)      #best_valid_ind is the gbest. am maintaining coz the iteration scheme can loose it (the structure is designed so)
            #print best is best_valid_ind
            last_k_improve.appendleft(1)    #Since there is improvement from the previous run so append 1 to the improvement list
        else:       #if there is no improvement here from the previous run then append a 0 to the improvement list
            last_k_improve.appendleft(0)


        ## This portion is for recording the FIFO list initiated at top for penalty_adaptive function
        if penalties.check_invalid_window(len(best), best, robot_num, parameters['robot_solutions_windowed']):
            # print '***************best is invalid*******************
            last_k_bests.appendleft(1)  # if the best is invalid in this gen then append 1
        else:
            last_k_bests.appendleft(0)  # else append a 0
        if g%10 == 0:   # at the end of every 10 generations
            if last_k_bests.count(1) == 10 and penalty_value < 20:  # if all previous 10 have been invalid then increase penalty
                penalty_value += penalty_value * .15
                # print 'penalty value changed to', penalty_value
            elif last_k_bests.count(0) == 10:  # and last_k_improve.count(0) == 10: #and if all have been valid then decrease penalty
                penalty_value -= penalty_value * .15
                # print 'penalty value changed to', penalty_value

        # # ***************************************** AIS based Population diversity measure ******************************
        if g % 10 == 0:
            pop = Selection.truncation(pop, 80)  # dropping the worst 5 solutions
            pop_AIS = toolbox.population(20)  # generating 5 random solutions
            for i, ind in enumerate(pop_AIS):  # for every new random ind check the amount of penalty
                ind.fitness.values = hetro_fitnessNpenalty_max_iterative(ind, dist_matrix, robot_num, parameters['robot_solutions_windowed'],0),
            pop += pop_AIS


        logbook.record(gen=0, **record)
        generation_count += 1
        # print('generation count', generation_count)

    ##********************************************* Record Keeping *****************************************************
    # writing the last best value in the best value record.
    best = best_valid_ind
    solution_file.write((str(best)) + (str(best.robots_assign)) + (str(best.fitness)) + '\n')
    # solution_file.write(str(hetro_fitness_max_2_list(best, dist_matrix, robot_num, start_location)))
    stats_file.close()
    solution_file.close()
    iteration_solution_file = open(FileString + 'iteration_solution', 'a')
    print('*************************This is the fitness for the final solution', best.fitness)
    iteration_solution_file.writelines(str(best.fitness.values[0]) + '\n')
    iteration_solution_file.close()
    solution_file_ind = open(FileString + 'solution_individual.txt', "wb")
    start = end = 0
    for robot in best.robots_assign:
        end += robot
        solution_file_ind.write(str(best[start:end]) + '\n')
        print (str(best[start:end]) + '\n')
        start += robot

    solution_file_ind.write(str(best.fitness) + str(best.robots_assign))
    solution_file_ind.write(str(hetro_fitness_max_iterative_list(best, dist_matrix, robot_num, parameters['robot_solutions_windowed'])))
    solution_file_ind.close()
    solution_file2 = open(FileString + 'solution.txt', "wb")
    solution_file2.write((str(best)) + "\n" + (str(best.robots_assign)) + "\n" + (str(best.fitness)) + "\n")
    solution_file2.write('left over tasks' + str(left_over_tasks))
    solution_file2.close()
    speed_file = open(FileString + 'speed.txt', "wb")
    speed_file.write(str(speed_matrix))
    speed_file.close()

    best_fitness = str(best.fitness)
    best_fitness = best_fitness.replace(")", "")
    best_fitness = best_fitness.replace("(", "")
    best_fitness = best_fitness.replace(",", "")
    # full_execution.write(best_fitness + '\n')  # making a single file for all the results
    # full_execution.close()

    # keeping a record if the best of last generation was a valid chromo or not.
    if penalties.check_invalid_window(len(best_valid_ind), best_valid_ind, robot_num, parameters['robot_solutions_windowed']):
        full_valid_invalid.write('invalid' + '\n')
    else:
        full_valid_invalid.write('valid' + '\n')

    # BSF.close()
    # ASF.close()
    pickle.dump(logbook, log_file)



    # Adding this in the TA phase of things to save the last population, to be
    # started from for next run of EA when new jobs arrive.
    # pop_file = open(FileString + "followup/" + 'population_file.pkl', 'wb')
    pop_file = open(FileString + 'population_file.pkl', 'wb')
    pickle.dump(pop, pop_file)
    pop_file.close()

    # The best solution also saved for the same purpose so that on the next run we know what was the solution previously followed.
    best_file = open(FileString + 'best_solution.pkl', 'wb')
    pickle.dump(best, best_file)
    best_file.close()
    # *******************************************************************************************************************
    plot_TSP.graph_plot(minm, avge, off_avge, off_min, generation_count, Jobs, robot_num, NGEN, POP_SIZE,iteration)  # ASF and BSF curves
    # plot_TSP.main(FileString, (map_dim * -1) - 2, map_dim + 2, 2)  # plotting the route map for the final solution
    # auction_traderbots.main(FileString, robot_num, Jobs, speed_matrix)    # running the auction based schemes. (for iterative execution).


    # ****************************** GRAPH PLOTTING ****************************
    # fig = plt.figure(figsize=(20, 20))
    # # x = list(range(0,NGEN))
    # x = list(range(0, generation_count))
    # plt.plot(x, minm, '-g', label='Best so Far(Minimum)')
    # # plt.plot(x,maxm, 'r', label='Maximum')
    # plt.plot(x, avge, 'b', label='Average')
    # plt.grid(True)
    # plt.legend(loc='upper left', fontsize=20)
    # plt.title("Graph for" + str(len(new_tasks)) + 'Jobs ' + str(robot_num) + 'robots', fontsize=40)
    # plt.ylabel('Fitness (minimum)', fontsize=30)
    # # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # # this is a huge jugaar, m 100% sure that there are better ways of adjusting the tick size and gridding.
    # # m just in a hurry to plot my graphs
    # plt.xticks(range(0, generation_count, 100), fontsize=20)
    # plt.yticks(range(int(min(avge)) - 10, int(max(avge)) + 10, 5), fontsize=20)
    # # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # plt.xlabel('Generation', fontsize=30)
    #
    # # for tick in fig.xaxis.get_major_ticks():
    # #     tick.label.set_fontsize(14)
    # #     specify integer or one of preset strings, e.g.
    # #     tick.label.set_fontsize('x-small')
    # # tick.label.set_rotation(
    # #     'vertical')
    # fig.savefig(FileString + str(NGEN) + 'G_' + str(POP_SIZE) + 'P_' + str(len(new_tasks)) + 'J_' + str(robot_num) + 'R.png')
    # # plt.show()
    # plt.close(fig)
    # # pickle.dump(logbook, log_file)
    # #
    # # # Comment the kill stuff so that gazebo runs throughout all the iterations (for iterative operations).
    # os.system("killall roslaunch")
    # os.system("killall python")
    # time.sleep(10)



# this initializes the robot_assign part (2nd part of the 2part chromosome) for an indvidual chromosome.One case at a time
#  this distributes the jobs evenly between all the robots. if 15 jobs and 3 robots then 5,5,5 will be the initialization
# This one does it for one solution at a time. The init_assignment routines below does it for the whole population in a single go.
def ind_init_assignment(IND_SIZE, IND,  robot_num):
    fraction = IND_SIZE / float(robot_num)
    IND.robots_assign = [int(fraction)] * robot_num
    ## The way I have assigned tasks for ST-SR-IA things this part of assigning equal capacities to robots
    ## has almost become useless. But still keeping this part as it might be useful in the coming problems
    if (fraction).is_integer():
        None
    else:  # if not a whole number then assign the remainder number one at a time to each robot.
        remainder = IND_SIZE % robot_num
        for j in range(remainder):
            IND.robots_assign[j] += 1


# 2nd part of the chromosome initialization with random numbers.
# the above routine was giving even distributions between robots
def init_assignment_random(IND_SIZE, IND, robot_num):
    failed_robots_count = failed_robots.count(0) #counting the number of failed robots.
    local_robot_assign = [None]*(robot_num - failed_robots_count)
    tasks_left = IND_SIZE  # keeping track of how many tasks are left to be assigned
    #This is the dummy allocation "local_robot_assign"
    # this will only make allocation to active robots but wouldn't take care of their ids
    for i in range(len(local_robot_assign)):
        if i == len(local_robot_assign)-1:  # if its the last robot then allocate all the left over tasks
            local_robot_assign[i] = tasks_left
        else:
            local_robot_assign[i] = random.randint(0,tasks_left)
            tasks_left -= local_robot_assign[i]
    index = 0
    # Now transferring the dummy allocation to the real robots while keeping track of failed robots.
    # simply placing the already allocated tasks at the right ids while keeping the failed robot allocations 0
    for i in range(robot_num):
        if failed_robots[i] == 1:   #if i corresponds to an active robot.
            IND.robots_assign[i] = local_robot_assign[index]
            index += 1
        else:   # if "i" corresponds to failed robot.
            IND.robots_assign[i] = 0


    # for id in range(robot_num):     # assignments are done for every robot
    #     if id == robot_num-1:       # if its the last robot to be assigned then assign the left over to it
    #         IND.robots_assign[id] = tasks_left
    #     else:                               # for all the robots except the last one.
    #         value = random.randint(0,tasks_left)        # pick a random number from the range of left over tasks
    #         IND.robots_assign[id] = value               # assign it to the current robot
    #         tasks_left -= value                         # subtract the assigned value from the leftover tasks number

#  #*********************************** Time based Fitness functions  **************************************************

# This function is same as hetro_fitness_max used in old days. This one combines the old solution from the iterative version
# to evaluate the complete fitness. This function also evaluates
def hetro_fitnessNpenalty_max_iterative(chromo, cst_mtrx, rbt_count, robot_solutions_windowed, penalty_value):
    # if previous iterations had lesser number of robots and the new one has more
    if len(robot_solutions_windowed) < rbt_count:
        # then add an empty list into already attempted for each robot.
        for i in range(abs(len(robot_solutions_windowed)-rbt_count)):
            robot_solutions_windowed.append([])
    start_matrix = cst_mtrx[0][:]   # initializing the start matrix, which is just the first line of cost matrix
    rbt_job = [[None]] * rbt_count  # initializing a list of list for the individual robot jobs for each robot
    fitness_list = []  # a list of fitness values to return the max from
    fitness = 0 #dummy fitness and penalty values
    penalty = 0
    previous_task = None
    task = None
    start = end = 0
    for i in range(rbt_count):        #for each and every robot I read the tour for that robot
        end += chromo.robots_assign[i]
        rbt_job[i] = chromo[start:end]
        rbt_job[i] = robot_solutions_windowed[i]+rbt_job[i] # append the previously attempted tasks by this robot
        rbt_job[i] = [int(j.split(",")[0]) for j in rbt_job[i]] #get rid of all the commas and sub jobs only the job ids
        start = end
        already_counted = []    #keeping track of already counted for tasks while computing the penalty, so that do not apply penalty for a task multiple times
        for j in range(len(rbt_job[i])): #for each task in the robot tour
            task = rbt_job[i][j]
            if j == 0:  #if its the first job then find the distance from depot to location.
                fitness += start_matrix[task]/speed_matrix[i] #coz I have appended this extra zero for home-home traveling  +1
            else:   #if not the first task then distance from previous task to next task
                fitness += cst_mtrx[previous_task][task]/speed_matrix[i] #+1 +1
            previous_task = task
            ## This portion handles the penalty comutation against current task
            count = rbt_job[i].count(task)
            if count > 1 and (task not in already_counted): # if task attempted more than once and isn't already counted for then apply penalty
                penalty += (count -1) * penalty_value
            already_counted.append(task)
        #
        # if len(rbt_job[i]) > 0:   # checking if this particular robot has no jobs then there is no point completing the tour
        #     fitness += start_matrix[task]/speed_matrix[i]        #This is for completing the tour. from home to home  +1
        fitness_list.append(fitness)                            #saving each robots fitness value into the list
        # for a normal distance based fitness just return the sum of this list with speed values set to 1 for all robots
        fitness = 0
        # print fitness_list
    fitness = max(fitness_list)     #since the same penalty will be applied to each robots tour the realative max min
    return fitness+penalty          # will remain the same so adding penalty at the end

# ********************this should never be used for evaluation purpose.**********************************
# Planning this function to be the same as hetro_fitnessNpenalty_max_iterative but with returning the list of all robots executions.
# for the time being I am not embedding the penalty part in here, coz I usually use this function at the end of the run
def hetro_fitness_max_iterative_list(chromo, cst_mtrx, rbt_count, robot_solutions_windowed):
    # if previous iterations had lesser number of robots and the new one has more
    if len(robot_solutions_windowed) < rbt_count:
        for i in range(abs(len(robot_solutions_windowed)-rbt_count)):
            robot_solutions_windowed.append([])     #then add an empty list into already attempted for each robot.
    start_matrix = cst_mtrx[0][:]   # initializing the start matrix, which is just the first line of cost matrix
    rbt_job = [[None]] * rbt_count  # initializing a list of list for the individual robot jobs for each robot
    fitness_list = []  # a list of fitness values to return the max from
    fitness = 0
    previous_task = None
    task = None
    start = end = 0
    for i in range(rbt_count):        #for each and every robot
        end += chromo.robots_assign[i]
        rbt_job[i] = chromo[start:end]
        rbt_job[i] = robot_solutions_windowed[i]+rbt_job[i] #appending the already attempted tasks with the new solution
        rbt_job[i] = [int(j.split(",")[0]) for j in rbt_job[i]] #stripping off the subtasks and the commas, only the task ids are left
        start = end
        for j in range(len(rbt_job[i])):    #for each task in this robots tour
            task = rbt_job[i][j]
            if j == 0:                      #if its the first job then read the starting location from start_location and then find the distance.
                fitness += start_matrix[task]/speed_matrix[i] #coz I have appended this extra zero for home-home traveling  +1
            else:
                fitness += cst_mtrx[previous_task][task]/speed_matrix[i] #+1 +1
            previous_task = task
        if len(rbt_job[i]) > 0:   # checking if this particular robot has no jobs then there is no point completing the tour
            fitness += start_matrix[task]/speed_matrix[i]        #This is for completing the tour. from home to home  +1
        fitness_list.append(fitness)                            #saving each robots fitness value into the list
        # for a normal distance based fitness just return the sum of this list with speed values set to 1 for all robots
        fitness = 0
    return fitness_list

# #very minor changes into the hetro_fitness function, now saving each robots total time into a list as fitness and returning
# # the max out of it. This is coz sir asked me to treat the maximum time consumed by a robot (distance/speed) as my fitness
# def hetro_fitness_max(chromo, cst_mtrx, rbt_count):
#     start_matrix = cst_mtrx[0][:]             # initializing the start matrix, which is just the first line of cost matrix
#     rbt_job = [[None]] * rbt_count  # initializing a list of list for the individual robot jobs for each robot
#     fitness_list = []  # a list of fitness values to return the max from
#     fitness = 0
#     previous_task = task = None
#     start = end = 0
#     for i in range(rbt_count):  # for each and every robot
#         end += chromo.robots_assign[i]
#         rbt_job[i] = chromo[start:end]
#         start = end
#         for j in range(len(rbt_job[i])):
#             task_id = rbt_job[i][j].split(',')
#             task = int(task_id[0])
#             if j == 0:                      #if its the first job then start from base and go till first location
#                 fitness += start_matrix[task]/speed_matrix[i] #coz I have appended this extra zero for home-home traveling  +1
#             else:
#                 fitness += cst_mtrx[previous_task][task]/speed_matrix[i] #+1 +1
#             previous_task = task
#         if len(rbt_job[i]) > 0:   # checking if this particular robot has no jobs then there is no point completing the tour
#             fitness += start_matrix[task]/speed_matrix[i]        #This is for completing the tour. from home to home  +1
#         fitness_list.append(fitness)                            #saving each robots fitness value into the list
#         # for a normal distance based fitness just return the sum of this list with speed values set to 1 for all robots
#         fitness = 0
#     return max(fitness_list)                                    #returning the max

# # Planning this function to be the same as hetro_fitness_max but with variable starting locations for the robots.
# def hetro_fitness_max_2(chromo, cst_mtrx, rbt_count, start_locations, robot_solutions_windowed):
#     for i in range(len(start_locations)):
#         start_locations[i] = 0
#     if len(robot_solutions_windowed) < rbt_count:  # if the already attempted tasks had lesser number of robots and the new one has more
#         for i in range(abs(len(robot_solutions_windowed)-rbt_count)):  #then add an empty list into already attempted for each robot.
#             robot_solutions_windowed.append([])
#     load_cofficient = parameters['load_cofficient']
#     start_matrix = cst_mtrx[0][:]             # initializing the start matrix, which is just the first line of cost matrix
#
#     rbt_job = [[None]] * rbt_count  # initializing a list of list for the individual robot jobs for each robot
#
#     fitness_list = []  # a list of fitness values to return the max from
#     fitness = 0
#     previous_task = None
#     task = None
#     start = end = 0
#     for i in range(rbt_count):        #for each and every robot
#         end += chromo.robots_assign[i]
#         rbt_job[i] = chromo[start:end]
#         rbt_job[i] = robot_solutions_windowed[i]+rbt_job[i]
#         start = end
#         for j in range(len(rbt_job[i])):
#             task_id = rbt_job[i][j].split(',')
#             task = int(task_id[0])
#             if j == 0:                      #if its the first job then read the starting location from start_location and then find the distance.
#                 fitness += cst_mtrx[start_locations[i]][task]/(speed_matrix[i]*load_cofficient[i]) #So basically the previous task is the starting task read from start_location and till the first task of this particular robot.
#             else:
#                 fitness += cst_mtrx[previous_task][task]/(speed_matrix[i]*load_cofficient[i]) #+1 +1
#             previous_task = task
#         if len(rbt_job[i]) > 0:   # checking if this particular robot has no jobs then there is no point completing the tour
#             fitness += start_matrix[task]/(speed_matrix[i]*load_cofficient[i])        #This is for completing the tour. from home to home  +1
#         fitness_list.append(fitness)                            #saving each robots fitness value into the list
#         # for a normal distance based fitness just return the sum of this list with speed values set to 1 for all robots
#         fitness = 0
#         # print fitness_list
#     return max(fitness_list)                                    #returning the max
#
# # Planning this function to be the same as hetro_fitness_max but with variable starting locations for the robots.
# def hetro_fitness_max_2_list(chromo, cst_mtrx, rbt_count, start_locations):
#     load_cofficient = parameters['load_cofficient']
#     start_matrix = cst_mtrx[0][:]             # initializing the start matrix, which is just the first line of cost matrix
#     rbt_job = [[None]] * rbt_count  # initializing a list of list for the individual robot jobs for each robot
#     fitness_list = []  # a list of fitness values to return the max from
#     fitness = 0
#     previous_task = None
#     task = None
#     start = end = 0
#     for i in range(rbt_count):        #for each and every robot
#         end += chromo.robots_assign[i]
#         rbt_job[i] = chromo[start:end]
#         start = end
#         for j in range(len(rbt_job[i])):
#             task_id = rbt_job[i][j].split(',')
#             task = int(task_id[0])
#             if j == 0:                      #if its the first job then read the starting location from start_location and then find the distance.
#                 # print 'Since this is the first task so computing distance between', start_location[i], 'and', task, 'with distance', cst_mtrx[start_locations[i]][task]/speed_matrix[i]
#                 fitness += cst_mtrx[start_locations[i]][task]/(speed_matrix[i]*load_cofficient[i]) #So basically the previous task is the starting task read from start_location and till the first task of this particular robot.
#             else:
#                 # print 'for all other tasks computing the distance between', previous_task, 'and', task, 'with distance', cst_mtrx[previous_task][task]/speed_matrix[i]
#                 fitness += cst_mtrx[previous_task][task]/(speed_matrix[i]*load_cofficient[i]) #+1 +1
#             previous_task = task
#         if len(rbt_job[i]) > 0:   # checking if this particular robot has no jobs then there is no point completing the tour
#             # print 'when the tasks are complete, time to go home from', task, 'with distance',  start_matrix[task]/speed_matrix[i]
#             fitness += start_matrix[task]/(speed_matrix[i]*load_cofficient[i])        #This is for completing the tour. from home to home  +1
#         fitness_list.append(fitness)                            #saving each robots fitness value into the list
#         # for a normal distance based fitness just return the sum of this list with speed values set to 1 for all robots
#         fitness = 0
#         print fitness_list
#     return fitness_list                                    #returning the max
#
# # Exactly same as hetro_fitness_max, will just return the whole list rather than just the max. making to write this list into the saved .txt solution file at the end
# def hetro_fitness_list(chromo, cst_mtrx, rbt_count):
#     start_matrix = cst_mtrx[0][:]             # initializing the start matrix, which is just the first line of cost matrix
#     rbt_job = [[None]] * rbt_count  # initializing a list of list for the individual robot jobs for each robot
#     fitness_list = []  # a list of fitness values to return the max from
#     fitness = 0
#     previous_task = task = None
#     start = end = 0
#     for i in range(rbt_count):        #for each and every robot
#         end += chromo.robots_assign[i]
#         rbt_job[i] = chromo[start:end]
#         start = end
#         for j in range(len(rbt_job[i])):
#             task_id = rbt_job[i][j].split(',')
#             task = int(task_id[0])
#             if j == 0:                      #if its the first job then start from base and go till first location
#                 fitness += start_matrix[task]/speed_matrix[i] #coz I have appended this extra zero for home-home traveling  +1
#             else:
#                 fitness += cst_mtrx[previous_task][task]/speed_matrix[i] #+1 +1
#             previous_task = task
#         if len(rbt_job[i]) > 0:   # checking if this particular robot has no jobs then there is no point completing the tour
#             fitness += start_matrix[task]/speed_matrix[i]        #This is for completing the tour. from home to home  +1
#         fitness_list.append(fitness)                            #saving each robots fitness value into the list
#         # for a normal distance based fitness just return the sum of this list with speed values set to 1 for all robots
#         fitness = 0
#     return fitness_list                                    #returning the max

#  #********************************************************************************************************************

# This routine is for ST-MR representation. would locate repetition of a primary job in a robot's substring
# If a single robot is assigned more than one instance of a single job, then assign that instance to another robot
# This one works for the initial population and after crossover or mutation
# The structure of job representation should be "1,2" type otherwise this routine fails.
# this routine looks at the input chromo and decides if its valid or not.
# looks at each individual robot plan one by one if there is repetation of tasks then assigns the repetation to
# another robot (randomly) who doesn't have that particular task assigned already.
# def ind_valid_random(ind_size, ind, rbt_count):
#     invalid = True              # Simple flag used all along to check where the loops are returning from
#     rbt_job = [[] for i in range(rbt_count)]  # initializing a list of list for the individual robot jobs for each robot
#     start = end = 0
#     for i in range(rbt_count):      # populating each robots sub tour
#         end += ind.robots_assign[i]
#         rbt_job[i] = ind[start:end]
#         start = end
#
#     while(invalid):
#         invalid = False
#         for i in range(len(rbt_job)):        #for each and every robot
#             temp = "-".join(rbt_job[i])  # creating a string out of the job list for a single robot
#             temp = "-" + temp  # adding a - to the first task, as the loop on top misses first task
#             for job in (rbt_job[i]):         # for every single job of robot 'i'
#                 job2 = copy.deepcopy(job)    # create a copy of job for later alterations
#                 data = job.split(",")
#                 if (temp.count("-" + data[0] + ",")) > 1:     #counting the occurances of jobs
#                     adjusted = True
#                     while(adjusted):
#                         rand_id = random.randint(0, len(rbt_job)-1)
#                         temp2 = "-".join(rbt_job[rand_id])
#                         temp2 = "-" + temp2
#                         if (temp2.count("-" + data[0] + ",")) == 0: # if have zero instances of task in question then assign
#                             adjusted = False
#                             ind.robots_assign[rand_id] += 1           # increase the robot_assign of this robot by 1
#                             ind.remove(job)                     # remove the job from the chromosome
#                             rbt_job[i].remove(job)              # remove job from initial robot's sub-tour array
#                             rbt_job[rand_id].insert(0,job)           # add it to new robot's sub-tour array
#                             # if new robot comes after initial robot then insertion is a bit different than the other case
#                             if rand_id > i:   # due to "ind.remove(job)" 2 lines up chromosome's indexing is now a bit jumbled up
#                                 ind.insert(sum(ind.robots_assign[0:rand_id])-1, job)
#                             else:       # if the new robot is before the problem facing robot then the indexing in first
#                                 ind.insert(sum(ind.robots_assign[0:rand_id]), job)    # part of the chromosome isn't really effected.
#                             break
#                     ind.robots_assign[i] -= 1
#                     invalid = True
#                     break
#             if invalid:
#                 break
#     return ind

def read_old():
    with open(FileString + 'population_file.pkl', 'rb') as pop_file:
        pop_old = pickle.load(pop_file)
    return pop_old

    # with open(FileString + "population_file.txt", "rb") as f:
    #     pop_old = f.readlines()
    #     pop_old = [x.strip("[") for x in pop_old]
    #     pop_old = [x.strip("\n") for x in pop_old]
    #     pop_old = [x.strip("]") for x in pop_old]
    #     # pop_old = [x.strip(",") for x in pop_old]
    # population_old = [None] * (len(pop_old))
    # for i in range(len(pop_old)):
    #     population_old[i] = [int(el) for el in pop_old[i].split(',')]
    # return population_old

# This routine initializes the population from previous run evaluates the solutions and returns the population and
# population fitnesses seperately
def init_random_penalty():
    pop = toolbox.population_guess()        # initialization of population from last run of EA
    # total_penalty = [None] * len(pop)
    # for i, ind in enumerate(pop):  # for every crossovered and mutated child check if its a valid solution or not
        # ind_valid_random(len(ind), ind, robot_num) # check if the mutated and crossoverd ind. are valid or not.
        # I am doing this for both ST-SR and ST-MR where it really isn't needed for ST-SR
        # will effect my running time.
        # total_penalty[i] = penalties.penalty_fix_window(len(ind), ind, robot_num,1, parameters['robot_solutions_windowed'])
    # print 'total penalty', total_penalty
    fitness = map(evaluate, pop)   #evaluate each individual of the population by mapping the evaluate function over the whole population
    return pop, fitness#[x + y for x, y in zip(total_penalty, fitness)]

# this routine deletes already attempted tasks from the whole population of solutions once they have been initiated.
# also deletes one unit from particular robot's robots_assign to make sure the plans remain intact.
def remove_attempted(ind, attempted_tasks):
    for task in attempted_tasks:
        if task in ind:
            task_index = ind.index(task)
            # Have to check that to which particular robot this attempted task belongs. Should reduce its allocation
            # by one unit and then at the end delete that particular task from the first part of the chromo as well.
            start = 0
            for i, entry in enumerate(ind.robots_assign):
                if task_index < start+entry:
                    entry -= 1
                    ind.robots_assign[i] = entry
                    break
                else:
                    start += entry
            ind.remove(task)
    return ind


# This is a multipurpose routine. It tests if new robots are being added from previous run? if yes then robots_assign size is increased
# Also tests if there are failed robots. If yes then the robots_assign values are to be altered.
# For alteration it goes for even distribution of tasks to all the remaining robots.
def check_failedNnew(ind, failed_robots):
    # This portion checks if there are new robots to be added into the allocation
    rbt_count = len(failed_robots)
    if len(ind.robots_assign) < rbt_count:  # if the previous solution had lesser number of robots and the new one has more
        for i in range(rbt_count - len(ind.robots_assign)):
            ind.robots_assign.append(0)     # add 0 values to the robots_assign matrix
    # This portion checks if there are any failed robots, and they have something assigned to them from previous solution
    # If nothing is assigned to the failed robots then no changes need to be made to robots_assign
    # if something is assigned to a failed robot then assignments are redone on even distribution basis.
    for i in range(len(ind.robots_assign)):
    #     #This if is inside a for but technically if it has to run it would run only once and give us the right form of distribution
        if (failed_robots[i] == 0 and ind.robots_assign[i] > 0):# or (failed_robots[i] == 1 and ind.robots_assign[i] == 0):  #check if there is a failed robot and something is assigned to it.
            tasks_removed = ind[sum(ind.robots_assign[0:i]):sum(ind.robots_assign[0:i])+ind.robots_assign[i]]
            del(ind[sum(ind.robots_assign[0:i]):sum(ind.robots_assign[0:i])+ind.robots_assign[i]])
            ind.robots_assign[i] = 0
            ind = add_new_task(ind, tasks_removed, failed_robots)

    for i in range(len(ind.robots_assign)):
        if (failed_robots[i] == 1 and ind.robots_assign[i] == 0):
            tasks_left = len(ind)  # keeping track of how many tasks are left to be assigned
            fraction = tasks_left / float(sum(failed_robots))
            #Allocating a flat zero to all the failed robots
            for j in [index for index, value in enumerate(failed_robots) if value == 0]:
                ind.robots_assign[j] = 0
            # Evenly distributing tasks amongst all the active robots. With an Added 1 for early robots
            remainder = len(ind) % sum(failed_robots)
            indexes = random.sample([index for index, value in enumerate(failed_robots) if value == 1], remainder)
            for j in [index for index, value in enumerate(failed_robots) if value == 1]:
                ind.robots_assign[j] = (int(fraction))
                if remainder and j in indexes:
                    ind.robots_assign[j] += 1
                    remainder -= 1
    return ind


# add new tasks (introduced in this run) into the solution. Takes care of not assigning to failed robots
def add_new_task(ind, new_tasks, failed_robots):
    rbt_count = len(failed_robots)
    if len(ind.robots_assign) < rbt_count:  # if the previous solution had lesser number of robots and the new one has more
        for i in range(rbt_count - len(ind.robots_assign)):
            ind.robots_assign.append(0)  # add 0 values to the robots_assign matrix
    for task in new_tasks:      #for each and every task in the new task list
        rand_robo_id = random.choice([index for index, value in enumerate(failed_robots) if value == 1])   #pick a random robot id to assign the new task to
        start = sum(ind.robots_assign[0:rand_robo_id])
        end = sum(ind.robots_assign[0:rand_robo_id])+ind.robots_assign[rand_robo_id]
        rand_location = random.randint(start,end)
        ind.robots_assign[rand_robo_id] += 1                        # increase the robots_assign number of that robo by 1
        ind.insert(rand_location, task) # insert the new task at a random location of the selected robot.
    return ind

if __name__ == "__main__":
    main()
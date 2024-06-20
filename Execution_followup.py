import os
import job_alteration
import pickle
import plot_TSP
import SSI_auction_Sven
import SSI_followup
import compiler
import parsers
import Deletion
import sys
import subprocess
import deap

# parameters = pickle.load(open("/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/parameters.pkl" , 'rb'))




def main():
    os.system('python config.py')  #executing config.py once to make sure that the settings are saved into a pickle file
    # reading the parameters in the pickle file saved by the config.py execution.
    parameters = pickle.load(open("/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/parameters.pkl", 'rb'))
    average_fitness = open('/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/' + 'Avg_Fitness_MultipleRun.txt','a')
    fitness_allrecord = open('/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/' + 'All_record_MultipleRun.txt','a')

    for a in range(50,55,5):        #Tasks
        for i in range(5,26):        # instances
            for j in range(4,5):    # number of runs for each instance for the purpose of averaging
                parameters['jobs'] = a
                parameters['iteration'] = i
                iteration_string = (str(parameters['jobs']) + "/" + str(parameters['iteration']) + "/")
                parameters['Complete_file_String'] = parameters['FileString'] + iteration_string
                pickle.dump(parameters, open(parameters['FileString'] + "parameters.pkl", "wb"))
                window_with_new_tasks()

        #Redaing the output of multiple runs from the RoSTAM_Results file and averaging it.
        #The averaging value is hard coded and must be changed according to the number of iterations run for averaging.
        fitness_file = open('/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/' + 'RoSTAM_Results.txt','r+')
        data = 0
        for line in fitness_file:
            data += float(line)
            fitness_allrecord.write(str(float(line)) + '\n')
        data = data / 15  # There are 3 cases in each task count and each case is run 5 times.
        average_fitness.write(str(data) + '\n')
        print 'the average value for the 15 runs is', data
        fitness_file.seek(0)
        fitness_file.truncate()
    fitness_file.close()
    average_fitness.close
    fitness_allrecord.close()

def iterative_execution():
    # This portion is the iterative player of iterative version of things. There are 3 separate job distribution with
    # their own folders like 50/0 50/1 50/2 (the i loop) each of them runs for 5 times (the j loop) also the SSI are
    # being run both before adding any tasks (like only of 50 tasks) and after the followup after adding the tasks
    # (like on 50+15 tasks) so The thing is on 50 task the normal execution then add the tasks and restart the followup
    # (which starts from old population) and then there is execution of GA_solo again which from start optimized 65 (50+15)

    # The procedure for execution is that in the iterative folder there is a copy of 30 tasks and a copy of 50 tasks,
    # put them (or similarly designed folder inside a simple 30 or 50 folder and start executing. It is important to arrange
    # the files inside these 30_copy or 50_copy folder accordingly coz different codes (GA_solo, deap_followup etc) appraoch
    # different files differently.

    # The results are saved outside the 30 or 50 folder in files RoSTAM_Results.txt and SSI_solution.txt where for RoSTAM
    # the format is that first result is for 30 task GA_Solo, 2nd solution is for 30+.. tasks deap_followup and the 3rd
    # result is for 30+.. task GA_solo complete rerun. For SSI the 1st is for 30 task, 2nd is for 30+.. task followup
    # and 3rd is for SSI rerun for 30+.. tasks.

    os.system('python config.py')  #executing config.py once to make sure that the settings are saved into a pickle file
    # reading the parameters in the pickle file saved by the config.py execution.
    parameters = pickle.load(open("/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/parameters.pkl", 'rb'))



    for i in range(0,3): #3 instances of the task distribution working for. The /0, /1, /2 folders
        # updating the parameter so that the other routines respond accordingly
        parameters['iteration'] = i
        parameters['FileString'] = "/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/"
        iteration_string = (str(parameters['jobs']) + "/" + str(parameters['iteration']) + "/")
        parameters['Complete_file_String'] = parameters['FileString'] + iteration_string
        pickle.dump(parameters, open(parameters['FileString'] + "parameters.pkl", "wb"))

        # multiple runs of a single instance to observe average behavior
        for j in range(0,3):
            os.system("python GA_solo.py")  #solution for the initial job distribution
            SSI_auction_Sven.main()         # SSI solution for initial job distribution
            # if its the first attempt then generate new jobs after running the intitial run i.e. 50+15
            if j == 0:
                pass
                # job_alteration.add_job_simple(parameters['new_jobs'], parameters['new_jobs_type'], parameters['Complete_file_String'], parameters['map_dim'])
                # job_alteration.job_representation_simple(parameters['Complete_file_String'])

            parameters['execution_number'] = j
            pickle.dump(parameters, open(parameters['FileString'] + "parameters.pkl", "wb"))
            os.system("python deap__followup.py")
            # the after followup flag makes the GA_solo and SSI read the new job discriptions from the 'followup' folder inside the 50/0 thing
            parameters['after_followup'] = True
            pickle.dump(parameters, open(parameters['FileString'] + "parameters.pkl", "wb"))
            os.system("python GA_solo.py")
            SSI_followup.main()
            SSI_auction_Sven.main()
            parameters['after_followup'] = False
            pickle.dump(parameters, open(parameters['FileString'] + "parameters.pkl", "wb"))


def GA_solo_only():
    # this one simply excutes the GA_solo code corresponding to the first iteration of any iterative process to come.
    parameters = pickle.load(
        open("/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/parameters.pkl", 'rb'))
    os.system("python GA_solo.py")

# This routine in previous version 12/11/2018 was iterating over the tasks with a fixed window size.
# Now its working with a time based window where dynamic_time tells at what intervals the disturbance will be introduced
def window_with_new_tasks():
    parameters = pickle.load(open("/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/parameters.pkl", 'rb'))
    execution_log = open(parameters['Complete_file_String'] + 'Dynamic_execution_log.txt', 'a')
    execution_log.seek(0)
    execution_log.truncate()

    # dynamic_times = [10, 10, 5, 10, 50000] #Time intervals at which the disturbance will be introduced.
    #first at 10 secs, next at 10 secs after first, next at 5 secs, next at 10 and final at a loooonng time for complete executions.

    #the first iteration should hold all active. after than you can have failed robots. The total count shouldn't go
    # down, can increase or remain same. IF you want to remove a robot permanently then make it 0 for all other iterations
    active_robots = [[1, 1, 1],
                     [1, 0, 1],
                     [1, 0, 1],
                     [1, 1, 0],
                     [1, 1, 0],
                     [1, 1, 0],
                     [0, 1, 1],
                     [0, 1, 1],
                     [0, 1, 1],
                     [0, 1, 1],
                     [1, 1, 0],
                     [1, 1, 0],
                     [1, 1, 0],
                     [1, 1, 0],
                     [1, 1, 1],
                     [1, 1, 1],
                     [1, 1, 1],
                     [1, 1, 1],
                     [1, 1, 1],
                     [1, 1, 1],
                     [1, 1, 1],
                     [1, 1, 1],
                     [1, 1, 1],
                     [1, 1, 1],
                     [1, 1, 1, 1, 1],
                     [1, 1, 1, 1, 1, 1, 1],
                     [1, 0, 1, 1, 1, 1, 1],
                     [1, 0, 1, 1, 1, 0, 1],
                     [1, 1, 1, 1, 1, 0, 1],
                     [1, 1, 1, 1, 1, 0, 0],
                     [1, 1, 1, 1, 1, 0, 0],
                     [1, 1, 1, 1, 1, 0, 0],
                     [1, 1, 1, 1, 1, 0, 0],
                     [1, 1, 1, 1, 1, 0, 0],
                     [1, 1, 1, 1, 1, 0, 0],
                     [1, 1, 1, 1, 1, 0, 0],
                     [1, 1, 1, 1, 1, 0, 0],
                     [1, 1, 1, 1, 1, 0, 0],
                     [1, 1, 1, 1, 1, 0, 0],
                     [1, 1, 1, 1, 1, 0, 0],
                     [1, 1, 1, 1, 1],
                     [1, 1, 1, 1, 1],
                     [1, 1, 1, 1, 1],
                     [1, 1, 1, 1, 1],
                     [1, 1, 1, 1, 1],
                     [1, 1, 1, 1, 1],
                     [1, 1, 1, 1, 1],
                     [1, 1, 1, 1, 1],
                     [1, 1, 1, 1, 1],
                     [1, 1, 1, 1, 1],
                     [1, 1, 1, 1, 1],
                     [1, 1, 1, 1, 1, 1],
                     [1, 1, 1, 0, 1, 1],
                     [1, 1, 1, 0, 1, 1],
                     [1, 1, 1, 1, 0, 1],
                     [1, 1, 1, 1, 0, 0],
                     [1, 1, 1, 1, 0, 1],
                     [1, 1, 1, 1, 1, 0],
                     [1, 1, 1, 1, 1, 0],
                     [1, 1, 1, 1, 1, 0],
                     [1, 1, 1, 1, 1, 1],
                     [1, 1, 1, 1, 1, 1],
                     [1, 1, 1, 1, 1, 1],
                     [1, 1, 1, 1, 1, 1],
                     [1, 1, 1, 1, 1, 1],
                     [1, 1, 1, 1, 1, 1],
                     [1, 1, 1, 1, 1, 1],
                     [1, 1, 1, 1, 1, 1]]

    # *******************************************************************************************
    # percentage of hidden tasks to be released in each iteration
    percentage_to_release = [0.5, 1.0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    # *******************************************************************************************

    iteration_number = 1
    # updating the parameter so that the other routines respond accordingly
    # parameters['iteration'] = 0
    # parameters['FileString'] = "/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/"
    # iteration_string = (str(parameters['jobs']) + "/" + str(parameters['iteration']) + "/")
    # parameters['Complete_file_String'] = parameters['FileString'] + iteration_string
    # pickle.dump(parameters, open(parameters['FileString'] + "parameters.pkl", "wb"))

    execution_log.writelines('INITIAL STARTING LOCATIONS' + str(parameters['start_location']) + '\n')
    execution_log.writelines('ACTIVE ROBOTS 1st RUN' + str(parameters['failed_robots']) + '\n')

    os.system("python GA_solo.py")  # solution for the initial job distribution

    # the after followup flag makes the GA_solo and SSI read the new job discriptions from the 'followup' folder inside the 50/0 thing
    # reading the parameters again before writing them so if they are changed within GA_Solo they wouldn't be overwritten
    parameters = pickle.load(open("/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/parameters.pkl", 'rb'))
    execution_log.writelines('TASKS DROPPED AT FIRST ITERATION FOR DYNAMIC ARRIVAL' + str(parameters['left_over_tasks']) + '\n')
    print 'LEFT OVER TASKS:', parameters['left_over_tasks']
    run_flag = True  # keeping a flag to terminate the while loop.

    open_file = open(parameters['Complete_file_String'] + 'partial_solution_file.txt', 'w')
    open_file.seek(0)
    open_file.truncate()
    open_file.close()


    while run_flag:
        start_locations = parameters['start_location_2']
        parameters['failed_robots'] = active_robots[iteration_number]

        ind_solution = open(parameters['Complete_file_String'] + 'solution_individual.txt', 'r')

        # Checking coz if new robots are added compared to first run then then solutions should be the size of new count
        if iteration_number == 1:   #checking first run
            partial_robot_solutions = [[] for _ in range(int(parameters['robot_num']))]
            # identifying partial robot solutions for each individual window.
        else:
            partial_robot_solutions = [[] for _ in range(int(parameters['robot_num_new']))]
            # identifying partial robot solutions for each individual window.

        ##extracting the window size based partial solutions for each robot from the best solution of last run
        extract_partial_robot_solutions_window_based(ind_solution, iteration_number, partial_robot_solutions, parameters)
        ## extracting the time based partial solutions for each robot from the best solution of last run
        # extract_partial_robot_solutions_time_based(ind_solution, iteration_number, partial_robot_solutions,parameters, dynamic_times[iteration_number-1])

        ## Updating a few parameters
        parameters['robot_num_new'] = len(active_robots[iteration_number])
        parameters['start_location_2'] = [0] * parameters['robot_num_new']  # This is altered in Execution_followup.py before calling deap__followup.py
        parameters['start_location_2'] = extract_start_location(partial_robot_solutions, start_locations, parameters['start_location_2'])

        execution_log.writelines('LEFT OVER TASKS STILL TO BE INTRODUCED' + str(parameters['left_over_tasks']) + '\n')
        execution_log.writelines(('TASKS INTRODUCED IN THIS ITERATION') + str(parameters['new_tasks_released']) + '\n')
        execution_log.writelines('WINDOW SOLUTION FOR ITERATION ' + str(iteration_number) + ' RUN : ' + str(partial_robot_solutions)+ '\n')
        execution_log.writelines('ACTIVE ROBOTS FOR ITERATION:' + str(iteration_number+1) + str(active_robots[iteration_number]) + '\n')
        execution_log.writelines('STARTING LOCATION FOR THIS ITERATION' + str(iteration_number+1) + str(parameters['start_location_2']) + '\n')

        # adding the partial solution to the complete solutions I have been maintaining from outside the while loop
        for element in partial_robot_solutions:
            parameters['attempted_tasks'].extend(element)

        # **********************************This is where new tasks are added from the left over tasks***********************************
        release_percentage = percentage_to_release[iteration_number - 1]
        print 'PERCENTAGE TO RELEASE & ITERATION NUMBER', release_percentage, iteration_number
        left_over_tasks = parameters['left_over_tasks']  # initiated in GA_solo then edited on each iteration by deap_followup
        number_to_release = int(len(left_over_tasks) * release_percentage)
        # if the number of tasks left is less than the size of the window OR the normal tasks are all attempted and
        # there are only left over tasks then release all remaining tasks.
        # if len(left_over_tasks) <= parameters['window_size'] or len(parameters['sub_job_dict']) == len(parameters['attempted_tasks']) + len(left_over_tasks):
        #     number_to_release = len(left_over_tasks)
        new_tasks = left_over_tasks[:number_to_release]     #read the tasks according to the number identified
        for task in new_tasks:      #remove all the added tasks from left_over_tasks
            left_over_tasks.remove(task)
        parameters['left_over_tasks'] = left_over_tasks #update parameters
        parameters['new_tasks_released'] = new_tasks
        # *******************************************************************************************
        pickle.dump(parameters, open(parameters['FileString'] + "parameters.pkl", "wb"))

        # *************************************** TERMINATION CHECK ***************************************************
        sub_job_dict = pickle.load(open(parameters['Complete_file_String'] + 'sub_job_dict.pkl', 'rb'))

        #***************************************************************************************************************
        ## Adding this portion on 3rd April 2020 for the CPLEX based implementation. What I am doing over here
        ## is create a seprate dist_matrix for the tasks attempted by the robots in previous iterations and then format
        ## that dictionary as per the ampl implementation. This will allow me to find the optimal allocations for the
        ## tasks attempted in each iteration of RoSTAM and prove the convergence of the scheme.
        ## these are stand alone changes and do not require any other changes.
        original_dict = pickle.load(open(parameters['Complete_file_String'] + 'dist_mtrx_pickle.pkl', 'rb')) #reading the original
        combined_allocation = [int(item.split(',')[0]) for sublist in partial_robot_solutions for item in sublist]
        combined_allocation.insert(0,0)
        # the above line is flattening all robot allocations from previous round and removing the subjobs from it
        combined_allocation.sort()  # The allocated tasks are sorted in ascending so a new dist_matrix can be formed
        partial_dist_matrix = [original_dict[i] for i in combined_allocation] #reading the lines for the given indexe, but this
        # dict also has the distances mentioned for the tasks which are not allocated to any robot and shouldn't be included
        final_dist_mtrx = [[] for _ in range(len(combined_allocation))]

        ## this for loop will give it the final shape where the dist matrix only has distances for tasks involved in current allocation
        for i, list_inside in enumerate(partial_dist_matrix):
            for index in combined_allocation:
                if index == 0:
                    final_dist_mtrx[i].append(0) #making the first column zero coz can't get rid of return to home depot part from ampl
                else:
                    final_dist_mtrx[i].append(partial_dist_matrix[i][index])

        # sir has recommended to run a local RoSTAM which only optimizes the isolated tasks, this is an effort for that.
        # have written individual files, still have to figure out a way to execute this.
        dist_matrix_local = open(parameters['Complete_file_String'] + 'dist_mtrx_local.txt', 'w')
        for line in final_dist_mtrx:
            dist_matrix_local.write('[')
            for value in line:
                dist_matrix_local.write(str(value) +',')
            dist_matrix_local.write(']\n')
        dist_matrix_local.close()
        pickle.dump(final_dist_mtrx, open(parameters['Complete_file_String'] + 'dist_mtrx_local.pkl', 'w'))

        # calling the write for ampl updated which writes a data file using the distance matrix and then from within it self initiates the NEOS process.
        parsers.for_ampl_updated(parameters['robot_num_new'], active_robots[iteration_number-1], len(final_dist_mtrx), parameters['Complete_file_String'], final_dist_mtrx, parameters['iteration'], iteration_number)
        ##**************************************************************************************************************

        for robot in partial_robot_solutions:
            for task in robot:
                task = int(task.split(',')[0])

        i = len(sub_job_dict)
        count = i+1
        # This is to detect the last iteration.
        # if the number of attempted tasks is greater than
        # The total number of tasks in the sub_job_dict minus the window size then do not execute the loop any further.
        # if len(parameters['attempted_tasks']) >= count - parameters['window_size']:

        # if attempted tasks is 1 less than total number of tasks then terminate
        if len(parameters['attempted_tasks']) >= len(sub_job_dict):
        # if len(parameters['attempted_tasks']) == len(sub_job_dict)-1: #not going for == as it was prompting errors
            run_flag = False

        if len(partial_robot_solutions) > len(parameters['robot_solutions_windowed']):  # if there are new robots
            # how many empty lists are to be appended depends on the different between robots of previous and this run
            for i in range(abs(len(partial_robot_solutions) - len(parameters['robot_solutions_windowed']))):
                parameters['robot_solutions_windowed'].append([])
        #Saving each robot's complete individual solution so far
        for i, entry in enumerate(partial_robot_solutions):
            parameters['robot_solutions_windowed'][i].extend(entry)

        fitness_partial(partial_robot_solutions, parameters['failed_robots'])

        print 'COMPLETE SOLUTION SO FAR', parameters['robot_solutions_windowed']
        final_fitness, fitness_list = fitness_final(parameters['robot_solutions_windowed'])
        print 'FITNESS VALUE:', final_fitness

        # # Computing a load cofficient to give some idea of how busy the robot is from previous window. Will be used
        # # while computing the fitness of that robot in next window. An occupied robot will not be preferred much.
        # load_cofficient = [(1-(x/sum(fitness_list))) for x in fitness_list] #more occupied smaller value.
        # for i in range(len(parameters['failed_robots']) - len(load_cofficient)):
        #     load_cofficient.append(1)
        # # parameters['load_cofficient'] = load_cofficient #not updating it as not using it. If want to use then update
        # ## and add its multiplication with all the speeds in the fitness routines used for both ga_solo and deap_followu
        # # print 'ROBOT LOAD COFFICIENT:', load_cofficient

        pickle.dump(parameters, open(parameters['FileString'] + "parameters.pkl", "wb"))

        # if haven't reached termination criteria then execute the followup.
        if run_flag:
            os.system('python deap__followup.py')
            # run_flag = False
        parameters = pickle.load(open("/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/parameters.pkl", 'rb'))
        print 'LEFT OVER TASKS:',parameters['left_over_tasks']
        iteration_number += 1

    #writing the final solution in a file with tours for each robot identified seprately
    FileString = parameters['Complete_file_String']
    solution_file_ind = open(FileString + 'solution_individual_window.txt', "w")    #For writing complete solution at the end
    # robot_soluiton_window_ind = parameters['robot_solutions_windowed']
    for tour in parameters['robot_solutions_windowed']: #write each robots tour separately
        solution_file_ind.write(str(tour) + '\n')
    solution_file_ind.write(str(final_fitness) + '\n')  #write the final fitness of the solution
    for tour in parameters['robot_solutions_windowed']:
        solution_file_ind.write(str(len(tour))+',')
    solution_file_ind.write(str(fitness_list))      #write individual fitnesses of each robots tour as well
    solution_file_ind.close()

    # saving the final fitness to an overall file for easier computation of average for mutliple runs
    full_execution = open('/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/' + 'RoSTAM_Results.txt', 'a')
    full_execution.write(str(final_fitness) + '\n')  # making a single file for all the results, RoSTAM_Results.txt
    full_execution.close()

    parameters['after_followup'] = False
    pickle.dump(parameters, open(parameters['FileString'] + "parameters.pkl", "wb"))

    execution_log.close()


def followup():
    parameters = pickle.load(
        open("/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/parameters.pkl", 'rb'))
    if int(parameters['new_jobs']) != 0:
        job_alteration.add_job_simple(parameters['new_jobs'], parameters['new_jobs_type'], parameters['Complete_file_String'], parameters['map_dim'])
        job_alteration.job_representation_simple(parameters['Complete_file_String'])

    os.system("python deap__followup.py")


# evaluates the final solution. rbt_job will be a list of list each of which will contain the allocations for a particular robot.
def fitness_partial(rbt_job, robot_status):
    parameters = pickle.load(open("/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/parameters.pkl", 'rb'))
    FileString = parameters['Complete_file_String']  # FileString + iteration_string
    cst_mtrx = pickle.load(open(FileString + 'dist_matrix.pkl', 'rb'))
    start_matrix = cst_mtrx[0][:]  # initializing the start matrix, which is just the first line of cost matrix
    rbt_count = len(rbt_job)
    speed_matrix = parameters['speed_matrix_2']
    fitness_list = []  # a list of fitness values to return the max from
    fitness = 0
    for i in range(rbt_count):  # for each and every robot
        # print 'this is for robot', i
        for j in range(len(rbt_job[i])):
            task_id = rbt_job[i][j].split(',')
            task = int(task_id[0])
            if j == 0:  # if its the first job then start from base and go till first location
                # print 'Since this is the first task so going from home to ', task, 'with fitness value',start_matrix[task] / speed_matrix[i]
                fitness += start_matrix[task] / speed_matrix[i]  # coz I have appended this extra zero for home-home traveling  +1
            else:
                # print 'Now going from', previous_task, ' to ', task, 'with fitness value', cst_mtrx[previous_task][task] / speed_matrix[i]
                fitness += cst_mtrx[previous_task][task] / speed_matrix[i]  # +1 +1
            previous_task = task
        fitness_list.append(fitness)  # saving each robots fitness value into the list
        # for a normal distance based fitness just return the sum of this list with speed values set to 1 for all robots
        fitness = 0
    partial_solution_file = open(FileString + 'partial_solution_file.txt', 'a')
    partial_output_file = open(FileString + 'partial_RoSTAM_out.txt', 'a')
    for lst in rbt_job:
        partial_solution_file.writelines(str(lst) + '\n')
    partial_solution_file.writelines((str(fitness_list)+ str(max(fitness_list))+ '\n'))
    partial_solution_file.writelines('Next Iteration Robot Status:' + str(robot_status)+ '\n')
    partial_output_file.writelines(str(max(fitness_list)) + '\n')
    partial_output_file.close()
    partial_solution_file.close()
    print ('************* this is the RoSTAM output *****************', str(max(fitness_list)))
    return


# evaluates the final solution. rbt_job will be a list of list each of which will contain the allocations for a particular robot.
def fitness_final(rbt_job):
    parameters = pickle.load(open("/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/parameters.pkl", 'rb'))
    FileString = parameters['Complete_file_String']  # FileString + iteration_string
    cst_mtrx = pickle.load(open(FileString + 'dist_matrix.pkl', 'rb'))
    start_matrix = cst_mtrx[0][:]  # initializing the start matrix, which is just the first line of cost matrix
    rbt_count = len(rbt_job)
    speed_matrix = parameters['speed_matrix_2']
    fitness_list = []  # a list of fitness values to return the max from
    fitness = 0
    for i in range(rbt_count):  # for each and every robot
        # print 'this is for robot', i
        for j in range(len(rbt_job[i])):
            task_id = rbt_job[i][j].split(',')
            task = int(task_id[0])
            if j == 0:  # if its the first job then start from base and go till first location
                # print 'Since this is the first task so going from home to ', task, 'with fitness value',start_matrix[task] / speed_matrix[i]
                fitness += start_matrix[task] / speed_matrix[i]  # coz I have appended this extra zero for home-home traveling  +1
            else:
                # print 'Now going from', previous_task, ' to ', task, 'with fitness value', cst_mtrx[previous_task][task] / speed_matrix[i]
                fitness += cst_mtrx[previous_task][task] / speed_matrix[i]  # +1 +1
            previous_task = task
        if len(rbt_job[i]) > 0:  # checking if this particular robot has no jobs then there is no point completing the tour
            # print 'last job so completing the tour from ', task, 'to home', start_matrix[task] / speed_matrix[i]
            fitness += start_matrix[task] / speed_matrix[i]  # This is for completing the tour. from home to home  +1
        fitness_list.append(fitness)  # saving each robots fitness value into the list
        # for a normal distance based fitness just return the sum of this list with speed values set to 1 for all robots
        fitness = 0
    print fitness_list
    return (max(fitness_list), fitness_list)


# this is reading a hard coded text file from the last output generated.
# could create problems or errors if the structure of the file is changed of anything.
def extract_partial_robot_solutions_window_based(ind_solution,iteration_number, partial_robot_solutions, parameters):
    # this is the file processing part which right now is pretty lousy and pathetic. Improve on this if you get time.
    # this portion reads the individual solution file from the specific folder, where each line gives
    # the plan on one robot (final plan for all tasks) then it simply picks up the first few tasks
    # from that solution based on the window size.
    for i, line in enumerate(ind_solution):
        line = line.strip('[')
        data = line.split("'")
        for element in data:
            if element == ', ' or element == '' or element == ']\n':
                data.remove(element)
        # These two if's are basically checking for the last line; coz the last line holds the total cost
        # of the plan in the text file and must not be read. So if its the first iteration then the lines
        # will be equal to the previous number of robots in the execution and if its a followup execution then
        # the number of lines in the text file will be equal to the new number of robots.
        if iteration_number == 1 and i < int(parameters['robot_num']):
            partial_robot_solutions[i].extend(data[0:int(parameters['window_size'])])
        if iteration_number != 1 and i < int(parameters['robot_num_new']):
            partial_robot_solutions[i].extend(data[0:int(parameters['window_size'])])
    print 'this is the partial robot solution', partial_robot_solutions

# this is reading a hard coded text file from the last output generated.
# could create problems or errors if the structure of the file is changed of anything.
def extract_partial_robot_solutions_time_based(ind_solution,iteration_number, partial_robot_solutions, parameters, time):
    # this is the file processing part which right now is pretty lousy and pathetic. Improve on this if you get time.
    # this portion reads the individual solution file from the specific folder, where each line gives
    # the plan on one robot (final plan for all tasks) then it simply picks up the first few tasks
    # from that solution based on the window size.
    parameters = pickle.load(open("/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/parameters.pkl", 'rb'))
    FileString = parameters['Complete_file_String']  # FileString + iteration_string
    cst_mtrx = pickle.load(open(FileString + 'dist_matrix.pkl', 'rb'))
    start_matrix = cst_mtrx[0][:]  # initializing the start matrix, which is just the first line of cost matrix
    speed_matrix = parameters['speed_matrix_2']
    tour_times= []

    for i, line in enumerate(ind_solution):
        if (iteration_number == 1 and i < int(parameters['robot_num'])) or (iteration_number != 1 and i < int(parameters['robot_num_new'])):
            line = line.strip('[')
            rbt_job = line.split("'")
            for element in rbt_job:
                if element == ', ' or element == '' or element == ']\n':
                    rbt_job.remove(element)

            time_taken = 0
            for j in range(len(rbt_job)):
                task_id = rbt_job[j].split(',')
                task = int(task_id[0])
                if j == 0:  # if its the first job then start from base and go till first location
                    time_taken += start_matrix[task] / speed_matrix[i]  # coz I have appended this extra zero for home-home traveling  +1
                else:
                    # print 'Now going from', previous_task, ' to ', task, 'with fitness value', cst_mtrx[previous_task][task] / speed_matrix[i]
                    time_taken += cst_mtrx[previous_task][task] / speed_matrix[i]  # +1 +1
                previous_task = task
                if time_taken > time:
                    break
                partial_robot_solutions[i].append(rbt_job[j])
            tour_times.append(time_taken)
            time_taken = 0
    print 'this is the partial robot solution', partial_robot_solutions
    print 'these are the individual tour times', tour_times
    print 'these are the total number of tasks attempted by the robots in this iteration', len(compiler.ast.flatten(partial_robot_solutions))


#This routine updates the starting location of the robots after each iteration
#Since my new fitness evaluation appends the previous solution with the new solution I am not really using this routine
# in current setup.
def extract_start_location(partial_robot_solutions, previous_start_locations, start_locations):
    ## Start_locations always start with a [0,0,0,0,0,0] if they are different then this routine will need revisiting
    # Extracting the last locations from each robots tours and altering the start_location_2 from parameters for future run
    print('These are the starting locations of the robots', start_locations)
    return start_locations

    for i, each_list in enumerate(partial_robot_solutions):
        if each_list:
            data = each_list[-1].split(',')[0]  # splitting the last element of partial solution and reading the first portion only
            # print data
            start_locations[i] = int(data)
        if not each_list:
            pass    #if no allocations for this particular robot in this run then let it remain 0
            # leave the last location untourched
    for i,item in enumerate(start_locations):
        if item == 0:   #if any particular robot is not being allocated any task and doesn't have an updated start location
            if i < len(previous_start_locations):   #and that robot was part of the team previously
                start_locations[i] = previous_start_locations[i]    #then allocate it the previous starting location
            else:
                start_locations[i] = 0  #otherwise assign it a new start location as it sounds like a new robot.
    return start_locations

if __name__ == '__main__':
    main()
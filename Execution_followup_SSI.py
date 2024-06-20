import os
import job_alteration
import pickle
import plot_TSP
import SSI_auction_Sven
import SSI_followup
import compiler
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

    for a in range(50,55,5):    #Iterating over all the tasks
        for i in range(0,1):    # Iterating over all the instances of those tasks
            parameters['jobs'] = a  # saving the two variables a and i into parameters for use within the code
            parameters['iteration'] = i
            iteration_string = (str(parameters['jobs']) + "/" + str(parameters['iteration']) + "/")
            parameters['Complete_file_String'] = parameters['FileString'] + iteration_string
            pickle.dump(parameters, open(parameters['FileString'] + "parameters.pkl", "wb"))    #dumping the parameters set on the top
            window_with_new_tasks()

        #Redaing the output of multiple runs from the RoSTAM_Results file and averaging it.
        #The averaging value is hard coded and must be changed according to the number of iterations run for averaging.
        fitness_file = open('/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/' + 'SSI_Results.txt','r+')
        data = 0
        for line in fitness_file:
            data += float(line)
            fitness_allrecord.write(str(float(line)) + '\n')
        data = data / 3  # There are 3 cases in each task count so averaging over 3.
        average_fitness.write(str(data) + '\n')
        fitness_file.seek(0)
        fitness_file.truncate()
    fitness_file.close()
    average_fitness.close
    fitness_allrecord.close()


def window_with_new_tasks():
    parameters = pickle.load(open("/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/parameters.pkl", 'rb'))
    execution_log = open(parameters['Complete_file_String'] + 'SSI_Dynamic_execution_log.txt', 'a')
    execution_log.seek(0)
    execution_log.truncate()

    # dynamic_times = [10, 10, 5, 10, 50000]  #Time intervals at which the disturbance will be introduced.
    #first at 10 secs, next at 10 secs after first, next at 5 secs, next at 10 and final at a loooonng time for complete executions.

    #the first iteration should hold all active. after than you can have failed robots. The total count shouldn't go
    # down, can increase or remain same. IF you want to remove a robot permanently then make it 0 for all other iterations
    active_robots = [[1, 1, 1],
                     [1, 0, 1],
                     [1, 0, 1],
                     [1, 1, 0],
                     [1, 1, 0],
                     [1, 1, 0],
                     [1, 1, 0, 1],
                     [1, 1, 1, 1],
                     [1, 0, 1],
                     [1, 0, 1],
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
    percentage_to_release = [0.5, 1.0, 0, 0, 0, 0, 0, 0]
    # *******************************************************************************************

    iteration_number = 1

    execution_log.writelines('starting locations for robots before first iteration' + str(parameters['start_location']) + '\n')
    execution_log.writelines('current active robot for first iteration' + str(parameters['failed_robots']) + '\n')

    SSI_auction_Sven.main()  # SSI solution for initial job distribution

    # the after followup flag makes the GA_solo and SSI read the new job discriptions from the 'followup' folder inside the 50/0 thing
    # reading the parameters again before writing them so if they are changed within GA_Solo they wouldn't be overwritten
    parameters = pickle.load(open("/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/parameters.pkl", 'rb'))
    execution_log.writelines('tasks randomly dropped at the start of first iteration' + str(parameters['left_over_tasks']) + '\n')
    print "*********************************************** these are the left over tasks *********************************************"
    print parameters['left_over_tasks']
    run_flag = True  # keeping a flag to terminate the while loop.

    while run_flag:
        start_locations = parameters['start_location_2']
        parameters['failed_robots'] = active_robots[iteration_number]

        ind_solution = open(parameters['Complete_file_String'] + 'auction_solution.txt', 'r')

        # Checking coz if new robots are added compared to first run then.
        # if so is the case then solutions should be the size of new count
        if iteration_number == 1:
            partial_robot_solutions = [[] for _ in range(int(parameters['robot_num']))]  # identifying partial robot solutions for each individual window.
        else:
            partial_robot_solutions = [[] for _ in range(int(parameters['robot_num_new']))]  # identifying partial robot solutions for each individual window.

        ##extracting the window size based partial solutions for each robot from the best solution of last run
        extract_partial_robot_solutions_window_based(ind_solution, iteration_number, partial_robot_solutions, parameters)
        ## extracting the time based partial solutions for each robot from the best solution of last run
        # extract_partial_robot_solutions_time_based(ind_solution, iteration_number, partial_robot_solutions, parameters, dynamic_times[iteration_number -1])

        ## Updating a few parameters
        parameters['robot_num_new'] = len(active_robots[iteration_number])
        start_location_2 = [0] * parameters['robot_num_new']  # This is altered in Execution_followup.py before calling deap__followup.py
        for i,item in enumerate(start_locations):
            start_location_2[i] = start_locations[i]
        parameters['start_location_2'] = extract_start_location(partial_robot_solutions, start_locations, start_location_2)

        execution_log.writelines('left over tasks still to be introduced' + str(parameters['left_over_tasks']) + '\n')
        execution_log.writelines(('new tasks which are to be released in this iteration') + str(parameters['new_tasks_released']) + '\n')
        execution_log.writelines('window solution from ' + str(iteration_number) + ' run : ' + str(partial_robot_solutions)+ '\n')
        execution_log.writelines('Active robot for iteartion' + str(iteration_number+1) + str(active_robots[iteration_number]) + '\n')
        execution_log.writelines('starting locations for robots for iteration' + str(iteration_number+1) + str(parameters['start_location_2']) + '\n')

        # adding the partial solution to the complete solutions I have been maintaining from outside the while loop
        for element in partial_robot_solutions:
            parameters['attempted_tasks'].extend(element)

        # *******************************************************************************************
        release_percentage = percentage_to_release[iteration_number - 1]
        print 'The percentage to release and the iteration number are', release_percentage, iteration_number - 1
        left_over_tasks = parameters['left_over_tasks']  # initiated in GA_solo then edited on each iteration by deap_followup
        number_to_release = int(len(left_over_tasks) * release_percentage)
        # if the number of tasks left is less than the size of the window OR the normal tasks are all attempted and there are only left over tasks then release all remaining tasks.
        # if len(left_over_tasks) <= parameters['window_size'] or len(parameters['sub_job_dict']) == len(parameters['attempted_tasks']) + len(left_over_tasks):
        #     number_to_release = len(left_over_tasks)
        new_tasks = left_over_tasks[:number_to_release] #read the tasks according to the number identified
        for task in new_tasks:          #remove all the added tasks from left_over_tasks
            left_over_tasks.remove(task)
        parameters['left_over_tasks'] = left_over_tasks #update parameters
        parameters['new_tasks_released'] = new_tasks
        # *******************************************************************************************

        pickle.dump(parameters, open(parameters['FileString'] + "parameters.pkl", "wb"))

        # ***************************************************************************************************************
        sub_job_dict = pickle.load(open(parameters['Complete_file_String'] + 'sub_job_dict.pkl', 'rb'))
        i = len(sub_job_dict)
        count = i + 1
        # This is to detect the last iteration. if the number of attempted tasks is greater than
        # The total number of tasks in the sub_job_dict minus the window size then do not execute the loop any further.
        # if len(parameters['attempted_tasks']) >= count - parameters['window_size']:

        # if attempted tasks is 1 less than total number of tasks then terminate
        if len(parameters['attempted_tasks']) >= len(sub_job_dict):
            run_flag = False

        iteration_number += 1

        # Saving the complete solution from partial to complete.
        # These next two lines are added to entertain robot addition into the team. if robots are being added then new
        # lists need to be appended into this array
        # partial robot solution is the updated array with current robot count and robot solution windowed is working with old count
        if len(partial_robot_solutions) > len(parameters['robot_solutions_windowed']):  # if there are new robots
            # how many empty lists are to be appended depends on the different between robots of previous and this run
            for i in range(abs(len(partial_robot_solutions) - len(parameters['robot_solutions_windowed']))):
                parameters['robot_solutions_windowed'].append([])
        for i, entry in enumerate(partial_robot_solutions):
            parameters['robot_solutions_windowed'][i].extend(entry)
        print 'this is the complete solution so far', len(parameters['robot_solutions_windowed']), parameters['robot_solutions_windowed']
        final_fitness, fitness_list = fitness_final(parameters['robot_solutions_windowed'])
        print parameters['robot_solutions_windowed']
        print 'this is the final fitness value', final_fitness

        pickle.dump(parameters, open(parameters['FileString'] + "parameters.pkl", "wb"))

        # if haven't reached termination criteria then execute the followup.
        if run_flag:
            SSI_followup.main()
        parameters = pickle.load(open("/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/parameters.pkl", 'rb'))
        print "*********************************************** these are the left over tasks *********************************************"
        print parameters['left_over_tasks']



    # writing the final solution in a file with tours for each robot identified seprately
    FileString = parameters['Complete_file_String']  # FileString + iteration_string
    solution_file_ind = open(FileString + 'SSI_window_solution.txt', "w")
    robot_soluiton_window_ind = parameters['robot_solutions_windowed']
    for tour in robot_soluiton_window_ind:
        solution_file_ind.write(str(tour) + '\n')
    solution_file_ind.write(str(final_fitness) + '\n')
    for tour in robot_soluiton_window_ind:
        solution_file_ind.write(str(len(tour))+',')
    solution_file_ind.write(str(fitness_list))
    solution_file_ind.close()

    # saving the final fitness to an overall file for easier computation of average for mutliple runs
    full_execution = open('/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/' + 'SSI_Results.txt', 'a')
    full_execution.write(str(final_fitness) + '\n')  # making a single file for all the results, RoSTAM_Results.txt
    full_execution.close()

    parameters['after_followup'] = False
    pickle.dump(parameters, open(parameters['FileString'] + "parameters.pkl", "wb"))

    execution_log.close()


def fitness_final(rbt_job):
    parameters = pickle.load(
        open("/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/parameters.pkl", 'rb'))
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

def extract_partial_robot_solutions_window_based(ind_solution,iteration_number, partial_robot_solutions, parameters):
    # this is the file processing part which right now is pretty lousy and pathetic. Improve on this if you get time.
    # this portion reads the individual solution file from the specific folder, where each line gives
    # the plan on one robot (final plan for all tasks) then it simply picks up the first few tasks
    # from that solution based on the window size.
    for i, line in enumerate(ind_solution):
        line = line.strip('[')
        line = line.replace(']\n', '')
        data = line.split("'")
        for entry in data:
            if entry == ',' or entry == ', ' or entry =='':
                data.remove(entry)


        # line = line.strip('[')
        # line = line.replace(',','')
        # line = line.replace(']\n','')
        # data = line.split(" ")
        # These two if's are basically checking for the last line; coz the last line holds the total cost
        # of the plan in the text file and must not be read. So if its the first iteration then the lines
        # will be equal to the previous number of robots in the execution and if its a followup execution then
        # the number of lines in the text file will be equal to the new number of robots.
        if iteration_number == 1 and i < int(parameters['robot_num']):
            # for j, entry in enumerate(data):
            #     data[j] = int(entry.split(',')[0])
            # print data
            partial_robot_solutions[i].extend(data[0:int(parameters['window_size'])])
        if iteration_number != 1 and i < int(parameters['robot_num_new']):
            # for j, entry in enumerate(data):
            #     data[j] = int(entry.split(',')[0])
            # print data
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




def extract_start_location(partial_robot_solutions, previous_start_locations, start_locations):
    # start_locations = [None] * len(partial_robot_solutions)

    # Extracting the last locations from each robots tours and altering the start_location_2 from parameters for future run
    for i, each_list in enumerate(partial_robot_solutions):
        if each_list:
            data = each_list[-1].split(',')[0]  # splitting the last element of partial solution and reading the first portion only
            print data
            start_locations[i] = int(data)
        if not each_list:
            pass
            # leave the last location untourched
    for i,item in enumerate(start_locations):
        if item == None:
            if i < len(previous_start_locations):
                start_locations[i] = previous_start_locations[i]
            else:
                start_locations[i] = 0
    return start_locations

if __name__ == '__main__':
    main()
import auction_robot
import parsers
import copy
import random
import pickle
import auction_traderbots
import SSI_auction_Sven


def main():
    parameters = pickle.load(open("/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/parameters.pkl", 'rb'))
    speed_matrix = parameters['speed_matrix_2']
    FileString = parameters['Complete_file_String']
    robot_num = parameters['robot_num_new']
    robot_num_old = len(parameters['robot_solutions_windowed'])
    attempted_tasks = parameters['attempted_tasks']
    robot_solutions = parameters['robot_solutions_windowed']

    dict_list = []      # an empty list of dictionaries for saving the bids against all tasks for each robot.

    # creating a list of robot type objects; robots have a job list and a cost value slot in them.
    robots = []
    for i in range(robot_num_old):
        if robot_solutions[i]:
                robots.append(auction_robot.Robot(robot_solutions[i], robot_solutions[i][-1], speed_matrix[i]))
        else:
            robots.append(auction_robot.Robot([], 0, speed_matrix[i]))
    for i in range(robot_num-robot_num_old):
        robots.append(auction_robot.Robot([], 0, speed_matrix[i]))


    dist_matrix = pickle.load(open(FileString + 'dist_matrix.pkl', 'rb'))   # The distance matrix saved by deap__followup
    sub_job_dict = pickle.load(open(FileString + 'sub_job_dict.pkl', 'rb'))     # job dict withou the old tasks (all of them removed)

    left_over_tasks = parameters['left_over_tasks']  # initiated in GA_solo then edited on each iteration by deap_followup
    # number_to_release = random.randint(0, len(left_over_tasks))  # randomly picking the number of tasks to be released
    # # if the number os tasks left is less than the size of the window OR the normal tasks are all attempted and there are only left over tasks then release all remaining tasks.
    # if len(left_over_tasks) <= parameters['window_size'] or len(sub_job_dict) == len(attempted_tasks) + len(left_over_tasks):
    #     number_to_release = len(left_over_tasks)
    # new_tasks = left_over_tasks[:number_to_release]
    # for task in new_tasks:
    #     left_over_tasks.remove(task)
    # for task in left_over_tasks:
    #     del sub_job_dict[task]
    # parameters['left_over_tasks'] = left_over_tasks
    # parameters['new_tasks_released'] = new_tasks

    for task in left_over_tasks:
        del sub_job_dict[task]
    for task in attempted_tasks:
        del sub_job_dict[task]
    new_tasks = parameters['new_tasks_released']


    print 'the sub job dict for the new operation',len(sub_job_dict), sub_job_dict.keys()
    print  'these are the new tasks',len(new_tasks), new_tasks
    print 'thse are the tasks which are still left to be introduced', len(left_over_tasks), left_over_tasks

    pickle.dump(parameters, open(parameters['FileString'] + "parameters.pkl", "wb"))

    sub_job_dict_auction = sub_job_dict #I am commenting the below few line on 21/Nov2018 and adding this line
    # effectively should do the same thing but wasn't really doing the same thing. Have a look at diary for details

    # # creating a sub_job_dictionary of tasks that are to be done now (after deleting the tasks already attempted).
    # sub_job_dict_auction = copy.deepcopy(sub_job_dict)
    # for entry in attempted_tasks:
    #     # if entry in new_tasks:
    #     if entry in sub_job_dict_auction:
    #         del sub_job_dict_auction[entry]
    # sub_job_dict = sub_job_dict_auction

    fitness = 0

    for i in range(robot_num):  # for each individual robot create a bid dictionary
        if robots[i].job_done:
            dict_list.append(update_bid_dict(robots[i], i, dist_matrix,sub_job_dict_auction, parameters['failed_robots']))  # not passing the robot over here coz all the robots are empty right now.
        else:
            dict_list.append(create_bid_dict(robots[i], i, dist_matrix,sub_job_dict_auction, parameters['failed_robots']))  # not passing the robot over here coz all the robots are empty right now.
        # The returned dict will be in the form Key:"taskid,insertion_point", values: fitness value(distance), insertion point, robot id, task_id
    remove_taskid_value = 0 #Dummy values
    assign_robotid = 90

    # Running the loop for all the sub_jobs
    for p in range(len(sub_job_dict)):
        new_dict_list = []      # creating an empty list of dictionaries for saving the altered dictionaries
        bids = []               # a list of all the bids to figure out the smallest bid later on

        # picking out the smallest bid in all the dictionaries combined. as the allocation is made to smallest bid per round
        for dict in dict_list:
            for keyz, values in dict.iteritems():
                bids.append(values[0])  #collecting all the bids as a list of entries
        # print 'incoming bids from all robot', bids
        min_bid = min(bids)         # Saving the minimum of the bids
        # print 'minimum of these bids', min_bid

        #*********************************************************************************************************
        final_key_min = 0
        minimum = 1000
        for j, dict in enumerate(dict_list):
            key_min = min(dict.keys(), key=(lambda k: dict[k]))
            if dict[key_min][0] < minimum:
                minimum = dict[key_min][0]
                final_key_min = key_min
                assign_robotid = dict[key_min][2]
                remove_taskid_value = dict[key_min][3]


        # next identifying the value(taskid) against the min_bid so that it can be assigned to the appropriate robot
        # and deleted from all the robot bids this particular job

        for dict in dict_list:          # this loop is to identify the task_id and robot_id against the minimum bid
            for key, value in dict.items():
                if min_bid == value[0]: # if minimm
                    remove_taskid_value = value[3]  # task id, removing the last 2 characters since key is in the for "8,1,0' where ,0 is the insertion id
                    assign_robotid = value[2]       # robot_id
        print 'Minimum bid details, robot:', assign_robotid, 'task_id', remove_taskid_value, 'bid', min_bid

        # from the above identified remove_taskid_value here is where i delete it from all robot's bids
        # that entry shouldn't exist anywhere since it is being assigned.
        # here i think I would have to create a list of all the unassigned tasks so far and then delete the entry from that as well.
        for dict in dict_list:
            # In the next line the task id in the form '8,1' will be matched in all the values of all the robot's bid
            # dictionaries and anywhere where it matches the particular key value pair will be removed.
            dict = {k:v for k,v in dict.items() if remove_taskid_value not in v}
            new_dict_list.append(dict)  # temporarily saving the altered bid_dict and will copy them on original in coming lines

        dict_list = new_dict_list       # updating the dictionaries after removing the assigned task
        robots[assign_robotid].job_done.append(remove_taskid_value) # assigning the task to the apt robot,#  the -1 is coz of the difference in indexing in this code (starts from 1) and my robots (start from 0)
        sub_job_dict.pop(remove_taskid_value, None) # removing the entry from new_sub_job_dict
        dict_list[assign_robotid] = update_bid_dict(robots[assign_robotid], assign_robotid, dist_matrix, sub_job_dict_auction, parameters['failed_robots'])

        # print "iteration" + str(p)
        # print "Assignments:"
        # for robot in robots:
        #     print robot.job_done
    fitness = auction_traderbots.final_fitness_max_time(robots, dist_matrix)
    print fitness
    print speed_matrix
    for i in range(len(robot_solutions)):
        for entry in attempted_tasks:
            if entry in robots[i].job_done:
                robots[i].job_done.remove(entry)
    file_write = open(FileString + 'auction_solution.txt', 'a')
    file_write.seek(0)
    file_write.truncate()
    full_execution = open('/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/' + 'SSI_solution_ITERwise.txt', 'a')
    # rest of the execution is very similar to the above two auction and reauction
    # file_write.write("SSI_auction_Followup_Run" + '\n')
    for robot in robots:
        file_write.write(str(robot.job_done) + '\n')
    file_write.write(str(fitness)+ '\n')
    for robot in robots:
        file_write.write(str(len(robot.job_done)) + ',')
    file_write.write(str(auction_traderbots.final_fitness_list_time(robots, dist_matrix)))
    full_execution.write(str(fitness) + '\n')
    file_write.close()
    full_execution.close()
    # print('*************************This is the fitness for the final solution', fitness)
    iteration_solution_file = open(FileString + 'SSI_iteration_solution', 'a')
    print('*************************This is the fitness for the final solution', fitness)
    iteration_solution_file.writelines(str(fitness) + '\n')
    iteration_solution_file.close()
    pickle.dump(robots, open(FileString + 'SSI_solution_file.pkl', 'wb'))
#
# # this routine will be responsible for creating a bidding dictionary for every individual robot, the dictionary structure will be
# # task_id(key): (values) bid value, insertion_id withincurrent plan, robot id, task_id
# # this routine is for creating the initial bids
# def create_bid_dict(robot, robot_id, robot_speed, dist_matrix, sub_job_dict):
#     bid_dict = {}
#     insert_id = 0
#     robot_id += 1           # the robot id's in my text are starting from 1 and in my code are starting from 0
#     for key in sub_job_dict:
#         job = key.split(',')
#         current_cost = robot.job_cost_hetro(int(robot.current_job.split(',')[0]), int(job[0]), dist_matrix)
#         value_list = [current_cost, insert_id, robot_id, key]
#         bid_dict[key + "," + str(insert_id)] = value_list
#     return bid_dict
#

# this routine will be responsible for creating a bidding dictionary for every individual robot, the dictionary structure will be
# task_id(key): (values) bid value, insertion_id withincurrent plan, robot id, task_id
# this routine is for creating the initial bids
def create_bid_dict(robot, robot_id, dist_matrix, sub_job_dict, failed_robots):
    bid_dict = {}
    robot_speed = robot.speed
    insert_id = 0
    # robot_id += 1           # the robot id's in my text are starting from 1 and in my code are starting from 0
    for key in sub_job_dict:
        job = key.split(',')
        current_cost = robot.job_cost_hetro(0, int(job[0]), dist_matrix)
        if failed_robots[robot_id] == 0:  # if robot already doing some portion of that job, don't assign another
            current_cost = 100000
        value_list = [current_cost, insert_id, robot_id, key]
        bid_dict[key + "," + str(insert_id)] = value_list
    return bid_dict

# This routine updates the bid_dictionary for particular robot, after the job has been assigned to them.
def update_bid_dict(robot, robot_id,  dist_matrix, sub_job_dict, failed_robots):
    robot_speed = robot.speed
    bid_dict = {}
    insert_id = len(robot.job_done)
    # robot_id += 1  # the robot id's in my text are starting from 1 and in my code are starting from 0
    allocation_list = [int(i.split(',')[0]) for i in robot.job_done]
    for key in sub_job_dict:
        job = key.split(',')
        last_job = robot.job_done[-1]
        last_job = last_job.split(',')
        current_cost = robot.job_cost_hetro(int(last_job[0]), int(job[0]), dist_matrix) # finding the cost of doing a job from the last job done
        if (int(job[0]) in allocation_list) or failed_robots[robot_id] == 0:  # if robot already doing some portion of that job, don't assign another
            current_cost = 100000
        value_list = [current_cost, insert_id, robot_id, key]
        bid_dict[key + "," + str(insert_id)] = value_list
    return bid_dict

if __name__ == '__main__':
    main()
import auction_robot
import parsers
import pickle
import auction_traderbots
import copy


def main():
    followup = False
    parameters = pickle.load(open("/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/parameters.pkl", 'rb'))
    speed_matrix = parameters['speed_matrix']
    FileString = parameters['Complete_file_String']
    robot_num = parameters['robot_num']
    percentage_to_drop = parameters['hidden_task_percentage']

    dict_list = []  # an empty list of dictionaries for saving the bids against all tasks for each robot.

    # creating a list of robot type objects; robots have a job list and a cost value slot in them.

    robots = [auction_robot.Robot([], 0, speed_matrix[i]) for i in range(robot_num)]

    # diupdate_bid_dict(robots[assign_robotid - 1], assign_robotid, dist_matrix, sub_job_dict)update_bid_dict(robots[assign_robotid - 1], assign_robotid, dist_matrix, sub_job_dict)update_bid_dict(robots[assign_robotid - 1], assign_robotid, dist_matrix, sub_job_dict)update_bid_dict(robots[assign_robotid - 1], assign_robotid, dist_matrix, sub_job_dict)st_matrix = parsers.read_distmatrix(FileString)
    dist_matrix = pickle.load(open(FileString + 'dist_matrix.pkl', 'rb'))
    job_dict = pickle.load(open(FileString + 'job_dict.pkl', 'rb'))

    sub_job_dict = pickle.load(open(FileString + 'sub_job_dict.pkl', 'rb'))

    # for entry in sub_job_dict:
    #     print entry
    # So adding this portion to strip off a few task keys for later entry as dynamic changes. This will include tasks expansion
    # more than new task creation. If I have to only work with task expansions then I think I should strip keys from job_dict rather than sub_job_dict
    # left_tasks = random.sample(sub_job_dict, int(len(sub_job_dict)*percentage_to_drop))
    left_tasks = sub_job_dict.keys()[int(len(sub_job_dict) * (1 - percentage_to_drop)):]
    for task in left_tasks:
        del sub_job_dict[task]
    print 'the new sub job dict with tasks removed', len(sub_job_dict), sub_job_dict.keys()
    print 'the removed tasks are', len(left_tasks), left_tasks

    parameters['left_over_tasks'] = left_tasks
    pickle.dump(parameters, open(parameters['FileString'] + "parameters.pkl", "wb"))


    fitness = 0

    for i in range(robot_num):  # for each individual robot create a bid dictionary
        dict_list.append(create_bid_dict(robots[i],i, speed_matrix[i], dist_matrix, sub_job_dict)) # not passing the robot over here coz all the robots are empty right now.
        # The returned dict will be in the form Key:"taskid,insertion_point", values: fitness value(distance), insertion point, robot id, task_id
    remove_taskid_value = 0 #Dummy Values
    assign_robotid = 100

    # Running the loop for all the sub_jobs
    for p in range(len(sub_job_dict)):
        new_dict_list = []      # creating an empty list of dictionaries for saving the altered dictionaries
        bids = []               # a list of all the bids to figure out the smallest bid latupdate_bid_dict(robots[assign_robotid - 1], assign_robotid, dist_matrix, sub_job_dict)er on

        # picking out the smallest bid in all the dictionaries combined. as the allocation is made to smallest bid per round
        for dict in dict_list:
            for keyz, values in dict.iteritems():
                bids.append(values[0])
        # print 'incoming bids from all robot', bids
        min_bid = min(bids)  # Saving the minimum of the bids
        # print 'minimum of these bids', min_bid

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
        task = remove_taskid_value.split(',')
        robots[assign_robotid-1].job_done.append(remove_taskid_value) # assigning the task to the apt robot
        sub_job_dict.pop(remove_taskid_value, None) # removing the entry from sub_job_dict
        dict_list[assign_robotid-1] = update_bid_dict(robots[assign_robotid - 1], assign_robotid, dist_matrix, sub_job_dict)

        # print "iteration" + str(p)
        # print "Assignments:"
        # for robot in robots:
        #     print robot.job_done
    fitness = auction_traderbots.final_fitness_max_time(robots, dist_matrix)
    print fitness
    print speed_matrix
    # This part is added at the end of all the SSI stuff to reschedule the SSI routes already found. A simple greedy algorithm
    # isn't changing the allocations at all, i guess coz they are alredy designed behind a greedy strucutre. Have to comeup
    # with something better.
    # I plan to alter the robot.current_jobs or such similar routine in there.
    # for robot in robots:
    #     greedy_reallocation(robot, 0, dist_matrix)


    file_write = open(FileString + 'auction_solution.txt', 'a')
    file_write.seek(0)
    file_write.truncate()
    full_execution = open('/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/' + 'SSI_solution_ITERwise.txt', 'a')
    # rest of the execution is very similar to the above two auction and reauction
    # file_write.write("SSI_auction_Complete_Run" + '\n')
    for robot in robots:
        file_write.write(str(robot.job_done) + '\n')
    file_write.write(str(fitness)+ '\n')
    for robot in robots:
        file_write.write(str(len(robot.job_done))+',')
    file_write.write(str(auction_traderbots.final_fitness_list_time(robots, dist_matrix)))
    full_execution.write(str(fitness) + '\n')
    # print('*************************This is the fitness for the final solution', fitness)
    iteration_solution_file = open(FileString + 'SSI_iteration_solution', 'a')
    print('*************************This is the fitness for the final solution', fitness)
    iteration_solution_file.writelines(str(fitness) + '\n')
    iteration_solution_file.close()
    # Writing this for the SSI followup sort of thing, where the procedure can be started from the previous allocation of robots.
    if parameters['after_followup'] == False:
        pickle.dump(robots, open(FileString + 'SSI_solution_file.pkl', 'wb'))


# this routine will be responsible for creating a bidding dictionary for every individual robot, the dictionary structure will be
# task_id(key): (values) bid value, insertion_id withincurrent plan, robot id, task_id
# this routine is for creating the initial bids
def create_bid_dict(robot, robot_id, robot_speed, dist_matrix, sub_job_dict):
    bid_dict = {}
    insert_id = 0
    robot_id += 1           # the robot id's in my text are starting from 1 and in my code are starting from 0
    for key in sub_job_dict:
        job = key.split(',')
        current_cost = robot.job_cost_hetro(0, int(job[0]), dist_matrix)
        value_list = [current_cost, insert_id, robot_id, key]
        bid_dict[key + "," + str(insert_id)] = value_list
    return bid_dict

# This routine updates the bid_dictionary for particular robot, after the job has been assigned to them.
def update_bid_dict(robot, robot_id,  dist_matrix, sub_job_dict):
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
        if int(job[0]) in allocation_list:  # if robot already doing some portion of that job, don't assign another
            current_cost = 100000
        value_list = [current_cost, insert_id, robot_id, key]
        bid_dict[key + "," + str(insert_id)] = value_list

    return bid_dict

def greedy_reallocation(robot, starting_location, distance_matrix):
    # current_plan = copy.deepcopy(robot.job_done)
    # current_location = starting_location
    # new_allocations = []
    # for i in range(len(current_plan)):
    #     task_costs = []
    #     for task in current_plan:
    #         task_costs.append(distance_matrix[int(current_location)][int(task)])
    #     min_index = task_costs.index(min(task_costs))
    #     new_allocations.append(current_plan[min_index])
    #     current_location = current_plan[min_index]
    #     current_plan.pop(min_index)
    None

if __name__ == '__main__':
    main()
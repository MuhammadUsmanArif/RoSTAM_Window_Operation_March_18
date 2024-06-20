# Right now calling this implementation of auction based scheme from within deap__init so that all the results are
# saved in the right folders and the speed values, jobs, filestring and distance matrices are passed from there as well.

import auction_robot
import parsers
import pickle
import random

#doesn't really pick any of these parameters from here until you want to execute main of this code (done for testing)
# in that case make main look like main() i.e. without any parameters
robot_num = 5
jobs = 50
# speed_matrix = parsers.read_speed()
# speed_matrix = [1, 1, 1, 1, 1]
speed_matrix = [ .7, .68, .65, .69, .66]
FileString = "/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/50/0/"


def main(FileString, robot_num, jobs, speed_matrix):
    robots = [auction_robot.Robot([],0, speed_matrix[i]) for i in range(robot_num)]  # creating a list of robot type objects; robots have a job list and a cost value slot and speed in them.

    #the robots initiated above were used by auction and when i reached the random allocation the same robots were used
    # which already had plans assigned, so I initiated a new set of robots for random allocations
    random_robots = [auction_robot.Robot([], 0, speed_matrix[i]) for i in range(robot_num)]  # creating a list of robot type objects; robots have a job list and a cost value slot and speed in them.

    # Reading the previously generated files
    dist_matrix = parsers.read_distmatrix(FileString)
    with open(FileString + 'job_dict.pkl','rb') as handle:
        job_dict = pickle.loads(handle.read())
    with open((FileString + 'sub_job_dict.pkl'), 'rb') as handle:
        sub_job_dict = pickle.loads(handle.read())
    fitness = 0

    #initial assignments
    init_assign(robots, sub_job_dict, dist_matrix)
    # computing the fitness of the final solution
    file_write = open(FileString + 'auction_solution.txt', 'wb')
    file_write.write("Auction" + '\n')
    for robot in robots:            # print the final plans of all the robots
        print robot.job_done
        file_write.write(str(robot.job_done) +'\n')
    fitness = final_fitness_max_time(robots, dist_matrix)
    print fitness
    file_write.write(str(fitness)+'\n')

    # reallocation after the initial allocation. makes something similar to turtlebot.
    re_auction(robots, dist_matrix)
    file_write.write("Re-Auction" + '\n')
    for robot in robots:  # print the final plans of all the robots
        print robot.job_done
        file_write.write(str(robot.job_done)+'\n')
    fitness = final_fitness_max_time(robots, dist_matrix)
    print fitness
    file_write.write(str(fitness)+'\n')

    random(random_robots, sub_job_dict)  # calling the random allocation routine which randomly allocates tasks, takes care of ST-SR-IA and ST-MR-IA both
    # initiating this extra text file so its easier to copy paste the readings of the whole iteration(from execution.py)
    # at the end.
    full_execution = open('/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/' + 'random_solution.txt', 'a')
    #rest of the execution is very similar to the above two auction and reauction
    file_write.write("Random Allocation" + '\n')
    for robot in random_robots:  # print the final plans of all the robots
        print robot.job_done
        file_write.write(str(robot.job_done) + '\n')
    fitness = final_fitness_max_time(random_robots, dist_matrix)
    print fitness
    file_write.write(str(fitness) + '\n')
    full_execution.write(str(fitness)+'\n')
    # file_write.close()


    # re_auction_absolute(robots, dist_matrix)
    # for robot in robots:  # print the final plans of all the robots
    #     print robot.job_done
    # fitness = final_fitness(robots, dist_matrix)
    # print fitness


#******************************************** Simple Greedy Auction ****************************************************
# this is the simple auction, which for each new job checks all the robots and figures out the least cost and assigns
# the job to that robot for that particular location.
def init_assign(robots, sub_job_dict, dist_matrix):
    for keys in sub_job_dict.keys():  # Run the allocation scheme for each and every subtask to be assigned
        min = 100  # random min values to compare with robot resources required everytime
        print keys
        min_length = len(sub_job_dict)
        # this portion initially assigns all the tasks to robots based on their initial bidds.
        for i in range(len(robots)):  # for each subjob check each robot (take bid from each robot)
            # the simple id tells the location of where this new job needs to be entered, and the length returns the number of jobs already assigned to the robot
            flag, job_id, id, length = robots[i].calculate_cost_hetro(keys,dist_matrix, 1000, min_length)
            if flag and robots[i].current_cost < min:  # if the new robot uses lesser resources than any previous robot then assign the task to this robot
                min_length = length
                min = robots[i].current_cost
                index = i  # This index basically is telling which robot will do this particular job
                insert_id = id  # what location the task should be entered in the current plan
            elif flag and robots[i].current_cost == min and length < min_length: #If there are two robots with the same cost then assign the task to robot with fewer tasks assigned
                min_length = length
                min = robots[i].current_cost
                index = i  # This index basically is telling which robot will do this particular job
                insert_id = id  # what location the task should be entered in the current plan
        robots[index].job_done.insert(insert_id, job_id)
        # robots[index].current_job = int(job_id)
        print('the minimum value is' + str(min))
        print ("job number " + str(keys) + "will be done by robot" + str(index))
#***********************************************************************************************************************


#*****************************************  Re Auctions ****************************************************************
# implemented these reacution to cover up for the order in which the jobs appear. Since the simple auction does not recheck
# for the initially assigned jobs if they can perform well for any other robot or even for the same robot anywhere in the
# later stages of the plan. So reauction will go through each robots plans for each job and check if it fitts somewhere better
# Didn't really work well for me.
def re_auction(robots, dist_matrix):
    for i in range(len(robots)):
        current_cost = 0
        deletion_index = []     # would make a list of indices which are allocated to other robots and must be deleted from current robot
        for j in range(len(robots[i].job_done)):
            deletion_flag = False
            change_flag = False
            if j == 0:  # if first job of this particular robot then find distance from home location
                current_cost = robots[i].job_cost_hetro(0, robots[i].job_done[j], dist_matrix)
            else:  # if not the first job of this robot then find distance from previous job to this job
                current_cost = robots[i].job_cost_hetro(robots[i].job_done[j - 1], robots[i].job_done[j], dist_matrix)
            for k in range(len(robots)):  # running another loop on robots to find cost of all other robots
                if k == i:  # if talking of the same robot then skip
                    None    # do nothing for the same robot.
                else:  # for all other robots find the new costs and if it is better then reassign
                    flag, job_id, id, min_length = robots[k].calculate_cost_hetro((str(robots[i].job_done[j]) + ','), dist_matrix, current_cost, len(robots[i].job_done))  # the simple id tells the location of where this new job needs to be entered.
                    if flag and robots[k].current_cost < current_cost:  # if the new robot uses lesser resources than any previous robot then assign the task to this robot
                        current_cost = robots[k].current_cost
                        change_flag = deletion_flag = True #set both the flags marking change and deletion of jobs from original robot
                        index = k  # This index basically is telling which robot will do this particular job
                        insert_id = id  # what location the task should be entered in the current plan
                        if deletion_flag:
                            deletion_flag = False
                            deletion_index.append(j) #if job reassgned then add to the list of indices to be deleted
            if change_flag:
                robots[index].job_done.insert(insert_id, job_id)
                change_flag = False
        # deletion of indices from original robot, only jobs that have been assigned to other robots.
        robots[i].job_done = [m for n, m in enumerate(robots[i].job_done) if n not in deletion_index]


# This one has similar concept for reauction but the cost is computed as the sum of both the cost to a particular job
# and the cost from the particular job to the next one in line. so basically calculating the cost of both the links
# and then looking if we are able to find a better link as a whole.
def re_auction_absolute(robots, dist_matrix):
    for i in range(len(robots)):
        current_cost = 0
        cost_to = 0
        cost_from = 0
        deletion_index = []
        for j in range(len(robots[i].job_done)):
            deletion_flag = False
            change_flag = False
            if j == 0:  # if first job of this particular robot then find distance from home location
                cost_to = robots[i].job_cost(0, robots[i].job_done[j], dist_matrix)
                cost_from = robots[i].job_cost(robots[i].job_done[j], robots[i].job_done[j + 1], dist_matrix)
            else:  # if not the first job of this robot then find distance from previous job to this job
                cost_to = robots[i].job_cost(robots[i].job_done[j - 1], robots[i].job_done[j], dist_matrix)
                current_cost = cost_to
                if j == len(robots[i].job_done)-1: # if j points to the last job then add the distance to home again
                    cost_from = robots[i].job_cost(0, robots[i].job_done[j], dist_matrix)
                else:   # if j was not pointing the the last job then find the ditance of j to its next job
                    cost_from = robots[i].job_cost(robots[i].job_done[j], robots[i].job_done[j+1], dist_matrix)
                current_cost = cost_to + cost_from
            for k in range(len(robots)):  # running another loop on robots to find cost of all other robots
                if k == i:  # if talking of the same robot then skip
                    # print 'nothing'
                    None

                else:  # for all other robots find the new costs and if it is better then reassign
                    # print 'everything'
                    flag, job_id, id, cost_to_new, cost_from_new = robots[k].calculate_cost_absolute((str(robots[i].job_done[j]) + ','), dist_matrix,cost_to, cost_from)  # the simple id tells the location of where this new job needs to be entered.
                    if flag and cost_to_new < cost_to and cost_from_new < cost_from:  # if the new robot uses lesser resources than any previous robot then assign the task to this robot
                        current_cost = robots[k].current_cost
                        change_flag = deletion_flag = True
                        index = k  # This index basically is telling which robot will do this particular job
                        insert_id = id  # what location the task should be entered in the current plan
                        if deletion_flag:
                            deletion_flag = False
                            deletion_index.append(j)

            if change_flag:
                robots[index].job_done.insert(insert_id, job_id)
                change_flag = False
        robots[i].job_done = [m for n, m in enumerate(robots[i].job_done) if n not in deletion_index]

#***********************************************************************************************************************

#***************************************************** Random **********************************************************
def random(robots, sub_job_dict):
    i = len(sub_job_dict)   #using this i variable to find out the modulo later on for one by one assignments to the robots
    for keyz in sub_job_dict.keys():  # Run the allocation scheme for each and every subtask to be assigned
        i += 1
        assigned = True     # to check if the keyz is assigned or not coming from down below. basically this portion
                            #and while loop is to check for MR type of tasks, no two instances should be allocated to one robot
        data = keyz.split(',')
        while assigned:     # if the number of robots in the team is less then the MR instances generated, this while loop will run for ever
            index = i % len(robots)
            if int(data[0]) not in robots[index].job_done:  # if no instance of current key is already assigned to the robot then assign
                robots[index].job_done.append(int(data[0]))
                assigned = False                # make assigned false so that the while look exists going to the next key value
            else:
                i += 1          # else try the next robot for this particular key value.

# ***********************************************************************************************************************


#****************************************** Fitness Functions **********************************************************
# Computes the final distance based fitness of the auction plan to tell the total cost incurred by the plan using auctions
def final_fitness(robot, cst_mtrx):
    fitness = 0
    value = 0
    for rbt in robot:                       # for each robot in the team
        for j in range(len(rbt.job_done)):  # for all jobs of each individual robot
            if j == 0:                      # if first job then distance from home
                value = cst_mtrx[0][rbt.job_done[j]]
                fitness +=value
            else:                           #for all jobs other than first and last
                value = cst_mtrx[rbt.job_done[j-1]][rbt.job_done[j]]
                fitness += value
            if j==(len(rbt.job_done)-1):    # if last job then distance from home
                value = cst_mtrx[0][rbt.job_done[j]]
                fitness += value
    return fitness

# Computes the final time based fitness of the auction plan to tell the total cost incurred by the plan using auctions
def final_fitness_max_time(robot, cst_mtrx):
    fitness = 0
    fitness_list = []
    value = 0
    for rbt in robot:   #for all the robots
        allocation_list = [int(i.split(',')[0]) for i in rbt.job_done]
        for j in range(len(rbt.job_done)):      # for all the jobs for each individual robot
            if j == 0:
                value = (cst_mtrx[0][allocation_list[j]])/rbt.speed    # if first job then time from home
                fitness +=value
            else:                                                   #for all jobs other than first and last
                value = (cst_mtrx[allocation_list[j-1]][allocation_list[j]])/rbt.speed
                fitness += value
            if j==(len(rbt.job_done)-1):                            # if last job then time for home
                value = (cst_mtrx[0][allocation_list[j]]) / rbt.speed
                fitness += value
        fitness_list.append(fitness)
        fitness = 0
    print fitness_list
    return max(fitness_list)

# Exactly same as final_fitness_max_time, just puttin in here to save the complete list of fitness values for SSI.
# Returns the whole list rather than just the max solution
def final_fitness_list_time(robot, cst_mtrx):
    fitness = 0
    fitness_list = []
    value = 0
    for rbt in robot:   #for all the robots
        allocation_list = [int(i.split(',')[0]) for i in rbt.job_done]
        for j in range(len(rbt.job_done)):      # for all the jobs for each individual robot
            if j == 0:
                value = (cst_mtrx[0][allocation_list[j]])/rbt.speed    # if first job then time from home
                fitness +=value
            else:                                                   #for all jobs other than first and last
                value = (cst_mtrx[allocation_list[j-1]][allocation_list[j]])/rbt.speed
                fitness += value
            if j==(len(rbt.job_done)-1):                            # if last job then time for home
                value = (cst_mtrx[0][allocation_list[j]]) / rbt.speed
                fitness += value
        fitness_list.append(fitness)
        fitness = 0
    print fitness_list
    return fitness_list

#***********************************************************************************************************************

if __name__ == '__main__':
    main()
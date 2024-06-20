from difflib import SequenceMatcher
import copy

# # Implementation of the Michalewicz Paper panelty scheme 3.2.2
def penalty_dynamic(ind_size, ind, rbt_count, generation_count):
    penalty = 0
    invalid = True              # Simple flag used all along to check where the loops are returning from
    rbt_job = [[] for i in range(rbt_count)]  # initializing a list of list for the individual robot jobs for each robot
    start = end = 0
    ind_split = [i.split(",")[0] for i in ind]
    for i in range(rbt_count):      # populating each robots sub tour
        end += ind.robots_assign[i]
        rbt_job[i] = ind_split[start:end]
        start = end
    total_count = 0
    for robot in rbt_job:
        already_counted = []
        for job in robot:
            count = robot.count(job)
            if count > 1 and (job not in already_counted):
                total_count += count
            already_counted.append(job)
    penalty = pow(0.3 * generation_count, 0.4) * pow(total_count,1)
    # print penalty
    return penalty

# Implementation of the Michalewicz Paper panelty scheme 3.2.4
def penalty_adaptive(ind_size, ind, rbt_count, generation_count, last_K_results):
    penalty = 0
    rbt_job = [[] for i in range(rbt_count)]  # initializing a list of list for the individual robot jobs for each robot
    start = end = 0
    ind_split = [i.split(",")[0] for i in ind]
    for i in range(rbt_count):      # populating each robots sub tour
        end += ind.robots_assign[i]
        rbt_job[i] = ind_split[start:end]
        start = end
    total_count = 0
    for robot in rbt_job:
        already_counted = []
        for job in robot:
            count = robot.count(job)
            if count > 1 and (job not in already_counted):
                total_count += count
            already_counted.append(job)

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

## this routine is to check if an indvidual is invalid or not. Using it primarily for panelty_adaptive routine on top.
## will be checking the best individual is invalid or not
def check_invalid(ind_size, ind, rbt_count):
    penalty = 0
    invalid = False              # Simple flag used all along to check where the loops are returning from
    rbt_job = [[] for i in range(rbt_count)]  # initializing a list of list for the individual robot jobs for each robot
    start = end = 0
    ind_split = [i.split(",")[0] for i in ind]
    for i in range(rbt_count):      # populating each robots sub tour
        end += ind.robots_assign[i]
        rbt_job[i] = ind_split[start:end]
        start = end
    for robot in rbt_job:
        if invalid == True:
            break
        for job in robot:
            count = robot.count(job)
            if count > 1:
                invalid = True
                break
    return invalid


# this routine simply checks if the provided solution is invalid or not.
# this is the windowed version and will append previously executed tasks by the robot to find the feasibility of the solution
def check_invalid_window(ind_size, ind, rbt_count, robot_solutions_windowed):
    if len(robot_solutions_windowed) < rbt_count: # if there were lesser robot in previous iterations compared to this iteration
        # then add an empty list into already_attempted.
        for i in range(abs(len(robot_solutions_windowed) - rbt_count)):
            robot_solutions_windowed.append([])
    invalid = False              # Simple flag used all along to check where the loops are returning from
    rbt_job = [[] for i in range(rbt_count)]  # initializing a list of list for the individual robot jobs for each robot
    start = end = 0
    # this is tour conditioning, where am getting rid of sub jobs and all.
    for i in range(rbt_count):  # for each and every robot I read the tour for that robot
        end += ind.robots_assign[i]
        rbt_job[i] = ind[start:end]
        rbt_job[i] = robot_solutions_windowed[i] + rbt_job[i]  # append the previously attempted tasks by this robot
        rbt_job[i] = [int(j.split(",")[0]) for j in
                      rbt_job[i]]  # get rid of all the commas and sub jobs only the job ids
        start = end
    for robot in rbt_job:
        if invalid == True:
            break
        for job in robot:
            count = robot.count(job)        #count the number of occurances of current task in this tour.
            if count > 1:           #if greater than one then the tour is invalid. break from the loop and return invalid
                invalid = True
                break
    return invalid

# ## this routine is to check if an indvidual is invalid or not.
# ## Will also check the tasks that have been attempted in any previous windows
# ## will be checking the best individual is invalid or not
# def check_invalid_window(ind_size, ind, rbt_count, robot_solutions_windowed):
#     if len(robot_solutions_windowed) < rbt_count:  # if the already attempted tasks had lesser number of robots and the new one has more
#         for i in range(abs(len(robot_solutions_windowed) - rbt_count)):  # then add an empty list into already attempted for each robot.
#             robot_solutions_windowed.append([])
#     window_attempted = copy.deepcopy(robot_solutions_windowed)
#     for i, list in enumerate(robot_solutions_windowed):
#         window_attempted[i] = [j.split(',', 1)[0] for j in list]  # isolating the job id from sub_job id
#     penalty = 0
#     invalid = False              # Simple flag used all along to check where the loops are returning from
#     rbt_job = [[] for i in range(rbt_count)]  # initializing a list of list for the individual robot jobs for each robot
#     start = end = 0
#     ind_split = [i.split(",")[0] for i in ind]
#     for i in range(rbt_count):      # populating each robots sub tour
#         end += ind.robots_assign[i]
#         rbt_job[i] = ind_split[start:end]
#         start = end
#     for i, robot in enumerate(rbt_job):
#         if invalid == True:
#             break
#         for job in robot:
#             count = robot.count(job)
#             if count > 1:
#                 # print 'this job is in the already attempted', job, window_attempted[i]
#                 invalid = True
#                 break
#             if job in window_attempted[i]:
#                 # print '******************** This job is inside the previous window for this robot', job, window_attempted[i]
#                 invalid = True
#                 break
#
#     return invalid


# this is my routine function the * 5 panelty function.
def penalty_fix(ind_size, ind, rbt_count, generation_count):
    penalty = 0
    invalid = True              # Simple flag used all along to check where the loops are returning from
    rbt_job = [[] for i in range(rbt_count)]  # initializing a list of list for the individual robot jobs for each robot
    start = end = 0
    ind_split = [i.split(",")[0] for i in ind]
    for i in range(rbt_count):      # populating each robots sub tour
        end += ind.robots_assign[i]
        rbt_job[i] = ind_split[start:end]
        start = end
        already_counted = []
        for job in rbt_job[i]:
            count = rbt_job[i].count(job)
            if job not in already_counted:
                penalty += (count-1) * 5
            already_counted.append(job)
    return penalty


# this is my routine function the * 5 panelty function.
def penalty_fix_window(ind_size, ind, rbt_count, generation_count, robot_solutions_windowed):
    if len(robot_solutions_windowed) < rbt_count:  # if the already attempted tasks had lesser number of robots and the new one has more
        for i in range(abs(len(robot_solutions_windowed)-rbt_count)):  #then add an empty list into already attempted for each robot.
            robot_solutions_windowed.append([])
    window_attempted = copy.deepcopy(robot_solutions_windowed)
    for i,list in enumerate(robot_solutions_windowed):
        window_attempted[i] = [j.split(',', 1)[0] for j in list]    #isolating the job id from sub_job id
    penalty = 0
    rbt_job = [[] for i in range(rbt_count)]  # initializing a list of list for the individual robot jobs for each robot
    start = end = 0
    ind_split = [i.split(",")[0] for i in ind]
    for i in range(rbt_count):      # populating each robots sub tour
        end += ind.robots_assign[i]
        rbt_job[i] = ind_split[start:end]
        rbt_job[i] = window_attempted[i] + rbt_job[i]
        start = end
        # checking for the penalties to be applied for repetation of jobs
        already_counted = []
        for job in rbt_job[i]:
            count = rbt_job[i].count(job)
            # count_attempted = window_attempted[i].count(job)
            # if repetation in current solution
            if count > 1 and (job not in already_counted):
                penalty += (count-1) * 5
            # if conflicting allocation from previous attempted tasks.
            # if job in window_attempted[i]:
            #     penalty += count_attempted*5
            already_counted.append(job)
    return penalty


def penalize_4m_best(best, chromosome):
    best_str = ''.join(best)
    chromo_str = ''.join(chromosome)
    # print best_str
    # print chromo_str
    return (SequenceMatcher(None, best_str, chromo_str).ratio())*10
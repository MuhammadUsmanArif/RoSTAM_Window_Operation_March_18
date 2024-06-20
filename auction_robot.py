class Robot(object):
    job_done = []
    current_job = int(0)
    current_cost = 0
    speed = 0

    def __init__(self, job_done, current_job, speed):
        self.job_done = job_done
        self.current_job = current_job
        self.speed = speed

    def make_robot(job_done, current_job):
        robot = Robot(job_done, current_job)
        return robot


# Simple distance based fitness function Which checks the least cost location for an incoming task in the current plan
# and if there is one compared to the current least cost then returns that location with a flag and the new least cost
    def calculate_cost(self, job, cst_mtrx, min_cost):
        data = job.split(',')  # strip the subjob representation structure off for checking the job in robots current list
        # print self.job_done
        if int(data[0]) in self.job_done:  # if robot already doing some portion of that job, don't assign another
            return False, int(data[0]), 0
        if len(self.job_done) == 0:  # if the robot job list is empty then no need to go through the list
            cost = cst_mtrx[0][int(data[0])]  # just find the distance from home location and return
            self.current_cost = cost
            return True, int(data[0]), 0
        # min_cost = 1000.001
        min_id = 999
        for i in range(len(self.job_done)):  # if robot job list has values then one by one find distance from each and every job and return the least distance with index
            # print (self.job_done[i], data[0])
            cost = cst_mtrx[self.job_done[i]][int(data[0])]
            # print cost
            if cost < min_cost:
                min_cost = cost
                min_id = i
        self.current_cost = min_cost
        return True, int(data[0]), min_id

# distance based fitness function which does the absolute thing. i.e. calculates the value of the whole link, both the
# distance from the job appraoching the job in question and the distance from the next job from the job in question.
# so both distance to and from combined make the absolute cost of attempting that job if there is one absolute cost which
# is less then the previous lowest absolute cost then returns that location with a flag and the new least cost
    def calculate_cost_absolute(self, job, cst_mtrx, min_cost_to, min_cost_from):
        data = job.split(',')  # strip the subjob representation structure off for checking the job in robots current list
        min_change_cost = min_cost_from + min_cost_to
        # print self.job_done
        if int(data[0]) in self.job_done:  # if robot already doing some portion of that job, don't assign another
            return False, int(data[0]), 0
        if len(self.job_done) == 0:  # if the robot job list is empty then no need to go through the list
            cost = cst_mtrx[0][int(data[0])]  # just find the distance from home location and return
            self.current_cost = cost
            return True, int(data[0]), 0
        # min_cost = 1000.001
        min_id = 999
        for i in range(len(self.job_done)):  # if robot job list has values then one by one find distance from each and every job and return the least distance with index
            # print (self.job_done[i], data[0])
            cost_to = cst_mtrx[self.job_done[i]][int(data[0])]
            if i == len(self.job_done)-1:
                cost_from =  cst_mtrx[0][int(data[0])] # find distance from home location and return the value
            else:
                cost_from = cst_mtrx[int(data[0])][self.job_done[i+1]]  # find distance from one job to another and return
            change_cost = cost_to + cost_from
            # print cost
            if change_cost < min_change_cost:
                min_cost_to = cost_to
                min_cost_from = cost_from
                min_id = i
                min_change_cost = change_cost
        self.current_cost = min_change_cost
        return True, int(data[0]), min_id, cost_to, cost_from

# distance based job cost finding routine
# does not handle task repetation for MR type of tasks
    def job_cost(self,prv_job, job, dist_mtrx):
        return dist_mtrx[prv_job][job]

# time based job cost finding routine
# does not handle task repetation for MR type of tasks
    def job_cost_hetro(self, prv_job, job, dist_mtrx):
        return (dist_mtrx[prv_job][job])/self.speed

# time based cost calculation for auction based scheme. Which checks the least cost location for an incoming task in the current plan
# and if there is one compared to the current least cost then returns that location with a flag and the new least cost
    def calculate_cost_hetro(self, job, cst_mtrx, min_cost, min_length):
        data = job.split(',')  # strip the subjob representation structure off for checking the job in robots current list
        # print self.job_done
        if int(data[0]) in self.job_done:  # if robot already doing some portion of that job, don't assign another
            return False, int(data[0]), 0, len(self.job_done)
        if len(self.job_done) == 0:  # if the robot job list is empty then no need to go through the list
            cost = (cst_mtrx[0][int(data[0])])/self.speed  # just find the distance from home location and return
            self.current_cost = cost
            return True, int(data[0]), 0, len(self.job_done)
        # min_cost = 1000.001
        min_id = 999
        for i in range(len(self.job_done)+1):  # if robot job list has values then one by one find distance from each and every job and return the least distance with index
            # print (self.job_done[i], data[0])
            if i == 0:
                cost = (cst_mtrx[0][int(data[0])]) / self.speed  # just find the time it takes from home location and return
            else:
                cost = (cst_mtrx[self.job_done[i-1]][int(data[0])])/self.speed # calculating the time from one job to another
            # print cost
            if cost < min_cost:
                min_cost = cost
                min_id = i
                min_length = len(self.job_done)
            elif cost == min_cost and len(self.job_done) < min_length:
                min_cost = cost
                min_id = i
                min_length = len(self.job_done)
        self.current_cost = min_cost
        return True, int(data[0]), min_id, len(self.job_done)
# This routine basically adds new jobs to a pre-existing job file and have another routine which converts the jobs
# into their appropriate representation

import rospy
import random
import pickle
import sys
import csv
import getopt
from copy import deepcopy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

FileString = "/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/"

def main(alt_jobs):
    print ('job_alteration _________________________________________________________________________________')

# Written for the iterative version of RoSTAM, where this routine adds jobs to the pre-existing job_file.txt file
# right now can do it for ST-SR and ST-MR type of jobs. Once done the job representation routine has to be called again
# to convert the job from simple types to chromosome representation type.
def add_job_simple(add_job,types, FileString, map_dim):
    job_file = open(FileString + 'job_file.txt', 'a')
    old_jobs = pickle.load(open(FileString + 'jobs.pkl', 'rb'))
    l = [(x, y) for x in range(map_dim*(-1), map_dim, 1) for y in range(map_dim*(-1), map_dim, 1)]
    l_new = [item for item in l if item not in old_jobs]

    assigned_jobs = list()
    for job in range(1, add_job + 1):
        random_task = random.sample(l_new,1)
        l_new.remove(random_task[0])
        assigned_jobs.append(random_task[0])
        for index in range(0, 3):
            if index == 2:  # This tells the job type
                job_file.write(str(random.randint(1, types)))
            else:
                job_file.write(str(assigned_jobs[job - 1][index]) + ',')  # list index out of range error might be prompted if generate more than 64 jobs in range -4,4
        job_file.write("\n")
    job_file.close()
    complete_jobs = old_jobs
    complete_jobs.extend(assigned_jobs)
    job_pkl = open(FileString + "jobs.pkl", 'wb')
    pickle.dump(complete_jobs, job_pkl)
    job_pkl.close()

def job_representation_simple(FileString):
    id2 = 1  # manages the subtask number
    id1 = 1  # manages the total number of jobs after subtask seperation
    job_file = open(FileString + 'job_representation.txt', 'a')
    job_file.seek(0)
    job_file.truncate()
    # txt_file = open (FileString + 'job_representation.txt', 'wb')
    # csv_file = r"/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/Round_20thJune/Prnt_Trnm3_Surv_AIS/job_representation.csv"
    with open(FileString + 'job_file.txt') as f:
        for line in f:
            data = line.split(',')
            for i in range(0, int(data[2])):  # iterate through the for loop for subjobs
                job_file.write(str(id2) + ',' + str(id1) + ',' + str(data[0]) + ',' + str(data[1] + ',' + '\n'))
                # for job representation the job_id, sub_job_id, x_coordinates, y_coordinates.
                # job_file.write(',')
                # job_file.write(str(id2))  # id2 linking the subjobs of a singel job up
                # job_file.write(',')
                # job_file.write(str(data[0]))  # The x value for the job
                # job_file.write(',')
                # job_file.write(str(data[1]))  # the y value for the job
                # job_file.write(',')
                # job_file.write("\n")
                id1 += 1
            id1 = 1
            id2 += 1
    job_file.close()
    # in_txt = csv.reader(open(txt_file, "rb"), delimiter=',')
    # out_csv = csv.writer(open(csv_file, 'wb'))
    # out_csv.writerows(in_txt)


if __name__ == '__main__':
    main()



#!/usr/bind/env python
## this is my first random job generation routine. It generated x,y coordinates of the jobs and also assigned robots to that job.
## The format is (x, y, robot #)
import rospy
import random
import pickle

FileString = "/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/"
def main(jobs,types, FileString, dim1, dim2):

    ## jobs are number of jobs to be generated, type is for MR type of jobs if MR type then this number represents how
    ## many job types should be there for SR jobs this number is 1, FileString is the location where the data is to be
    ## saved, dime1 and dim2 are the ranges in which the jobs are to be generated dim1 is the
    ## smaller number and dim2 is the larger one. for x (dim1, dim2) for y (dim1, dim2)

    # #this version of code is commented on 29July2017. will produce a job text file which has chances of job repetation.
    # # job repetation is causing me problems while plotting the final solution on a 2d xy coordinates. and might coz
    # # problems while defending my logic.
    # pointNangle = [0.0, 0.0, 0.0]   #not exactly using this right now
    # job_list = [pointNangle]        #not exactly using this right now either
    #
    # #for point in job_list:
    # #job_list[point] = 2
    # job_file = open('/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/job_file.txt', 'a')
    # job_file.seek(0)
    # job_file.truncate()
    # for job in range(1,jobs+1):
    #     for index in range(0,3):
    #         if index == 2:              #This tells the job type
    #             # job_file.write(str(1))
    #             job_file.write(str(random.randint(1,types)))
    #         else:
    #             job_file.write(str(random.randint(-4,4)) + ',')  #job_file.write(str(random.randint(-4,4)) + ' ') This is the one for giving space with jobs
    #     job_file.write("\n")
    #
    # job_file.close()

    # This version is created on 29July 2017. creates an ordered pair list of xy values and after shuffling it takes out
    # desired number of job locations from it. will not repeat any ordered pair, but will not produce jobs more than 64
    # as thats the max # of order pair you get in -4,4. if want to get more increase the size of your map
    # generating a list of xy ordered pairs in which each pair is unique. so no jobs are repeated.
    l = [(x, y) for x in range(dim1, dim2, 1) for y in range(dim1, dim2, 1)]
    random.shuffle(l)  # shuffling it for random picking of jobs
    # pointNangle = [0.0, 0.0, 0.0]  # not exactly using this right now
    # job_list = [pointNangle]        #not exactly using this right now either
    # #for point in job_list:
    # #job_list[point] = 2
    job_file = open(FileString + 'job_file.txt', 'a')
    job_file.seek(0)
    job_file.truncate()
    assigned_jobs = list()
    for job in range(1, jobs + 1):
        assigned_jobs.append(l[job])
        for index in range(0, 3):
            if index == 2:  # This tells the job type
                # job_file.write(str(1))
                job_file.write(str(random.randint(1, types)))
            else:
                job_file.write(str(l[job - 1][index]) + ',')  # list index out of range error might be prompted if generate more than 64 jobs in range -4,4
        job_file.write("\n")
    job_file.close()
    job_pkl = open(FileString + "jobs.pkl", 'wb')
    pickle.dump(assigned_jobs, job_pkl)



if __name__ == '__main__':
    main()

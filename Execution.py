import os
import sys
import timeit
import job_alteration
import time
import SSI_auction_Sven
import Deletion
import Seed_creation

# while switching between hetro and homogeneous robots do take care of the speed values. The speed values for homo
# robots are made 1,1,1 and for hetro are altered to what ever the desired values are. Which means that homogeneous robot

# run as a special case of hetrogeneous robots.
sys.path.append('/home/usman/research_gaz_ros/src/turtlebot_research/src/charm/execute_ST-SR_MR-IA_TA')
sys.path.append("/home/usman/research_gaz_ros/src/turtlebot_research/src")
import reader_classbased_dict
import reader_advanced

FileString = "/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/"

# not using this variable all the time, pay attention if you are passing this to the python file deap__init.py
# robot_num=3

def main():
    # jobs = 50             #when you are running for a fix number of tasks then comment the first for loop and reverse
                            #indent the other for loop.

    # This portion is initiated to write the average of all the 5 executions (for one job number) onto an average file

    # average_SSI = open('/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/' + 'Avg_SSI.txt','a')
    # SSI_fitness = open('/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/' + 'SSI_solution.txt','r+')
    average_fitness = open('/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/' + 'Avg_Fitness_MultipleRun.txt','a')
    fitness_allrecord = open('/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/' + 'All_record_MultipleRun.txt','a')
    execution_time = open('/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/' + 'Time_avg.txt','a')
    average_fitness.seek(0)
    average_fitness.truncate()
    fitness_allrecord.seek(0)
    fitness_allrecord.truncate()
    execution_time.seek(0)
    execution_time.truncate()

    for a in range(10,55,5):    #this for loop is basically for the tasks, running through a range of tasks with step size 5
        average_fitness = open('/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/' + 'Avg_Fitness_MultipleRun.txt','a')
        fitness_allrecord = open('/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/' + 'All_record_MultipleRun.txt','a')
        execution_time = open('/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/' + 'Time_avg.txt', 'a')
        fitness_allrecord.write('*******************' + str(a) + " Jobs **********************" +'\n')
        # And delete those 5 executions so that the new job number can be run 5 times.
        for i in range(0,5):    #running 5 iterations for each task distribution
            start_time = timeit.default_timer()
            # population = (a / 10) * 10
            # generation = a * 150

            for r in range(0,5):
                ##For ST-SR-IA Without the for loop for variable population size and generation number
                # os.system("python Darrah_2015.py -r 3 -g " + str('1000') + " -p " + str('50') + " -j " + str(a) + " -i " + str(i))  # the i is for iterations coming from the iterative version of ST-SR-IA.
                ##For ST-MR-IA With the for loop for variable population size and generation number
                # if a >= 70:
                #   os.system("python deap__init.py -r 5 -g 3000 -p 100 -j "+str(a) + " -i "+str(i)) # the i is for iterations coming from the iterative version of ST-SR-IA. not really needed in our case over here.
                    # os.system("python GA_solo.py -r 5 -g " + str('6000') + " -p " + str('60') + " -j " + str(a) + " -i " + str(i))  # the i is for iterations coming from the iterative version of ST-SR-IA. not really needed in our case over here.
                # elif a >= 65:
                #     os.system("python GA_solo.py -r 5 -g " + str('6000') + " -p " + str('60') + " -j " + str(a) + " -i " + str(i))  # the i is for iterations coming from the iterative version of ST-SR-IA. not really needed in our case over here.
                # elif a >= 40:
                #     os.system("python GA_solo.py -r 5 -g " + str('5000') + " -p " + str('50') +" -j " + str(a) + " -i " + str(i))  # the i is for iterations coming from the iterative version of ST-SR-IA. not really needed in our case over here.
                # elif a >= 25:
                #     os.system("python GA_solo.py -r 5 -g " + str('4000') + " -p " + str('50') + " -j " + str(a) + " -i " + str(i))  # the i is for iterations coming from the iterative version of ST-SR-IA. not really needed in our case over here.
                # else:
                #     os.system("python GA_solo.py -r 5 -g " + str('4000') + " -p " + str('50') + " -j " + str(a) + " -i " + str(i))  # the i is for iterations coming from the iterative version of ST-SR-IA. not really needed in our case over here.
                #
                if a >= 70:
                    os.system("python Darrah_2015.py -r 5 -g " + str('6000') + " -p " + str('60') + " -j " + str(a) + " -i " + str(
                        i))  # the i is for iterations coming from the iterative version of ST-SR-IA. not really needed in our case over here.
                elif a >= 65:
                    os.system("python Darrah_2015.py -r 5 -g " + str('6000') + " -p " + str('60') + " -j " + str(
                        a) + " -i " + str(
                        i))  # the i is for iterations coming from the iterative version of ST-SR-IA. not really needed in our case over here.
                elif a >= 40:
                    os.system("python Darrah_2015.py -r 5 -g " + str('5000') + " -p " + str('50') + " -j " + str(
                        a) + " -i " + str(
                        i))  # the i is for iterations coming from the iterative version of ST-SR-IA. not really needed in our case over here.
                elif a >= 25:
                    os.system("python Darrah_2015.py -r 5 -g " + str('4000') + " -p " + str('50') + " -j " + str(
                        a) + " -i " + str(
                        i))  # the i is for iterations coming from the iterative version of ST-SR-IA. not really needed in our case over here.
                else:
                    os.system("python Darrah_2015.py -r 5 -g " + str('4000') + " -p " + str('50') + " -j " + str(
                        a) + " -i " + str(
                        i))  # the i is for iterations coming from the iterative version of ST-SR-IA. not really needed in our case over here.


            # ********************************   SSI   *******************************************
            # SSI_auction_Sven.main(FileString + str(a) + '/' + str(i) + '/', )
            # Deletion.main(FileString+str(a)+'/'+str(i)+'/', ) # this routine made for deleting unwanted files after execution
            # ***********************************************************************************

            # termination of gazebo and related nodes at the end of every execution. Wonder why!
            # os.system("killall roslaunch")
            # time.sleep(2)     # giving some delay for things to settle down
            # os.system("killall rosmaster")
            # time.sleep(10)


            elapsed_time = timeit.default_timer() - start_time
            print elapsed_time
            print '******************************************************'
            execution_time.write(str(elapsed_time/5.0) + '\n')

        fitness_file = open('/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/' + 'RoSTAM_Results.txt', 'r+')

        data = 0
        for line in fitness_file:
            data += float(line)
            fitness_allrecord.write(str(float(line)) + '\n')
        data = data/25
        average_fitness.write(str(data) + '\n')
        fitness_file.seek(0)
        fitness_file.truncate()
        fitness_file.close()
        average_fitness.close()
        fitness_allrecord.close()


        # data = 0
        # for line in SSI_fitness:
        #     data += float(line)
        # data = data / 5
        # average_SSI.write(str(data) + '\n')
        # SSI_fitness.seek(0)
        # SSI_fitness.truncate()
        # SSI_fitness.close()
    fitness_allrecord.close()
    execution_time.close()
    average_fitness.close()

            # job_alteration.add_job_simple(15,1)     #Number of jobs and number of types allowed in the job
    # reader_classbased_dict.job_representation()
    # os.system("python deap__followup.py -r 5 -g 1000 -p 100 -j 35 -i 0")  # the i is for iterations coming from the iterative version of ST-SR-IA. not really needed in our case over here.


if __name__ == "__main__":
    main()
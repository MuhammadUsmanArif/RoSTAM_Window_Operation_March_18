# I have created this additional python and put in most of the parsers of different types for easier referencing.
# in the main routine right now is the ampl converter which basically alters my file for easier ampl copy paste

import ast
import pickle
import NeosXML

FileString = "/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/"
FileString_excel = '/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/'
leftover_excel = ''


def main():
    for_ampl(50, FileString_excel)


# used by surface_plot to get dist matrix
def read_distmatrix(FileString):
    file_mtrx = open(FileString+'dist_matrix.txt').read()
    file_mtrx = file_mtrx.replace("[", "")      #removing the brackets and all kachra
    file_mtrx = file_mtrx.replace("]", "")
    file_mtrx = file_mtrx.replace(" ", "")
    dist_matrix = file_mtrx.splitlines()        #reading the file into the matrix line by line
    for i in range(0, len(dist_matrix)):        #seperating individual values within lines
        dist_matrix[i] = map(ast.literal_eval, dist_matrix[i].split(','))
    return dist_matrix
# used by surface_plot to get dist matrix
# This read solution is a copy from the get_job() routine below, I don't think I made much changes into it
# provides only job idz at the end, no instance added i.e. only 9 not 9,1
def read_solution():
    solution_file = open(FileString + "solution.txt", 'rb')
    # reading the first line of the solutions file which holds the 1st part of the chromosome.
    solution = solution_file.readline()
    solution = solution.replace("\n", "")
    solution = solution.replace("[", "")
    solution = solution.replace("]", "")
    solution = solution.replace("'", "")
    solution = solution.replace(" ", "")
    solutions = solution.split(',')
    solutions = solutions[0::2]         # reading every 2nd entry of the processed line, dropping the instances, only reading task idz
    solutions = map(int, solutions)
    # reading the 2nd portion of the file holding the robot distributions.
    line2 = solution_file.readline()
    line2 = line2.replace("[", "")
    line2 = line2.replace("]", "")
    line2 = line2.replace("\n", "")
    robot_assign = map(int, line2.split(","))
    return solutions,robot_assign

# Used in the plot_TSP.py code to get the job dict and solution file
def get_job(FileString):
    job_dict = pickle.load(open(FileString + 'job_dict.pkl', 'rb'))
    solution_file = open(FileString + "solution.txt", 'rb')

    #reading the first line of the solutions file which holds the 1st part of the chromosome.
    solution = solution_file.readline()
    solution = solution.replace("\n", "")
    solution = solution.replace("[", "")
    solution = solution.replace("]", "")
    solution = solution.replace("'", "")
    solution = solution.replace(" ", "")
    solutions = solution.split(',')
    solutions = solutions[0::2]

    #reading the 2nd line of the solutions file which holds the robot distribution(2nd part of the chromosome)
    line2 = solution_file.readline()
    line2 = line2.replace("[","")
    line2 = line2.replace("]", "")
    line2 = line2.replace("\n", "")
    robot_assign = map(int,line2.split(","))

    #Going into isolating the job assignments for each robot so that the coordinates can be later read from dict
    start = 0
    end = 0
    robot_jobs = [None] *len(robot_assign)
    for i in range (len(robot_assign)):
        end += robot_assign[i]
        robot_jobs[i] = solutions[start:end]
        start=end

    return job_dict,robot_jobs


# making this routine to convert my conventional distance matrix into AMPL inputable text file, which i can load in
# Notepad ++ and copy paste into ampl
def for_ampl(robots, tasks, FileString_excel):
    # tasks = 40
    dist_mtrx = []

    dist_mtrx_file = open(FileString_excel +  leftover_excel + "dist_matrix.txt", 'rb')
    processed_dist = open(FileString_excel +  leftover_excel + "dist_matrix_processed.txt", 'wb')
    processed_dist.writelines("param tasks:=" + str(tasks+1) + ";\n")
    processed_dist.writelines("param robots:=" + str(robots)+ ";\n")
    processed_dist.writelines("param S : 1  2   3:=\n")
    processed_dist.writelines("1    1   1   1;\n")
    processed_dist.write("param D :")
    for i in range(1,tasks+2):
            processed_dist.write(' ' + str(i))
    processed_dist.writelines(":=\n")
    i = 1               # using i for simple serial numbering of each line.
    for line in dist_mtrx_file:
        line = line.replace("[", "\t")
        line = line.replace(",", "\t")
        line = line.replace("]", "")
        line = str(i)+'\t' +line
        i+=1
        processed_dist.writelines(str(line))
    processed_dist.write(';')
    dist_mtrx_file.close()
    processed_dist.close()

# making this routine to convert my conventional distance matrix into AMPL inputable text file, which i can load in
# Notepad ++ and copy paste into ampl
# Routine updated on 3rd April 2020
def for_ampl_updated(robots, failed_robots, tasks, FileString_excel, dist_matrix, instance, window):
    # this portion is basically opening the file and giving it the initial shape for amplied

    processed_dist = open(FileString_excel +  leftover_excel + "dist_matrix_processed_" + str(window) + ".txt", 'wb')
    processed_dist.writelines("param tasks:=" + str(tasks) + ";\n")
    processed_dist.writelines("param robots:=" + str(sum(failed_robots))+ ";\n")
    processed_dist.writelines("param S :")
    for i in range(sum(failed_robots)):
        processed_dist.write('  ' + str(i+1))
    processed_dist.writelines(":=\n")
    processed_dist.writelines("1")
    for i in range(sum(failed_robots)):
        processed_dist.write("  1")
    processed_dist.write(";\n")
    processed_dist.write("param D :")

    # now writing the distance matrix with the given format
    for i in range(1,tasks+1):
            processed_dist.write(' ' + str(i))  #these are the column headers i.e. the task numbers
    processed_dist.writelines(":=\n")
    for i,lst in enumerate(dist_matrix):    # first the entry number of the parameters
        processed_dist.write(str(i+1))
        for value in lst:
            processed_dist.write('\t' + str(value)) # these are the distance values proceeded by a tab for each entry
        processed_dist.write('\n')
    processed_dist.write(';')
    processed_dist.close()

    NeosXML.create_xml(sum(failed_robots), FileString_excel, instance, window)


# reads speed values if saved by deap__init.py, can be used by auctions later on.
def read_speed():
    speed_file = open(FileString + 'speed.txt', 'rb')
    data = speed_file.read()
    data = data.strip(']')
    data = data.strip('[')
    # data = data[:-3]    # couldn't get rid of the last ] so doing this instead
    data = data.strip('\n')
    data.strip(']')
    speed_values = data.split(',')
    speed_values = map(float, speed_values)
    return speed_values


if __name__ == '__main__':
    main()
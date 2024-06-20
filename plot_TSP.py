""" This portion of code is taken from https://gist.github.com/payoung/6087046
which is basically plotting multiple solutions for the same set of coordinates"""

import matplotlib.pyplot as plt
import sys
import random
import numpy as np
import parsers

FileString = "/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/"

def graph_plot(minm, avge, off_avge, off_min, generation_count, Jobs, robot_num, NGEN, POP_SIZE, iteration):
    # ****************************** GRAPH PLOTTING ****************************
    fig = plt.figure(figsize=(50, 30))
    x = list(range(0, generation_count))  # x has the number of generations
    plt.plot(x, minm, 'g', linewidth=3, label='Best so Far(Minimum)')  # 1 y plot has the BSF curve against x
    plt.plot(x, avge, 'b', linewidth=3, label='Average')  # other y plot has avg. against x

    # plt.plot(x, off_avge, 'r', linewidth=3, label='Off_avg')  # other y plot has avg. against x
    # plt.plot(x, off_min, 'm', linewidth=3, label='Off_min')  # other y plot has avg. against x
    plt.grid(True)
    plt.legend(loc='upper left', fontsize=30)
    plt.title("Graph for" + str(Jobs) + 'Jobs ' + str(robot_num) + 'robots', fontsize=40)
    plt.ylabel('Fitness (minimum)', fontsize=30)
    # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # this is a huge jugaar, m 100% sure that there are better ways of adjusting the tick size and gridding.
    # m just in a hurry to plot my graphs
    plt.xticks(range(0, generation_count, 100), fontsize=20)
    plt.yticks(range(int(min(minm)), int(max(minm)), 5), fontsize=20)
    # plt.yticks(range(35, int(max(avge)), 5), fontsize=20)
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    plt.xlabel('Generation', fontsize=30)

    # for tick in fig.xaxis.get_major_ticks():
    #     tick.label.set_fontsize(14)
    #     specify integer or one of preset strings, e.g.
    #     tick.label.set_fontsize('x-small')
    # tick.label.set_rotation(
    #     'vertical')
    fig.savefig(FileString + '/' + str(Jobs) + '/' + str(iteration) + '/' + str(NGEN) + 'G_' + str(POP_SIZE) + 'P_' + str(Jobs) + 'J_' + str(robot_num) + 'R.png')
    # plt.show()
    plt.close(fig)


def plotTSP(paths, points, num_iters, dim1, dim2):

    """
    path: List of lists with the different orders in which the nodes are visited
    points: coordinates for the different nodes
    num_iters: number of paths that are in the path list

    """
    # Unpack the primary TSP path and transform it into a list of ordered
    # coordinates

    line_type = ['solid' , 'dashed', 'dashdot', 'dotted' ]
    colorz = ['blue', 'pink', 'green', 'orange','red', 'black', 'purple', 'brown']

    x = []; y = []
    for i in (paths[0]):
        x.append(points[i][0])
        y.append(points[i][1])

    # Set axis too slitghtly larger than the set of x and y
    # plt.xlim(-max(x)*1.2,max(x)*1.2)
    # plt.ylim(-max(y) * 1.2, max(y) * 1.2)
    #setting the size of the plot
    plt.xlim(dim1,dim2)      # commenting the top portion and fixing the range of map, if increase the size of map would
    plt.ylim(dim1,dim2)      # have to change these values.

    #picking a color and line sequencex
    clr = colorz[num_iters%8]
    linez = line_type[num_iters%4]

    #plotting all the cities/tasks
    plt.plot(x, y, 'co', color=clr)

    # Set a scale for the arrow heads (there should be a reasonable default for this, WTF?)
    a_scale = .2


    # this line makes a link between the first city and the last. I don't really think its needed by my problem so commenting
    # plt.arrow(x[-1], y[-1], (x[0] - x[-1]), (y[0] - y[-1]), head_width = a_scale,color =clr, length_includes_head=True, ls=linez)

    #making lines one at a time.
    for i in range(0,len(x)-1):
        plt.arrow(x[i], y[i], (x[i+1] - x[i]), (y[i+1] - y[i]), head_width = a_scale,color =clr, length_includes_head = True, ls=linez) #If there is error of index due to this line its probably coz of having two instances of a task next to one another. like 16,1 16,2 and subtraction of those two gives a zero

def main(FileString, dim1, dim2, graph_number): #adding this graph_number part for the followup implementaion where I try and add multiple graphs into a single folder.
    job_dict, robot_jobs = parsers.get_job(FileString)

    fig = plt.figure(figsize=(20, 20))
    fig.suptitle(('\n\n' + str(len(job_dict)) + ' Jobs, EA Best Solution For ' +str(len(robot_jobs)) + ' Robots'), fontsize=14, fontweight='bold')
    #will run this for loop the number of time equal to robots. the # of robots can be found from the len of robot_jobs
    for i in range(len(robot_jobs)): # for each robot
        x_cor = list()
        y_cor = list()
        path = list()
        #for each job entry in the robot_jobs
        for j in range(len(robot_jobs[i])): # for all the jobs inside it
            data = job_dict[robot_jobs[i][j]]
            x_cor.append(int(data[1]))  #find the x coordinate
            y_cor.append(int(data[2]))  #find the y coordinate
            path.append(j)
        points = []
        for k in range(0, len(x_cor)):
            points.append((x_cor[k], y_cor[k]))
        print points

    # Pack the paths into a list
        paths = [path]
        plotTSP(paths, points, i, dim1, dim2) #calling the plotting function
    # plt.show()  #putting the plotting function on so that all the paths are shown together. otherwise only 1 robot will be shown
    fig.savefig(FileString + 'route' + str(graph_number) + '.png')


def TSP_plot_window(FileString, dim1, dim2, graph_number, window_size, window_assignment): #adding this graph_number part for the followup implementaion where I try and add multiple graphs into a single folder.
    job_dict, robot_jobs = parsers.get_job(FileString)
    robot_jobs = window_assignment
    fig = plt.figure(figsize=(20, 20))
    fig.suptitle(('\n\n' + str(len(job_dict)) + ' Jobs, EA Best Solution For ' +str(len(robot_jobs)) + ' Robots'), fontsize=14, fontweight='bold')
    #will run this for loop the number of time equal to robots. the # of robots can be found from the len of robot_jobs
    for i in range(len(robot_jobs)): # for each robot
        x_cor = list()
        y_cor = list()
        path = list()
        #for each job entry in the robot_jobs
        for j in range(len(robot_jobs[i])): # for all the jobs inside it
            target_string = robot_jobs[i][j]
            target_string = target_string.split(',')
            data = job_dict[target_string[0]]
            x_cor.append(int(data[1]))  #find the x coordinate
            y_cor.append(int(data[2]))  #find the y coordinate
            path.append(j)
        points = []
        for k in range(0, len(x_cor)):
            points.append((x_cor[k], y_cor[k]))
        print points

    # Pack the paths into a list
        paths = [path]
        plotTSP(paths, points, i, dim1, dim2) #calling the plotting function
    # plt.show()  #putting the plotting function on so that all the paths are shown together. otherwise only 1 robot will be shown
    fig.savefig(FileString + 'route' + str(graph_number) + '.png')


if __name__ == '__main__':
    main()
## this is where I create 2 xmls to upload on NEOS server and record the results.
## the results are recording in two different files, one only saves the final objective function
## the other file records the complete output of the optimization run.
import lxml.etree as ET
import subprocess

def create_xml(robot_count, file_string, iteration, window):

    ## this part is handling creation of xml for FilMINT
    problem = ET.Element('problem')
    ET.SubElement(problem, "category").text = "minco"
    ET.SubElement(problem, "solver").text = 'FilMINT'
    ET.SubElement(problem, "inputType").text = "AMPL"
    ET.SubElement(problem, "priority").text = "long"
    # ET.SubElement(problem, "email").text = "mailusman@hotmail.com"
    model = ET.SubElement(problem, "model")
    if robot_count == 3:    #if there are no robot failures for Rbt_fail based experiment or there are 4 robots in the robot_add experiment but one is failed go for the 3 robot ampl file
        model_file = open('model_3rbt.txt', 'r')
    elif robot_count == 2:  # if there are two active robots in total go for 2 active robot experiment.
        model_file = open('model_2rbt.txt', 'r')

    model_lines = model_file.read()
    model.text = ET.CDATA(str(model_lines))

    # next read the distance file which I have saved as distance matrix processed_1, _2, _3.... The 1 2 3 will be managed by the window attribute paseed.
    data = ET.SubElement(problem, "data")

    data_file = open(file_string + "dist_matrix_processed_" + str(window) + ".txt", 'r')
    data_lines = data_file.read()

    data.text = ET.CDATA(str(data_lines))

    # this is the command part of the xml where I have hard coded them.
    # if the execution is running on 2 robots then the last line over here might prompt an error in the filMINT output file
    # but it doesn't matter, the results would still be fine.
    command = ET.SubElement(problem, "commands")
    command.text = ET.CDATA('''solve;
    show;
    display _varname, _var;
    display x;
    display objective_function;
    display sum{i in city, j in city} x[i,j,1]*D[i,j];
    display sum{i in city, j in city} x[i,j,2]*D[i,j];
    display sum{i in city, j in city} x[i,j,3]*D[i,j];
    ''')

    tree = ET.ElementTree(problem)
    tree.write("trial.xml")

    Knitro = ET.Element('Knitro')
    ET.SubElement(Knitro, "category").text = "minco"
    ET.SubElement(Knitro, "solver").text = 'Knitro'
    ET.SubElement(Knitro, "inputType").text = "AMPL"
    ET.SubElement(Knitro, "priority").text = "long"
    # ET.SubElement(problem, "email").text = "mailusman@hotmail.com"
    model = ET.SubElement(Knitro, "model")
    if robot_count == 3:
        model_file = open('model_3rbt.txt', 'r')
    elif robot_count == 2:
        model_file = open('model_2rbt.txt', 'r')

    model_lines = model_file.read()
    model.text = ET.CDATA(str(model_lines))

    data = ET.SubElement(Knitro, "data")

    data_file = open(file_string + "dist_matrix_processed_" + str(window) + ".txt", 'r')
    data_lines = data_file.read()

    data.text = ET.CDATA(str(data_lines))

    command = ET.SubElement(Knitro, "commands")
    command.text = ET.CDATA('''solve;
    show;
    display _varname, _var;
    display x;
    display objective_function;
    display sum{i in city, j in city} x[i,j,1]*D[i,j];
    display sum{i in city, j in city} x[i,j,2]*D[i,j];
    display sum{i in city, j in city} x[i,j,3]*D[i,j];
    display sum{i in city, j in city} x[i,j,4]*D[i,j];
    ''')

    tree = ET.ElementTree(Knitro)
    tree.write("trial_knitro.xml")

    execute_Neos_filmint(robot_count, file_string, iteration, window)
    execute_Neos_knitro(robot_count, file_string, iteration, window)


def execute_Neos_knitro(robot_count, file_string, iteration, window):
    return_value = subprocess.Popen(['python', 'NeosClient.py', 'trial_knitro.xml'], stdout=subprocess.PIPE)
    stdout = return_value.communicate()[0]  # saving all the output from shell into a string named stdout.
    output_file = open(file_string + 'Knitro_OUT_' + str(window), 'w') #saving the standard ampl output to a file within the given folder with winodow # appended
    output_file.write(stdout)
    output_file.close()
    result_file = open(file_string + 'outputKnitro', 'a')
    for line in stdout.split('\n'):  # splitting stdout on the basis of new lines and iterating over them
        if 'objective_function =' in line:  # looking for the objective_function value
            obj_value = float(line.split(' ')[-1])
            result_file.write(str(obj_value) + '\n')
            print (str('*****************This is the Knitro output**********************'+ str(obj_value)))
            result_file.close()
            return
    result_file.write(str(000.000) + '\n')
    print (str('*****************This is the Knitro output**********************' + 'No Response'))
    result_file.close()
    return
    # print 'STDOUT:{}'.format(stdout)  # this line would print the whole stdout string

def execute_Neos_filmint(robot_count, file_string, iteration, window):
    return_value = subprocess.Popen(['python', 'NeosClient.py', 'trial.xml'], stdout=subprocess.PIPE)
    stdout = return_value.communicate()[0]  # saving all the output from shell into a string named stdout.
    output_file = open(file_string + 'FilMINT_OUT_' + str(window),
                       'w')  # saving the standard ampl output to a file within the given folder with winodow # appended
    output_file.write(stdout)
    output_file.close()
    result_file = open(file_string + 'outputFilMINT', 'a')
    for line in stdout.split('\n'):  # splitting stdout on the basis of new lines and iterating over them
        if 'objective_function =' in line:  # looking for the objective_function value
            obj_value = float(line.split(' ')[-1])
            result_file.write(str(obj_value) + '\n')
            print (str('*****************This is the filMINT output**********************' + str(obj_value)))
            result_file.close()
            return
    result_file.write(str(000.000) + '\n')
    print (str('*****************This is the filMINT output**********************' + 'No Response'))
    result_file.close()
    return
# create_xml(3, '/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/50/11/', 1, 5)


# problem = ET.Element('problem')
# ET.SubElement(problem, "category").text = "minco"
# ET.SubElement(problem, "solver").text = 'Knitro'
# ET.SubElement(problem, "inputType").text = "AMPL"
# ET.SubElement(problem, "priority").text = "long"
# ET.SubElement(problem, "email").text = "mailusman@hotmail.com"
# model = ET.SubElement(problem, "model")
#
# model.text = ET.CDATA('''param tasks;
# param robots;
# set city = 1..tasks;
# set DEST = 2..tasks;
# set ROBO = 1..robots;
# param D{city,city};
# param S{ROBO};
# var U{city} >= 0 integer;
# var x{city,city,ROBO} binary;
# param N := card(city);
#
#
# minimize objective_function:
#             max(sum{i in city, j in city}	x[i,j,1]*D[i,j], sum{i in city, j in city}	x[i,j,2]*D[i,j], sum{i in city, j in city}	x[i,j,3]*D[i,j]);
#
#
#
# subject to depart{k in ROBO}:			# m robots leave depot
#     sum{j in city} x[1,j,k] = 1;
#
# subject to return{k in ROBO}:			# m robots return to depot
#     sum{j in city} x[j,1,k] = 1;
#
# subject to noreturn{i in city, k in ROBO}:		# robots can't depart and arrive to same location
#     x[i,i,k] = 0;
#
# subject to routecontinuity{p in city, k in ROBO}:
#     sum{i in city} x[i,p,k] - sum{j in city} x[p,j,k] = 0;
#
# subject to singleentrance{j in DEST}: 	# all nodes are entered exactly once
#     sum{i in city} sum{k in ROBO} x[i,j,k] = 1;
#
# subject to singleexit{i in DEST}:	# all nodes are exited exactly once
#     sum{j in city} sum{k in ROBO} x[i,j,k] = 1;
#
# subject to c3{l in ROBO, k in city, j in city: j > 1 and k > 1}:  			#no subtours
#           U[j] - U[k] + N*x[j,k,l] <= N-1;
#
# ''')
#
#
# data = ET.SubElement(problem, "data")
#
# data.text = ET.CDATA('''param tasks:=5;
# param robots:=3;
# param S : 1  2   3:=
# 1    1   1   1;
# param D : 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16:=
# 1	0	2.25937	2.0	2.0	3.0	3.1769	1.41428	3.28745e-07	3.21671	2.25928	1.0	3.0	1.41422	4.476	2.24039	1.0
# 2	0	0	3.60682	4.13812	5.12221	5.41733	2.23937	2.24387	1.0	4.0	2.82843	1.41428	1.0	6.40418	3.16666	2.0
# 3	0	3.60818	0	2.82845	3.60768	3.2161	1.41422	2.0	4.24265	3.60673	1.0	3.60729	3.16755	2.82843	1.0	3.0
# 4	0	4.18747	2.82848	0	1.0	1.41422	3.17728	2.0	5.18436	1.0	2.24407	5.0	3.21671	4.0	3.60653	2.25937
# 5	0	5.18436	3.60653	1.0	0	1.0	4.13825	3.0	6.1811	1.41422	3.17728	6.0	4.18747	4.12747	4.49671	3.21671
# ;''')
#
# command = ET.SubElement(problem, "commands")
# command.text = ET.CDATA('''solve;
# show;
# display _varname, _var;
# display x;
# display objective_function;
# display sum{i in city, j in city} x[i,j,1]*D[i,j];
# display sum{i in city, j in city} x[i,j,2]*D[i,j];
# display sum{i in city, j in city} x[i,j,3]*D[i,j];
# ''')
#
# tree = ET.ElementTree(problem)
# tree.write("trial.xml")

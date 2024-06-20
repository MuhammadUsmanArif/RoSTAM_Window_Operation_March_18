Latest Major Revision on 13/07/2018:
Trying to add new robots being added during the window operation part into the Execution followup. Most of the major
revisions are in the Execution_followup, Operators with minor revisions in GA_solo, deap_followup and penalties.py. Most
of the revisions are with the indexing and all coz when the robots are made from say 5 to 6 most of the arrays initiated
based on the previous instantiation start giving indexing error as they have only 5 slots and the followup demands 6 slots.


updates of this file were done on 29/4/2018
Copy and pasting the window edition of laptop on desktop
and saving separate version of both on the code backup. incase you feel that there is some garbar then can switch back
to the old versions.

The minor changes that are there between iterative and window is that for iterative the two solution_individual files will
go as solution_individual_run3 and solution individual_run1 (simple search and change the names, so that I can differentiate
between the first and the last run. In the deap_followup the same file is renamed as solution_individual_run2 (Opening
after 3 months I have renamed all these 3 files to solution_individual.txt). Rest make some appropriate changes to the
config file (no new tasks should be there for window operation). Further make sure to switch the penalty function used
between the fix penalty for window based execution and fix penalty for normal execution. That way your windowed execution
will not produce infeasible solutions. Also you can complete comment out the gazebo launch
portion in the deap_followup while running the window operation as it isn't needed in window operations.
One thing important to note over here is that both iterative and windowed versions will be needing this
followup folder inside the /40/0/followup job distribution you are talking about.

The mutationBig in the Operator.py (within the mutation routines) doesn't work well with window based operation. Gives an
Error in the final execution of things. So change it from mutationBig to mutation1 when running window operation and can
go back to mutationBig when running the iterative version of things. Also the panelty fix function is different for the
window operation and for normal iterative operation. so comment uncomment those portions as well


The windowed Operation is basically designed for the iteration based execution where RoSTAM runs parallel to the task
execution phase of robots. So in this scheme the algo keeps running and assigns a few task to the robots at the end of
each execution, and then the algorithm starts all over again from the point it left. This keeps happening until there
are no more tasks left to be allocated. This windowed version can also handle robot failures using the active_robots
matrix where a 1 for a robot in a particular run means the robot is available for allocations in this cycle and a 0 means
the robot isn't available. This matrix has a row for each iteration of the algorithm and an inactive robot can come back
as active in future iterations.
So basically the whole operation starts with the GA_solo run. when complete the Execution_followup goes through a series
of steps these include
1. Isolating from the complete solution the window sized allocations for each robot. This is read from the solution_
individual.txt file then processed and the individual allocations read. With respect to the allocations the starting
locations of the robots are also altered (next starting location is the last task attempted in this window). I did write
a new fitness function which allows robots to take up task from non depot locations. Next the termination criteria is
evaluated by comparing the task left to be assigned (read from task_representaion.txt and compared against allocated tasks
in parameters). If the number of tasks is less than the next window size then the operation is terminated. At the end of
the whole operation the final solution is saved in solution_individual for further reference. Also the fitness is evalu
ated for this windowed final solution using an inhouse fitness evalution function written in Execution_followup.



File Details:

____G___P___J___R____I_jobs_solutions.txt:
____G___P___J___R____I_jobs_stats.txt:      These two file are put in by GA_solo.py and deap__followup.py to record the
                                            stats and best solution of each generation.
best_solution.pkl:  Dumped by GA_solo and deap__followup at the end of their runs effectively saving the last best
                    solution. This is used in deap__followup for edit_start_locations routine which
                    is usually commented out but could be really important if you are going into any iterative routine
                    with already attempted tasks so that the last solution could be checked for where the robots left
                    their operation by comparing the solution and attempted tasks.
start2jobs.txt:     This file is made by the reader_classbased_dict.get_costmatrix routine where this is nothing but
                    the distance of each task from the home location. The first line of the cost matrix effectively
                    computed using fitness4mstart sort of some routine.
dist_matrix.txt:    This file is made by the reader_classbased_dict.get_costmatrix routine where this is the matrix
                    calculated using ROS, storing the inter task distances including the start matrix. The complete matrix
dist_matrix_pickle.pkl: The same distance matrix calculated in reader_classbased_dict.get_costmatrix is also saved as
                        this file. has all the distances including the start matrix seems to be never used
dist_matrix.pkl:    This is exctly the same file as above (dist_matrix_pickle) the only difference is that this one is
                    dumped in GA_solo and deap__followup which means the dist_matrix.pkl is updated after every run
start_matrix.pkl:   This is the first line of the dist matrix. is dumped in GA_solo and deap__followup which means the
                    start_matrix.pkl is updated after every run. Is extensively used in GA_solo for the already
                    generated cases of tasks. Otherwise it is generated afresh by reader_classbased_dict.get_costmatrix
                    in both GA_solo and deap__followup.
job_representation.txt: used in reader_classbased_dict.get_job_dict to read the conventional jobs written in the text
                        file and convert them into job dictionary and sub_job_dict
job_dict.pkl:       reader_classbased_dict.get_job_dict creates this file and is used in GA_solo and deap_followup.
                    The GA_solo dumps it if the whole ROS thing runs, i.e. there is a brand new iteration running.
                    The deap_followup updates it always as deap followup usually runs after new tasks arrive.
                    The file is always altered by deap_followup
sub_job_dict.pkl:   reader_classbased_dict.get_job_dict creates this file and is used in GA_solo and deap_followup.
                    The GA_solo dumps it if the whole ROS thing runs, i.e. there is a brand new iteration running.
                    The deap_followup updates it always as deap followup usually runs after new tasks arrive.
                    The file is always altered by deap_followup
sub_job_dict_followup.pkl: created in deap__followup by deleting attempted jobs from all sub jobs i.e.
                    all_tasks - attempted tasks = followup tasks to be done. Used in SSI_followup later on.
                    Only updated by deap__followup. Becomes important when there are already attempted tasks
job_file.txt:       Formed by the main routine in gen_job.py where for a given count new jobs are generated and stored
                    in this file. used in the jobs_alteration.py by jobs_representation_simple for creating the sub job
                    based represetation out of the normal jobs initiated in this file.
jobs.pkl:           This is the same file as job_file.txt. Created by gen_job.py main routine. where generated jobs
                    are saved. Used in the job_alteration.py by the routine add_job_simple where old jobs can be kept in
                    front while generating new jobs so that no conflict arises.
job_representation.txt:     This is created by job_alteration.job_representaion_simple OR reader_classbased_dict.job_representation
                            Contains the sub job based represetation out of the normal jobs initiated in this file.
                            Is used in reader_classbased_dict.py routined init_jobs and get_job_dict(which returns job_dict
                            and sub_job_dict) once these are formed then there is no real need of this file. GA_solo and
                            deap__followup then use the job_dict and sub_job_dict.
population_file.pkl: Written in GA_solo and used and written in deap__followup. Basic use is to save last generation so that
                             if could be used on start up for the iterative version of things.
solution.txt and solution_individual.txt:   Both written in GA_solo and deap__followup and used no where. Just to show you the resutls.
speed.txt:          Written in GA_solo and deap__followup and if transfered outside the iteration specific folder i.e.
                    /home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA.txt then is used by
                    parsers.read_speed routine which is used in auction and SSI to read the speed of robots against a particular
                    allocation to be tested.
auction_solution.txt    Solutions saved by auction_traderbots and SSI of their results. Used no where, is for the user.
SSI_solution_file.pkl: Written by SSI_auction_Sven at the end of the run to save the solution and used by SSI_followup.py
                        for reinitiating the work from where it is left before the addition of tasks.

import Tkinter
import threading
import Execution_followup
import pickle
import os
import GA_solo
import sys
import subprocess

parameters = pickle.load(open("/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/parameters.pkl" , 'rb'))



def execute_followup(added_tasks, aded_task_type, aded_robot_num, attempted_tasks, failed_robots, CXPB_follow, MUTPB_follow, GEN_follow, POP_follow):
    temp = added_tasks.get()
    parameters['new_jobs'] = int(added_tasks.get())
    parameters['new_jobs_type'] = int(aded_task_type.get())
    parameters['robot_num_new'] = int(aded_robot_num.get())
    parameters['CXPB_2'] = float(CXPB_follow.get())
    parameters['MUTPB_2'] = float(MUTPB_follow.get())
    parameters['NGEN_2'] = int(GEN_follow.get())
    # parameters['POP_SIZE'] = int(POP.get())

    print 'executing followup', temp
    pickle.dump(parameters, open(parameters['FileString'] + "parameters.pkl", "wb"))
    GA_solo.main()
    # def execution_followup():
    #     Execution_followup.followup()
    #
    # t = threading.Thread(target=execution_followup)
    # t.start()


def button1_press():
    os.system('python config.py')  #executing config.py once to make sure that the settings are saved into a pickle file
    # reading the parameters in the pickle file saved by the config.py execution.
    parameters = pickle.load(
        open("/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/parameters.pkl", 'rb'))
    parameters['jobs'] = int(Task_count.get())
    parameters['jobs_type'] = int(task_type.get())
    parameters['robot_num'] = int(robot_num.get())
    parameters['CXPB'] = float(CXPB.get())
    parameters['MUTPB'] = float(MUTPB.get())
    parameters['NGEN'] = int(NGEN.get())
    parameters['POP_SIZE'] = int(POP.get())
    parameters['window_size'] = int(window_size.get())
    iteration_string = (str(parameters['jobs']) + "/" + str(parameters['iteration']) + "/")
    parameters['Complete_file_String'] = parameters['FileString'] + iteration_string
    print parameters
    pickle.dump(parameters, open(parameters['FileString'] + "parameters.pkl", "wb"))


    def execution():
        # output = subprocess.check_output("python GA_solo.py", shell=True)
        # Text_box1.insert(Tkinter.END, output)
        Execution_followup.main()
    t = threading.Thread(target=execution)
    t.start()
    



def followup_window():
    # create child window
    followup_win = Tkinter.Toplevel()
    followup_win.geometry("723x588+152+80")

    # display message
    # message = "This is the child window"
    # Tkinter.Label(followup_win, text=message).pack()
    # quit child window and return to root window
    # the button is optional here, simply use the corner x of the child window
    # Tkinter.Button(followup_win, text='OK', command=followup_win.destroy).pack()

    Frame0 = Tkinter.Frame(followup_win)
    Frame0.place(relx=0.007, rely=0.05, relheight=0.52, relwidth=0.52)
    Frame0.configure(relief='groove')
    Frame0.configure(borderwidth="2")
    Frame0.configure(width=365)

    Label1 = Tkinter.Label(Frame0)
    Label1.place(relx=0.01, rely=0.01, height=19, width=145)
    Label1.configure(text='''Number of Added Tasks''')
    added_tasks = Tkinter.Entry(followup_win)
    added_tasks.place(relx=0.01, rely=0.09, height=21, relwidth=0.2)
    added_tasks.configure(background="white")
    added_tasks.configure(font="TkFixedFont")
    added_tasks.insert(0, '15')

    Radiobutton1 = Tkinter.Radiobutton(Frame0)
    Radiobutton1.place(relx=0.42, rely=0.1, relheight=0.04
                       , relwidth=0.30)
    Radiobutton1.configure(activebackground="#d9d9d9")
    # Radiobutton1.configure(justify='right')
    Radiobutton1.configure(text='''Multi-Robot Tasks''')

    Label2 = Tkinter.Label(Frame0)
    Label2.place(relx=0.01, rely=0.15, height=19, width=125)
    Label2.configure(text='''Type of Added Tasks''')

    aded_task_type = Tkinter.Entry(Frame0)
    aded_task_type.place(relx=0.01, rely=0.21, height=21, relwidth=0.2)
    aded_task_type.configure(background="white")
    aded_task_type.configure(font="TkFixedFont")
    aded_task_type.insert(0, '1')

    Label3 = Tkinter.Label(Frame0)
    Label3.place(relx=0.01, rely=0.27, height=19, width=110)
    Label3.configure(text='''Number of Robots''')

    aded_robot_num = Tkinter.Entry(Frame0)
    aded_robot_num.place(relx=0.01, rely=0.32, height=21, relwidth=0.2)
    aded_robot_num.configure(background="white")
    aded_robot_num.configure(font="TkFixedFont")
    aded_robot_num.insert(0, '3')

    Label4 = Tkinter.Label(Frame0)
    Label4.place(relx=0.01, rely=0.4, height=19, width=325)
    Label4.configure(text='''Already attempted tasks if any, seperated by comma''')

    attempted_tasks = Tkinter.Entry(Frame0)
    attempted_tasks.place(relx=0.01, rely=0.47, height=21, relwidth=0.2)
    attempted_tasks.configure(background="white")
    attempted_tasks.configure(font="TkFixedFont")
    attempted_tasks.insert(0, '3')

    Label5 = Tkinter.Label(Frame0)
    Label5.place(relx=0.01, rely=0.54, height=19, width=265)
    Label5.configure(justify='left')
    Label5.configure(text='''Failed robots (if any), seperated by comma''')

    failed_robots = Tkinter.Entry(Frame0)
    failed_robots.place(relx=0.01, rely=0.61, height=21, relwidth=0.2)
    failed_robots.configure(background="white")
    failed_robots.configure(font="TkFixedFont")
    failed_robots.insert(0, '3')

    Frame1 = Tkinter.Frame(followup_win)
    Frame1.place(relx=0.67, rely=0.05, relheight=0.52, relwidth=0.3)
    Frame1.configure(relief='groove')
    Frame1.configure(borderwidth="2")
    Frame1.configure(width=365)

    Label4_f1 = Tkinter.Label(Frame1)
    Label4_f1.place(relx=0.03, rely=0.03, height=19, width=92)
    Label4_f1.configure(text='''EA Parameters''')

    Label5_f1 = Tkinter.Label(Frame1)
    Label5_f1.place(relx=0.03, rely=0.16, height=19, width=93)
    Label5_f1.configure(text='''Crossover Rate''')

    CXPB_follow = Tkinter.Entry(Frame1)
    CXPB_follow.place(relx=0.03, rely=0.23, height=21, relwidth=0.4)
    CXPB_follow.configure(background="white")
    CXPB_follow.configure(font="TkFixedFont")
    CXPB_follow.insert(0, '1.0')

    Label6_f1 = Tkinter.Label(Frame1)
    Label6_f1.place(relx=0.03, rely=0.33, height=19, width=88)
    Label6_f1.configure(text='''Mutation Rate''')

    MUTPB_follow = Tkinter.Entry(Frame1)
    MUTPB_follow.place(relx=0.03, rely=0.39, height=21, relwidth=0.4)
    MUTPB_follow.configure(background="white")
    MUTPB_follow.configure(font="TkFixedFont")
    MUTPB_follow.insert(0, '0.005')

    Label7_f1 = Tkinter.Label(Frame1)
    Label7_f1.place(relx=0.03, rely=0.49, height=19, width=110)
    Label7_f1.configure(text='''Generation Count''')

    GEN_follow = Tkinter.Entry(Frame1)
    GEN_follow.place(relx=0.03, rely=0.56, height=21, relwidth=0.4)
    GEN_follow.configure(background="white")
    GEN_follow.configure(font="TkFixedFont")
    GEN_follow.insert(0, '2000')

    Label8_f1 = Tkinter.Label(Frame1)
    Label8_f1.place(relx=0.03, rely=0.66, height=19, width=95)
    Label8_f1.configure(text='''Population Size''')

    POP_follow = Tkinter.Entry(Frame1)
    POP_follow.place(relx=0.03, rely=0.72, height=21, relwidth=0.4)
    POP_follow.configure(state='disabled')
    POP_follow.configure(background="white")
    POP_follow.configure(font="TkFixedFont")
    POP_follow.insert(0, '60')

    Button1 = Tkinter.Button(followup_win)
    Button1.place(relx=0.79, rely=0.88, height=27, width=96)
    Button1.configure(activebackground="#d9d9d9")
    Button1.configure(command= lambda: execute_followup(added_tasks, aded_task_type, aded_robot_num, attempted_tasks, failed_robots, CXPB_follow, MUTPB_follow, GEN_follow, POP_follow))
    Button1.configure(text='''Execute''')


# this main page is the startup page which would initiate GA_solo from execution_followup
main = Tkinter.Tk()
main.geometry("723x588+152+80")
main.title("RoSTAM")

Frame0 = Tkinter.Frame(main)
Frame0.place(relx=0.007, rely=0.05, relheight=0.45, relwidth=0.32)
Frame0.configure(relief='groove')
Frame0.configure(borderwidth="2")
Frame0.configure(width=365)

Label1 = Tkinter.Label(Frame0)
Label1.place(relx=0.01, rely=0.01, height=19, width=105)
Label1.configure(text='''Number of Tasks''')

Task_count = Tkinter.Entry(Frame0)
Task_count.place(relx=0.01, rely=0.09, height=21, relwidth=0.2)
Task_count.configure(background="white")
Task_count.configure(font="TkFixedFont")
Task_count.insert(0, '15')


Radiobutton1 = Tkinter.Radiobutton(Frame0)
Radiobutton1.place(relx=0.22, rely=0.1, relheight=0.04, relwidth=0.6)
Radiobutton1.configure(activebackground="#d9d9d9")
# Radiobutton1.configure(justify=LEFT)
Radiobutton1.configure(text='''Multi-Robot Tasks''')

Label2 = Tkinter.Label(Frame0)
Label2.place(relx=0.01, rely=0.16, height=19, width=85)
Label2.configure(text='''Type of Tasks''')

task_type = Tkinter.Entry(Frame0)
task_type.place(relx=0.01, rely=0.23, height=21, relwidth=0.2)
task_type.configure(background="white")
task_type.configure(font="TkFixedFont")
task_type.insert(0, '1')


Label3 = Tkinter.Label(Frame0)
Label3.place(relx=0.01, rely=0.31, height=19, width=113)
Label3.configure(text='''Number of Robots''')

robot_num = Tkinter.Entry(Frame0)
robot_num.place(relx=0.01, rely=0.40, height=21, relwidth=0.2)
robot_num.configure(background="white")
robot_num.configure(font="TkFixedFont")
robot_num.insert(0, '3')

Label4 = Tkinter.Label(Frame0)
Label4.place(relx=0.01, rely=0.46, height=19, width=200)
Label4.configure(text='''Window Operation, Window Size''')

window_size = Tkinter.Entry(Frame0)
window_size.place(relx=0.01, rely=0.52, height=21, relwidth=0.2)
window_size.configure(background="white")
window_size.configure(font="TkFixedFont")
window_size.insert(0, '3')


Frame1 = Tkinter.Frame(main)
Frame1.place(relx=0.33, rely=0.05, relheight=0.45, relwidth=0.3)
Frame1.configure(relief='groove')
Frame1.configure(borderwidth="2")
# Frame1.configure(relief=GROOVE)
Frame1.configure(width=365)

Label4 = Tkinter.Label(Frame1)
Label4.place(relx=0.03, rely=0.03, height=19, width=92)
Label4.configure(text='''EA Parameters''')

Label5 = Tkinter.Label(Frame1)
Label5.place(relx=0.03, rely=0.16, height=19, width=93)
Label5.configure(text='''Crossover Rate''')

CXPB = Tkinter.Entry(Frame1)
CXPB.place(relx=0.03, rely=0.23, height=21, relwidth=0.4)
CXPB.configure(background="white")
CXPB.configure(font="TkFixedFont")
CXPB.insert(0, '1.0')


Label6 = Tkinter.Label(Frame1)
Label6.place(relx=0.03, rely=0.33, height=19, width=88)
Label6.configure(text='''Mutation Rate''')

MUTPB = Tkinter.Entry(Frame1)
MUTPB.place(relx=0.03, rely=0.39, height=21, relwidth=0.4)
MUTPB.configure(background="white")
MUTPB.configure(font="TkFixedFont")
MUTPB.insert(0, '0.005')

Label7 = Tkinter.Label(Frame1)
Label7.place(relx=0.03, rely=0.49, height=19, width=110)
Label7.configure(text='''Generation Count''')

NGEN = Tkinter.Entry(Frame1)
NGEN.place(relx=0.03, rely=0.56, height=21, relwidth=0.4)
NGEN.configure(background="white")
NGEN.configure(font="TkFixedFont")
NGEN.insert(0, '2000')

Label8 = Tkinter.Label(Frame1)
Label8.place(relx=0.03, rely=0.66, height=19, width=95)
Label8.configure(text='''Population Size''')

POP = Tkinter.Entry(Frame1)
POP.place(relx=0.03, rely=0.72, height=21, relwidth=0.4)
POP.configure(background="white")
POP.configure(font="TkFixedFont")
POP.insert(0, '60')

Button1 = Tkinter.Button(main)
Button1.place(relx=0.79, rely=0.88, height=27, width=96)
Button1.configure(activebackground="#d9d9d9")
Button1.configure(command=followup_window)
Button1.configure(text='''Changes''')

Button2 = Tkinter.Button(main)
Button2.place(relx=0.59, rely=0.88, height=27, width=96)
Button2.configure(activebackground="#d9d9d9")
Button2.configure(command=button1_press)
Button2.configure(text='''Load & Run''')

Text_box1 = Tkinter.Text(main)
Text_box1.place(relx=0.01, rely=0.6, relheight=0.27, relwidth=0.95)
Text_box1.configure(background="white")
Text_box1.configure(font="TkTextFont")
Text_box1.configure(selectbackground="#c4c4c4")
Text_box1.configure(width=686)
Text_box1.configure(wrap='word')



main.mainloop()


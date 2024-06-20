import pickle

parameters = dict()
## ********************************* This is for first run GA_solo.py *************************************************
parameters['FileString'] = "/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/"
parameters['jobs'] = 50
parameters['iteration'] = 2
parameters['jobs_type'] = 1
parameters['map_dim'] = 6
parameters['robot_num'] = 3     # need to update from here, woiuldn't be updated from Execution_followup.py
parameters['execution_number'] = 0  #execution number is used in deap_followup to figure out if gazebo is to be started and new tasks need to be generated (for iterative version) and the distance matrix needs to be updated or not
parameters['window_size'] = 3       # this is for windowed operation.
parameters['robot_solutions_windowed'] = [[] for _ in range(int(parameters['robot_num']))]  #if windowed operation then the results at the end of eeach window based iteration will be saved over here. I guess it is also used to compute the penalty based on if any tasks were attempted in previous windows.
iteration_string = (str(parameters['jobs']) + "/" + str(parameters['iteration']) + "/")
parameters['Complete_file_String'] = parameters['FileString'] + iteration_string
parameters['start_location'] = [0] * parameters['robot_num']  # this is not being used right now but have added to allow robots to start from different location
# The information will be passed on to hetro_fitness_max_2 routine which allows robot to start form non depot locations
# The ending on depot can simply be commented out of the routine. A review of all other schemes
# i.e. Darrah, SSI, auction will be needed if this is implemented.

# parameters['speed_matrix'] = [1, 1, 1]
# parameters['speed_matrix'] = [0.7, 0.68, 0.66]
# parameters['speed_matrix'] = [.7, .68, .66, .64, .62]
parameters['speed_matrix'] = [1, 1, 1]

parameters['CXPB'] = 1.0              # 0.9 MR homo
parameters['MUTPB'] = 0.7           #MUTPB for gene based mutations is working with relatively lower values (0.005) compared to normal mutation (0.1)
parameters['NGEN'] = 1501             # number of jobs x 80 for MR Homo
parameters['POP_SIZE'] = 100
parameters['after_followup'] = False # used by GA_solo to figure out where to read dictionary from, the followup folder or main folder. Also to re-name the full run solution accordingly, run1 or run3 (iterative version).
parameters['load_cofficient'] = [1,1,1,1,1,1,1,1,1,1]
parameters['percentage_closeness_1'] = 0.2

## ********************************* This is for second run deap_followup.py *************************************************
#FileString = "/home/usman/research_gaz_ros/src/turtleb ot_research/src/txt_files/ST_SR_TA/"
# Implemented in the window based operations.
# Makes no sense starting with a failed robot so these all remain 1 in first iteration.
# Then this is tinkered with in Execution_followup.py
parameters['failed_robots'] = [1, 1, 1]   #Any 0 put in here will treat that robot as failed.
parameters['hidden_task_percentage'] = 0      #This is the percentage of tasks that should be hidden as for dynamic arrival
# I dont' think these are needed anymore. They were used when I used to generate new jobs for dynamic arrival.
# parameters['new_jobs_type'] = 4
# parameters['new_jobs'] = 0
# parameters['new_jobs_iteration'] = 3    #defines the iteration in which new jobs will be introduced.
parameters['robot_num_new'] = 3         #just a dummy value. will be updated by Execution_followup.py automatically.
parameters['start_location_2'] = [0] * parameters['robot_num_new'] #This is altered in Execution_followup.py before calling deap__followup.py
# The information will be passed on to hetro_fitness_max_2 ruotine which allows robot to start form non depot locaitons
# The ending on depot can simply be commented out of the routine. A review of all other schemes
# i.e. Darrah, SSI, auction will be needed if this is implemented.

# parameters['speed_matrix_2'] = [1, 1, 1]
# parameters['speed_matrix_2'] = [0.7, 0.68, 0.66]
# parameters['speed_matrix_2'] = [.7, .68, .66, .64, .62]
parameters['speed_matrix_2'] = [1, 1, 1, 1, 1, 1, 1]    # keep it of ample size, so that robot fluctuations don't give an error

parameters['CXPB_2'] = 1.0                # 0.9 MR homo
parameters['MUTPB_2'] = 0.7           #MUTPB for gene based mutations is working with relatively lower values (0.005) compared to normal mutation (0.1)
parameters['NGEN_2'] = 1201             # number of jobs x 80 for MR Homo
parameters['percentage_closeness_1_Onwards'] = 0.2

# Again populated by Execution_followup before calling the deap__followup routine. This is done from the previous best solution.
parameters['attempted_tasks'] = []# these are the tasks that have already been attempted, windowed or iterative operation
# updated on the move by Execution_followup.py
# left over tasks from the whole subdictionary are computed in GA_solo based on the percentage declared on the top
parameters['left_over_tasks'] = []
# Every run of deap__followup adds a certain number of jobs and updates the left over tasks accordingly.
parameters['new_tasks_released'] = []

pickle.dump(parameters, open(parameters['FileString'] + "parameters.pkl", "wb"))

# parameters = dict()
#
# ## ********************************* This is for first run GA_solo.py *************************************************
# FileString = "/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/"
# jobs = 50
# iteration = 2
# jobs_type = 4
# map_dim = 6
# robot_num = 5
# iteration_string = (str(jobs) + "/" + str(iteration) + "/")
# Complete_file_String = FileString + iteration_string
# start_location = [0] * robot_num  # this is not being used right now but have added to allow robots to start from different location
# # The information will be passed on to hetro_fitness_max_2 ruotine which allows robot to start form non depot locaitons
# # The ending on depot can simply be commented out of the routine. A review of all other schemes
# # i.e. Darrah, SSI, auction will be needed if this is implemented.
# # speed_matrix = [1, 1, 1]
# # speed_matrix = [0.7, 0.68, 0.66]
# # speed_matrix = [.7, .68, .66, .64, .62]
# speed_matrix = [1, 1, 1, 1, 1]
# CXPB = 1.0              # 0.9 MR homo
# MUTPB = 0.005           #MUTPB for gene based mutations is working with relatively lower values (0.005) compared to normal mutation (0.1)
# NGEN = 5             # number of jobs x 80 for MR Homo
# POP_SIZE = 60
#
#
# ## ********************************* This is for second run deap_followup.py *************************************************
# FileString = "/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/"
# new_jobs = 7            #
# new_jobs_type = 4
# robot_num_new = 5
# start_location_2 = [0] * robot_num_new  # this is not being used right now but have added to allow robots to start from different location
# # The information will be passed on to hetro_fitness_max_2 ruotine which allows robot to start form non depot locaitons
# # The ending on depot can simply be commented out of the routine. A review of all other schemes
# # i.e. Darrah, SSI, auction will be needed if this is implemented.
# # speed_matrix = [1, 1, 1]
# # speed_matrix = [0.7, 0.68, 0.66]
# # speed_matrix = [.7, .68, .66, .64, .62]
# speed_matrix_2 = [1, 1, 1, 1, 1, 1, 1]
# failed_robots = [1, 1, 1, 1, 1]
# CXPB_2 = 1.0              # 0.9 MR homo
# MUTPB_2 = 0.005           #MUTPB for gene based mutations is working with relatively lower values (0.005) compared to normal mutation (0.1)
# NGEN_2 = 5             # number of jobs x 80 for MR Homo
# attempted_tasks = ['1,1','2,1','4,1', '4,4', '5,1', '9,2', '45,1'] # these are the tasks that have already been attempted
#
#
# # -*- coding: utf-8 -*-
# import dash
# from dash.dependencies import Output, Input, State
# import dash_core_components as dcc
# import dash_html_components as html
# import plotly.graph_objs as go
# import pandas as pd
# import sys
# import Execution_followup
# # import config
#
#
# app = dash.Dash()
#
# # df = pd.read_csv(
# #     'https://gist.githubusercontent.com/chriddyp/'
# #     'cb5392c35661370d95f300086accea51/raw/'
# #     '8e0768211f6b747c0db42a9ce9a0937dafcbd8b2/'
# #     'indicators.csv')
# #
# # available_indicators = df['Indicator Name'].unique()
#
# app.layout = html.Div([
#     html.H1(children="RoSTAM"),
#     html.H2(children='''Robust and Self-adpative Task Allocation for Multi-robot teams'''),
#
#     html.Div([
#         html.Div(children='''Enter the task distribution you want to execute'''),
#         html.Div([
#             dcc.Dropdown(
#                 id='MRTA Distribution',
#                 options=[
#                     {'label': 'ST-SR-TE', 'value': 'ST-SR-TE'},
#                     {'label': 'ST-MR-TE', 'value': 'ST-MR-TE'}
#                 ],
#                 value='ST-SR-TE'),
#         ],
#         style={'width': '48%', 'display': 'inline-block'}),
#
#         html.Div([
#             html.Div(children='''Number of Robots'''),
#             html.Div([
#                 dcc.Dropdown(
#                     id='number of robot',
#                     options=[{'label': i, 'value': i} for i in range(1, 10)],
#                     value=3
#                 ),
#             ])
#          ],style={'width': '48%',  'display': 'inline-block'}),
#
#
#         html.Div([
#             html.Div(children='''Number of Tasks'''),
#             html.Div([
#                 dcc.Dropdown(
#                     id='number of tasks',
#                     options=[{'label': i, 'value': i} for i in range(10, 55, 5)],
#                     value=10
#                 ),
#             ])
#         ], style={'width': '48%',  'display': 'inline-block'}),
#
#
#         html.Div([
#             html.Div(children='''Type of Tasks'''),
#             html.Div([
#                 dcc.Dropdown(
#                     id='type of tasks',
#                     options=[{'label': i, 'value': i} for i in range(1, 5)],
#                     value=1
#                 ),
#             ])
#         ], style={'width': '20%', 'float': 'left', 'display': 'inline-block'}),
#
#         html.Div([
#             html.Div(children='''Iteration for Run'''),
#             html.Div([
#                 dcc.Dropdown(
#                     id='iteration',
#                     options=[{'label': i, 'value': i} for i in range(0, 5)],
#                     value=0
#                 ), ])
#         ], style={'width': '15%', 'float': 'right', 'display': 'inline-block'}),
#
#         html.Div([
#             html.Div(children='''Map dimensions'''),
#             html.Div([
#                 dcc.Dropdown(
#                     id='map dimension',
#                     options=[{'label': i, 'value': i} for i in range(4, 8)],
#                     value=4
#                 ), ])
#         ], style={'width': '15%', 'float': 'right', 'display': 'inline-block'}),
#     ]),
#
#     html.Div([
#         html.Label('Enter robot speeds'),
#         dcc.Input(
#             id='speeds of robots',
#             value='1.0, 1.0, 1.0, 1.0',
#             type='text'
#         ),
#         ],style={'width': '48%', 'float': 'right', 'display': 'inline-block'}),
#
#
#
#     html.Div(id='awaeen'),
#     html.Div([
#         html.Button('Submit', id='button'),
#         html.Div(id='output-container-button',
#                  children='Enter all the execution details above and press submit')
#     ])
# ])
#
# @app.callback(
#     Output('output-container-button', 'children'),
#     [Input('button', 'n_clicks')],
#     [State('number of tasks', 'value'),
#      State('speeds of robots', 'value'),
#      State('number of robot', 'value'),
#      State('iteration', 'value'),
#      State('type of tasks', 'value'),
#      State('map dimension', 'value')])
# def update_output(n_clicks, task_number, robot_speed, robot_count, iterations, type_of_tasks, map_dimension):
#     parameters['jobs'] = task_number
#     parameters['robot_num'] = robot_count
#     parameters['iteration'] = iterations
#     parameters['type_of_tasks'] = type_of_tasks
#     parameters['map_dim'] = map_dimension
#     for entry in robot_speed:
#         entry.replace(" ", "")
#     another_list = robot_speed.split(',')
#     print type(n_clicks), n_clicks
#     print type(task_number), task_number
#     print type(robot_speed), robot_speed
#     print type(another_list), another_list
#     print type(map_dimension), map_dimension
#     # Execution_followup.main()
#     return 'The number of tasks {} and the number of robots {} and their speeds {}'.format(
#         task_number,
#         robot_count,
#         robot_speed,
#         )
#
# @app.callback(
#     Output(component_id='awaeen', component_property='children'),
#     [Input(component_id='speeds of robots', component_property='value')]
# )
# def update_output_div(input_value):
#     print input_value
# #
# # @app.callback(
# #     dash.dependencies.Output('indicator-graphic', 'figure'),
# #     [dash.dependencies.Input('xaxis-column', 'value'),
# #      dash.dependencies.Input('yaxis-column', 'value'),
# #      dash.dependencies.Input('xaxis-type', 'value'),
# #      dash.dependencies.Input('yaxis-type', 'value'),
# #      dash.dependencies.Input('year--slider', 'value')])
# # def update_graph(xaxis_column_name, yaxis_column_name,
# #                  xaxis_type, yaxis_type,
# #                  year_value):
# #     dff = df[df['Year'] == year_value]
# #
# #     return {
# #         'data': [go.Scatter(
# #             x=dff[dff['Indicator Name'] == xaxis_column_name]['Value'],
# #             y=dff[dff['Indicator Name'] == yaxis_column_name]['Value'],
# #             text=dff[dff['Indicator Name'] == yaxis_column_name]['Country Name'],
# #             mode='markers',
# #             marker={
# #                 'size': 15,
# #                 'opacity': 0.5,
# #                 'line': {'width': 0.5, 'color': 'white'}
# #             }
# #         )],
# #         'layout': go.Layout(
# #             xaxis={
# #                 'title': xaxis_column_name,
# #                 'type': 'linear' if xaxis_type == 'Linear' else 'log'
# #             },
# #             yaxis={
# #                 'title': yaxis_column_name,
# #                 'type': 'linear' if yaxis_type == 'Linear' else 'log'
# #             },
# #             margin={'l': 40, 'b': 40, 't': 10, 'r': 0},
# #             hovermode='closest'
# #         )
# #     }
#
#
# if __name__ == '__main__':
#     app.run_server(debug = True/
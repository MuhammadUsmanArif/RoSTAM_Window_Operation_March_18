import xlsxwriter


#this file reads the FilMINT the Knitro and the RoSTAM output files created within each individual folder
# there are two sheets created within the workbook where the first sheet records the results experiment by experiment so I can see whats happening
# the 2nd sheet just compares the three readings value by value so I can calculate total error etc, percentile etc.


workbook = xlsxwriter.Workbook('/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/50/FinalOUTPUT.xlsx')
worksheet1 = workbook.add_worksheet()
worksheet2 = workbook.add_worksheet()
RoSTAM_row = 0  #keeping track of which row to write the values in
column_start = 8    #the column represents the experiment folder from where the stuff would be read
for clmn in range(column_start,26):        # instances
    open_file = open('/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/50/' + str(clmn) + '/partial_RoSTAM_out.txt', 'r')
    worksheet1.write(0, clmn - column_start, ('RoSTAM ' + str(clmn-column_start+1))) #to start writing from column 0 I am going for clmn-column_start
    for i, line in enumerate(open_file):
        worksheet1.write(i+1, clmn-column_start, float(line.strip('\n'))) # this changes the column for every new experiment
        worksheet2.write(RoSTAM_row+i, 0, float(line.strip('\n')))          # this keeps writing in the same column so later on I can compare value by value for overall error.
    open_file.close()

    open_file = open('/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/50/' + str(clmn) + '/outputFilMINT', 'r')
    worksheet1.write(10, clmn - column_start, ('FilMINT ' + str(clmn-column_start+1)))
    for i, line in enumerate(open_file):
        worksheet1.write(i+11, clmn-column_start, float(line.strip('\n')))
        worksheet2.write(RoSTAM_row + i, 1, float(line.strip('\n')))
    open_file.close()


    open_file = open('/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/50/' + str(clmn) + '/outputKnitro', 'r')
    worksheet1.write(20, clmn - column_start, ('Knitro ' + str(clmn-column_start+1)))
    for i, line in enumerate(open_file):
        worksheet1.write(i+21, clmn-column_start, float(line.strip('\n')))
        worksheet2.write(RoSTAM_row + i, 2, float(line.strip('\n')))
    open_file.close()

    open_file = open('/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/50/' + str(clmn) + '/partial_RoSTAM_out.txt', 'r')
    RoSTAM_row += len(open_file.readlines())    #running the number of rows to proceed after the 1 experiment has been completely recorded.
    open_file.close()

workbook.close()
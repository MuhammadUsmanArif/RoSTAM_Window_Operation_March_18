import os
import glob
import time

# writing this code to delete all the extra files from floders for re-runs of schemes. Like I have all the data for
# ST-SR-IA but when had to rerun for SSI I had much of the data repeated. So just held the SSI solution file back and
# deleted everything else in all the folders. Before was doing it manually.

FileString = "/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/15/0/"

def main(FileString):
    files = glob.glob(FileString + '*')
    print files
    for f in files:
            if f == (FileString + '*.pkl'):
                os.remove(f)
            else:
                pass



if __name__ == '__main__':
    main()
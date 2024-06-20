import os
import glob

FileString = "/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/"
def main():
    None
    Names_ASF = ['ASF_followup25', 'ASF_followup60', 'ASF_followup90', 'ASF_followup115', 'ASF_gaSOLO']
    Names_BSF = ['BSF_followup25', 'BSF_followup60', 'BSF_followup90', 'BSF_followup115', 'BSF_gaSOLO']

    for i, Name in enumerate(Names_ASF):
        ASF_file = open(FileString + Name + '.txt', 'rb')
        BSF_file = open(FileString + Names_BSF[i] + '.txt', 'rb')
        ASF_file_avg = open(FileString + Name + '_avg.txt', 'wb')
        BSF_file_avg = open(FileString + Names_BSF[i] + '_avg.txt', 'wb')
        ASF_lines = ASF_file.readlines()
        BSF_lines = BSF_file.readlines()
        for p in range(0, 1000):
            asf_sum = 0
            bsf_sum = 0
            for q in range (0,10000, 1000):
                asf_sum += float(ASF_lines[p+q].strip('\n'))
                bsf_sum += float(BSF_lines[p+q].strip('\n'))
            print asf_sum, bsf_sum
            ASF_file_avg.write(str(asf_sum/10.0) + '\n')
            BSF_file_avg.write(str(bsf_sum/10.0) + '\n')
        ASF_file_avg.close()
        BSF_file_avg.close()

    for i, Name in enumerate(Names_ASF):
            os.remove(FileString + Name + '.txt')
            os.remove(FileString + Names_BSF[i] + '.txt')


if __name__ == "__main__":
    main()
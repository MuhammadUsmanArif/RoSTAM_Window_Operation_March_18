import pickle

#This routine from the auction_solution.txt file forms an auction.txt file which has the plain result for later seed creation.
# which result out of the auction, reauction, SSI has to be used for file creation is to be decided by NameString
def auc_resutl_2_file(FileString, NameString, robot_num):
    # Auction Re-Auction Random Allocation SSI_auction
    NameString = NameString + '\n'      #there is a '\n' at the end of every line in this file
    auction_solution = open((FileString + 'auction_solution.txt'), 'r')
    conversion = open((FileString) + 'auction.txt', 'wb')
    lines = auction_solution.readlines()
    for i in range(0, len(lines)):
        if lines[i] == NameString:
            for j in range(robot_num):
                conversion.write(lines[i+j+1])
    # for i, line in enumerate(auction_solution):
    #     if line == NameString:
    #         for j in range(robot_num):
    #             print auction_solution.readline(i+j)


# This routine uses the auction.txt file created using the auc_resutl_2_file routine on top to form a seed.pkl.
# seed.pkl can then be used for forming initial value using this seed.
def auction_2_GA_random(FileString, robot_num):
    # print FileString
    ind = [[]] * 2 #[None] * ind_size
    dummy_ind = list() #[None] * ind_size
    sub_tour = [None] * robot_num
    auc_sol_file = open((FileString +'auction.txt'), 'r')
    # seed_file = open(FileString + 'seed.pkl', 'a')
    for i,line in enumerate(auc_sol_file):
        line = line.replace("[","")
        line = line.replace("]", "")
        line = line.replace(" ", "")
        line = line.replace("\n", "")
        for j, letter in enumerate(line.split(',')):
            # print i, letter
            count = dummy_ind.count(letter)
            sub_tour[i] = j+1
            ind[0].append(str(letter) + "," + str(count+1))
            dummy_ind.append(letter)
    ind[1] = sub_tour
    # print type(ind)
    with open(FileString + 'seed.pkl', 'wb') as ind_file:
        pickle.dump(ind,ind_file)
        # pickle.dump(sub_tour, ind_file)
    ind_file.close()
    # for value in ind:
    #     seed_file.write(value)
    # seed_file.write(str(ind))
    # seed_file.write(str(sub_tour))
    print sub_tour
    print ind
    print dummy_ind
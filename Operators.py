## File update on 12/07/2018 Major changes in the 2nd part mutation 1 and mutation Big, Also deleting a few commented executions here and there. Have a look at an older copy if need anything.
from __future__ import division
import random
import itertools
import pickle
import re
import copy
import warnings

from collections import Sequence
from itertools import repeat

from compiler.ast import flatten
import copy


def TCX_yuan2013(ind1, ind2):
    temp1 = copy.deepcopy(ind1)
    ind1 = TCX_dirty_work(ind1, ind2)
    ind2 = TCX_dirty_work(ind2, temp1)
    return ind1, ind2

def TCX_dirty_work(cl1, cl2):     #cl1 should be the one to be altered and returned. cl2 is just for ref.

    # cl1= copy.deepcopy(cl1)
    # cl2 = copy.deepcopy(cl2)
    start = 0
    sub_str = [None] * len(cl1.robots_assign)         # empty list of list for saving the substrings
    # substr_len = [None]*len(ind1.robot_assign)      # empty list of list for saving he len of substring


    for i in range(len(cl1.robots_assign)):  # for each individual robot
        end = start + cl1.robots_assign[i]  # mark the ending location for reading the jobs
        jobs = cl1[start:end]  # pick jobs for the first robot
        if len(jobs) > 0:
            substr_start = random.randint(start, start + len(jobs) - 1)  # pick random starting and ending locations for reading the substr
            substr_end = random.randint(substr_start + 1, start + len(jobs))
            sub_str[i] = cl1[substr_start:substr_end]  # read the substring
        else:
            sub_str[i] = []
        start = end  # shift the markers for the next robot


    merged_substr = list(itertools.chain.from_iterable(sub_str))
    left_clone1 = [x for x in cl1 if x not in merged_substr]
    index_dict = dict((value, idx) for idx, value in enumerate(cl2))  # creating a dictionary of values to index for finding out the index of left over items
    indexLEFT_clone1 = [index_dict[x] for x in left_clone1]  # the list of index values of the left over items
    SORTED_leftclone1 = [left_clone1 for (indexLEFT_clone1, left_clone1) in sorted(zip(indexLEFT_clone1, left_clone1))] #sorting out the left over items of cl1 according to their appearance in clone 2
    del (cl1[:])
    empty_list = list()

    for i in range(len(sub_str)):
        empty_list.append(sub_str[i])   #read the first substring from substring array initiated from cl1
        if SORTED_leftclone1:
            copy_index = random.randint(1, len(SORTED_leftclone1))  #pick a random index to copy from the left over list of clone 1 sorted according to clone 2
        else:
            copy_index = 0
        if i == len(sub_str) - 1:   #if this is the last entry of the list of substring (last robot we are talking about) picked from clone 1
            empty_list.append(SORTED_leftclone1[:])    #append all of whats left in the sorted list
            value = len(sub_str[i]) + len(SORTED_leftclone1)    #update the value accordingly so robots assignment could be updated.
        else:                                       #if not the last robot we are talking about
            empty_list.append(SORTED_leftclone1[0:copy_index])  #then copy only some portion of the sorted list
            value = len(sub_str[i]) + copy_index    #update the value accordingly for updation of robots assignment
        cl1.robots_assign[i] = value             #updating robots assignment
        del (SORTED_leftclone1[0:copy_index])
    empty_list = flatten(empty_list)
    cl1.extend(empty_list)
    return cl1

# same as permutation_crossover but the permutation crossover was creating giving same children after crossover at the
# later stages due to the fact that it copies the same index values from parent 2 to child 1 which results in no diversity
# in the later stages of the algorithm. So had to make this routine where the changes are made starting right after the
# constant substring for child1 but the values are copied from first index of parent 2 (vise versa)
# provided much more diversity till the end of the run
#  Simple crossover for permutation where portion of child1 is kept constant from parent 1 and the remaining values
# (tasks) are copied from the 2nd parent in its order where any repetations are skipped. Same happens for 2nd child
def permutation_crossover2(cl1, cl2):
    temp = copy.deepcopy(cl1)  # keeping a copy of cl1 for later use (while forming child 2)
    a, b = random.sample(xrange(len(cl1)), 2)  # picking 2 ids to identify the sub-tour which has to be kept constant
    if a > b:
        a, b = b, a  # making sure a < b

    # this portion keeps the substring and replaces everything else with a string of number i.e. str(x)
    for x in range(0, len(cl1)):
        if x in range(a, b):
            pass
        else:
            cl1[x] = str(x)
    # this portion copies from 2nd parent the rest of the part. Start from index where the constant slice left in parent 1 and starts copying from the same index of parent 2
    counter = 0                 # keeping an extra counter to keep track of how many replacements have been made from parent 2 to child 1
    for y in range(len(cl2)):
        if cl2[y] in cl1:       # if the item is already in child 1 then do nothing
            pass
        else:                   # if item not in child 1 then copy it starting from after substring (counter+b)
            cl1[(counter + b)%len(cl1)] = cl2[y]  # for first case counter = 0 so counter + b = b
                                                # the % helps start from beginning when counter + b exceeds length
            counter += 1                        # update the counter so next item is copied next to last copy

    # Repeat for child 2
    for x in range(0, len(cl2)):
        if x in range(a, b):
            pass
        else:
            cl2[x] = str(x)
    counter = 0
    for y in range(len(temp)):
        if temp[y] in cl2:
            pass
        else:
            cl2[(counter + b) % len(cl2)] = temp[y]
            counter += 1

    # performing the crossover for the 2nd part of the chromosome
    # Snd_Part_Cros_Rvrs(cl1)
    # Snd_Part_Cros_Rvrs(cl2)

    return cl1, cl2


def permutation_Carter_2006(cl1, cl2):
    if len(cl1) <= 1:
        return cl2, cl1

    temp_1 = copy.deepcopy(cl1)  # keeping a copy of cl1 and cl2 for later use (while forming child 1 and child2)
    temp_2 = copy.deepcopy(cl2)
    if len(cl1) == 1:
        return cl2,cl1
    a, b = random.sample(xrange(len(cl1)), 2)  # picking 2 ids to identify the sub-tour which has to be kept constant
    if a > b:
        a, b = b, a  # making sure a < b
    # this portion keeps the substring and replaces everything else with a string of number i.e. str(x)
    for x in range(0, len(cl1)):
        if x in range(a, b):
            pass
        else:
            cl1[x] = str(x)
            cl2[x] = str(x)
    counter = 0                 # keeping an extra counter to keep track of how many replacements have been made from parent 2 to child 1
    #this portion copies from 2nd parent the rest of the part. Start from index 0 of parent 1 and starts copying from index 0 of parent 2
    for y in range(len(temp_2)):
        if counter == a:        # if have reached the left edge of the copied string (from parent 2) then teleport to
            counter += (b-a)    # the right edge of the substring
        if temp_2[y] in cl1:       # if the item is already in child 1 then do nothing
            pass
        else:                   # if item not in child 1 then copy it starting from after substring (counter+b)
            cl1[(counter)%len(cl1)] = temp_2[y]  # for first case counter = 0 so counter + b = b
                                                 # the % helps start from beginning when counter + b exceeds length
            counter += 1                        # update the counter so next item is copied next to last copy
    # doing the same as above but now for child 2
    counter = 0
    for y in range(len(temp_1)):
        if counter == a:
            counter += (b - a)
        if temp_1[y] in cl2:
            pass
        else:
            cl2[(counter)%len(cl2)] = temp_1[y]
            counter += 1

    # # performing the crossover for the 2nd part of the chromosome
    # if random.random() < 0.2:
    #     Snd_Part_Cros_Rvrs(cl1)
    #     Snd_Part_Cros_Rvrs(cl2)
    return cl1, cl2

def ordered_crossover(cl1, cl2):
    if len(cl1) <= 1:
        return cl2, cl1

    temp_1 = copy.deepcopy(cl1)  # keeping a copy of cl1 and cl2 for later use (while forming child 1 and child2)
    temp_2 = copy.deepcopy(cl2)

    id1, id2 = random.sample(xrange(len(cl1)), 2)
    if id2 < id1:
        id1, id2 = id2, id1
    substring = cl1[id1:id2]
    substring_index = []
    for value in substring:
        substring_index.append(cl2.index(value))
    # print substring
    # print substring_index
    sorted_substring = [x for _, x in sorted(zip(substring_index, substring))]
    # print sorted_substring
    cl1[id1:id2] = sorted_substring


    substring = cl2[id1:id2]
    substring_index = []
    for value in substring:
        substring_index.append(temp_1.index(value))
    # print substring
    # print substring_index
    sorted_substring = [x for _, x in sorted(zip(substring_index, substring))]
    # print sorted_substring
    cl2[id1:id2] = sorted_substring

    return cl1, cl2

def BOX(cl1, cl2, best): ##best_ordered_crossover Inspired from Andreica_Chira_2015, For exact working please see the paper
    if len(cl1) <= 3:   #this operator works be creating 3 substrings. So if chromo is very small apply ORX instead
        return ordered_crossover(cl1, cl2)

    temp_1 = copy.deepcopy(cl1)  # keeping a copy of cl1 and cl2 for later use (while forming child 1 and child2)
    temp_2 = copy.deepcopy(cl2)

    #the chromo is to be split into n substring where each's length should be between 1 and len(chromo)/3
    random_sequence = [random.randint(1, int(len(cl1)/3)) for x in range(0,len(cl1)-1)] #generating random lengths
    #creating atlease 3 substrings against the picked up random lengths
    cut_off_points = [random_sequence[0], random_sequence[0] + random_sequence[1], random_sequence[0]+random_sequence[1]+random_sequence[2]]
    #if there is still room for more substrings then at more indexes
    for j in range(3,len(random_sequence)):
        if cut_off_points[j-1] + random_sequence[j] <= len(cl1)-1:
            cut_off_points.append(cut_off_points[j-1] + random_sequence[j])
            # print 'updated cutoff points are', cut_off_points
        else:
            break
    # print 'updated cutoff points are', cut_off_points
    ## Against each substring identify the source to be used while giving that substring ordering.
    sequence_source = []
    for j in range(len(cut_off_points)+1):
        sequence_source.append(random.choice([-1,-2,-3]))

    child = []
    start = 0
    sorted_substring = []
    # print 'going for alteration in ', cl1
    ## Read the substring from the chromosome against already identified indexes
    for j in range(0,len(cut_off_points)):
        if j < len(cut_off_points)-1:
            end = cut_off_points[j]
        else:
            end = len(cl1)
        substring = cl1[start:end]
        substring_index = []
        # print 'this is the substring', substring, start,end
        ## Depending upon the source for that substring reorder them against self, parent2 or best and then put in offspring
        if sequence_source[j] == -1:    ## Against Self
            for value in substring:
                substring_index.append(cl1.index(value))
            sorted_substring = [x for _, x in sorted(zip(substring_index, substring))]
            child.extend(sorted_substring)
        if sequence_source[j] == -2:    ## Against parent 2
            for value in substring:
                substring_index.append(cl2.index(value))
            sorted_substring = [x for _, x in sorted(zip(substring_index, substring))]
            child.extend(sorted_substring)
        if sequence_source[j] == -3:    ## Against best
            for value in substring:
                substring_index.append(best.index(value))
            sorted_substring = [x for _, x in sorted(zip(substring_index, substring))]
            child.extend(sorted_substring)
        start = end
    # print 'this is the final shape', child
    cl1[:] = child[:]   ## When done with all substrings copy the offspring back


    ## Repeat the same for parent 2
    random_sequence = [random.randint(1, int(len(cl2)/3)) for x in range(0,len(cl2)-1)]
    cut_off_points = [random_sequence[0], random_sequence[0] + random_sequence[1], random_sequence[0]+random_sequence[1]+random_sequence[2]]
    for j in range(3,len(random_sequence)):
        if cut_off_points[j-1] + random_sequence[j] <= len(cl2)-1:
            cut_off_points.append(cut_off_points[j-1] + random_sequence[j])
            # print 'updated cutoff points are', cut_off_points
        else:
            break
    # print 'updated cutoff points are', cut_off_points
    sequence_source = []
    for j in range(len(cut_off_points)+1):
        sequence_source.append(random.choice([-1,-2,-3]))


    child = []
    start = 0
    sorted_substring = []
    # for j, entry in enumerate(cut_off_points):
    #     end = entry
    for j in range(0, len(cut_off_points)):
        if j < len(cut_off_points) - 1:
            end = cut_off_points[j]
        else:
            end = len(cl1)
        substring = cl2[start:end]
        substring_index = []
        if sequence_source[j] == -1:
            for value in substring:
                substring_index.append(cl2.index(value))
            sorted_substring = [x for _, x in sorted(zip(substring_index, substring))]
            child.extend(sorted_substring)
        if sequence_source[j] == -2:
            for value in substring:
                substring_index.append(temp_1.index(value))
            sorted_substring = [x for _, x in sorted(zip(substring_index, substring))]
            child.extend(sorted_substring)
        if sequence_source[j] == -3:
            for value in substring:
                substring_index.append(best.index(value))
            sorted_substring = [x for _, x in sorted(zip(substring_index, substring))]
            child.extend(sorted_substring)
        start = end
    cl2[:] = child[:]
    return cl1, cl2


def robot_based_BOX(cl1, cl2, best): ##Idea inspired from Andreica_Chira_2015 but over here the substrings are each robots tour

    temp_1 = copy.deepcopy(cl1)  # keeping a copy of cl1 and cl2 for later use (while forming child 1 and child2)
    temp_2 = copy.deepcopy(cl2)

    rbt_count = len(cl1.robots_assign)
    rbt_job = [[None]] * rbt_count  # initializing a list of list for the individual robot jobs for each robot
    start = end = 0
    for i in range(rbt_count):        #for each and every robot take out the tour for that robot
        end += cl1.robots_assign[i]
        rbt_job[i] = cl1[start:end]
        start = end
    child = []
    for entry in rbt_job:
        substring = entry
        substring_index = []
        sequence_source = random.choice([-1,-2,-3]) #pick out the ordering source for this particular robot tour
        if sequence_source == -1:   #if -1 then from the parent itself
            for value in substring:
                substring_index.append(cl1.index(value))
            sorted_substring = [x for _, x in sorted(zip(substring_index, substring))]
            child.extend(sorted_substring)
        if sequence_source == -2:   #if -2 then arrange w.r.t parent 2
            for value in substring:
                substring_index.append(cl2.index(value))
            sorted_substring = [x for _, x in sorted(zip(substring_index, substring))]
            child.extend(sorted_substring)
        if sequence_source == -3:   # if -3 then arrange w.r.t best
            for value in substring:
                substring_index.append(best.index(value))
            sorted_substring = [x for _, x in sorted(zip(substring_index, substring))]
            child.extend(sorted_substring)
    # print 'this is the final shape', child
    #once all the subtours are sorted then copy to the original child
    cl1[:] = child[:]

    # repeat the same for the 2nd child
    rbt_count = len(cl2.robots_assign)
    rbt_job = [[None]] * rbt_count  # initializing a list of list for the individual robot jobs for each robot
    start = end = 0
    for i in range(rbt_count):  # for each and every robot
        end += cl2.robots_assign[i]
        rbt_job[i] = cl2[start:end]
        start = end
    child = []
    for entry in rbt_job:
        substring = entry
        substring_index = []
        sequence_source = random.choice([-1, -2, -3])
        if sequence_source == -1:
            for value in substring:
                substring_index.append(cl2.index(value))
            sorted_substring = [x for _, x in sorted(zip(substring_index, substring))]
            child.extend(sorted_substring)
        if sequence_source == -2:
            for value in substring:
                substring_index.append(temp_1.index(value))
            sorted_substring = [x for _, x in sorted(zip(substring_index, substring))]
            child.extend(sorted_substring)
        if sequence_source == -3:
            for value in substring:
                substring_index.append(best.index(value))
            sorted_substring = [x for _, x in sorted(zip(substring_index, substring))]
            child.extend(sorted_substring)
    cl2[:] = child[:]
    return cl1, cl2

# shuffles a subportion of the mutant. The subportion will be 10% of the total size
def shuffle_mutation(mutant):
    ten_percent_size = int(len(mutant)*.1)
    id1 = random.randint(0, (len(mutant) - ten_percent_size))
    id2 = random.randint(id1, id1+ten_percent_size)
    subarray = mutant[id1:id2]  # plugging out the substring
    random.shuffle(subarray)    # shuffling it
    mutant[id1:id2] = subarray  # putting it back
    return mutant

#identify two ids and reverse the substring between those two ids.
def reverse_mutation(mutant):
    length = len(mutant)
    id1, id2 = random.sample(xrange(length), 2) #x range generates a list of numbers till the given range and then there are two random samples picked from that
    # mutant[id1], mutant[id2] = mutant[id2], mutant[id1]
    if id1 > id2:
        id1, id2 = id2, id1         # making sure that id1 is smaller than id2
    subarray = mutant[id1:id2]      # plugging out the substring
    subarray.reverse()              # reversing it
    mutant[id1:id2] = subarray      # putting it back

# Simply identifies two id's and swaps their values. irrespective of what robot they belong to.
def swap_mutation(mutant):
    id1, id2 = random.sample(xrange(len(mutant)), 2)
    mutant[id1], mutant[id2] = mutant[id2], mutant[id1]
    return mutant


# Identify two id's and swap the value between.
def gene_swap_mut(mutant, MUTPB):
    if len(mutant) <= 1:
        return mutant
    # for gene in mutant:     #Going through every gene
    if random.random() < MUTPB: #Checking if it is to be mutated or not
        index_a, index_b = random.sample(range(0, len(mutant)), 2)
        mutant[index_a], mutant[index_b] = mutant[index_b], mutant[index_a]

    ## mutate the 2nd part of the chromosome
    # mutant = Snd_Part_Muta1_Prob(mutant, MUTPB)
    # mutant =  Snd_Part_MutaBig_Prob(mutant, MUTPB)
    mutant = Snd_Part_MutaBig(mutant)
    # mutant = Snd_Part_Muta1(mutant)
    return mutant

# Identify two id's and insert the later one next to the earlier one.
def gene_insert_mut(mutant, MUTPB):
    if len(mutant) <= 1:
        return mutant
    # for gene in mutant:     #Going through every gene
    if random.random() < MUTPB: #Checking if it is to be mutated or not
        index_a, index_b = random.sample(range(0,len(mutant)), 2)
        if index_a > index_b:  # if index_b falls before index a then swap the two values
            index_a, index_b = index_b, index_a  # making sure that index_a is smaller than index_b

        ##*************************************************************************************************************
        start = 0
        for i, entry in enumerate(mutant.robots_assign):
            if index_b < start + entry:
                entry -= 1
                mutant.robots_assign[i] = entry
                break
            else:
                start += entry
        start = 0
        for i, entry in enumerate(mutant.robots_assign):
            if index_a < start + entry:
                entry += 1
                mutant.robots_assign[i] = entry
                break
            else:
                start += entry
        ##*************************************************************************************************************

        mutant.insert(index_a+1, mutant[index_b])
        del mutant[index_b+1]

    ## mutate the 2nd part of the chromosome
    # mutant = Snd_Part_Muta1_Prob(mutant, MUTPB)
    # mutant =  Snd_Part_MutaBig_Prob(mutant, MUTPB)
    mutant = Snd_Part_MutaBig(mutant)
    # mutant = Snd_Part_Muta1(mutant)
    return mutant



# Identifies two id and reverses the string between them
# the string length remains constantly random and does not change unlike gene_inverse_mut_II
def gene_inverse_mut(mutant, MUTPB):
    if len(mutant) <= 1:
        return mutant

    # for gene in mutant:     #Going through each gene in the chromosome
    if random.random() < MUTPB: # checking if it needs to be mutated or not.
        # print 'inverse mutation'
        index_a, index_b = random.sample(range(0, len(mutant)), 2)
        # index_a = random.randint(0, len(mutant)-1)
        # index_b = random.randint(0, len(mutant)-1)
        if index_a > index_b:       # if index_b falls before index a then swap the two values
            index_a, index_b = index_b, index_a  # making sure that index_a is smaller than index_b
        substring = mutant[index_a:index_b]
        substring.reverse()
        mutant[index_a:index_b] = substring
            # break     #I have tested removing a break statement from here. Wasn't giving very different results
    ## mutate the 2nd part of the chromosome
    # mutant =  Snd_Part_Muta1_Prob(mutant, MUTPB)
    # mutant =  Snd_Part_MutaBig_Prob(mutant, MUTPB)
    mutant =  Snd_Part_MutaBig(mutant)
    # mutant = Snd_Part_Muta1(mutant)

    return mutant

#Identifies two id and reverses the string between them
# In this routine the length of the substring to be randomly selected an reversed alters with the number of generation
# being run. For earlier generation the mutation is drastic. for later stages the string length gets smaller.
def gene_inverse_mut_II(mutant, MUTPB,step_size):   #step_size is the number of generations run so far.
    for gene in mutant: #for each gene in the chromosome
        if random.random() < MUTPB: #test the chances of it undergoing mutation.
            index_a = mutant.index(gene)
            size = int(len(mutant)/(step_size/500)) #here is where we figure out the potential size based on # of Generation
            # I didn't really like the point that the substring always lies infront of index_a think something about it
            if size > len(mutant):
                index_b = random.randint(index_a, index_a+len(mutant)-1)
            else:
                index_b = random.randint(index_a, index_a+size)
            substring = mutant[index_a:index_b]
            substring.reverse()
            mutant[index_a:index_b] = substring
            break
    ## mutate the 2nd part of the chromosome
    # mutant =  Snd_Part_Muta1_Prob(mutant, MUTPB)
    # mutant =  Snd_Part_MutaBig_Prob(mutant, MUTPB)
    mutant =  Snd_Part_MutaBig(mutant)
    # mutant = Snd_Part_Muta1(mutant)
    return mutant


# Mutation for the first part of the chromosome, gene based. selects an id and swaps it with its neighbour.
def gene_reverse_mut(mutant, MUTPB):
    for gene in mutant:
        if random.random() < MUTPB:
            index_a = mutant.index(gene)
            index_b = (index_a +1)%len(mutant)
            mutant[index_a], mutant[index_b] = mutant[index_b], mutant[index_a]
    return mutant

# Identifies a random id, makes a large addition or subtraction from it and does the opposite with the id next to it
def Snd_Part_MutaBig(mutant):
    # This portion checks for any failed robots (windowed version) if there are any then there allocations will be hard 0
    # and shouldn't be tinkered with
    MUTPB = 0.2 #0.2 / len(mutant.robots_assign)
    if random.random() < MUTPB:
        parameters = pickle.load(open("/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/parameters.pkl", 'rb'))
        allowed_ids = [i for i, x in enumerate(parameters['failed_robots']) if x == 1] #picking ids of working robots.
        id, id2 = random.sample(allowed_ids, 2)     #picking two ids from the allowed robots
        if mutant.robots_assign[id] == 0 and mutant.robots_assign[id2] == 0:
            return mutant
        elif mutant.robots_assign[id] == 0:
            number = random.randint(0, mutant.robots_assign[id2])
            mutant.robots_assign[id2] -= number
            mutant.robots_assign[id] += number
        elif mutant.robots_assign[id2] == 0:
            number = random.randint(0, mutant.robots_assign[id])
            mutant.robots_assign[id] -= number
            mutant.robots_assign[id2] += number
        else:
            number = random.randint(0, min(mutant.robots_assign[id], mutant.robots_assign[id2]))  # Max 10% of the total length of the chromosome
            if (mutant.robots_assign[id] - number) > 0:
                mutant.robots_assign[id] -= number
                mutant.robots_assign[id2] += number
            else:   # if the negative part doesn't work then go for the other way around, mostly works when the whole amount is
                # subtracted if gene value is 8 and 8 is being subtracted then the top portion wouldn't work this one will.
                number -= 1
                mutant.robots_assign[id] -= number
                mutant.robots_assign[id2] += number
    return mutant

# this is similar to mutation big. that routine brings a large change to a random id and the id next to it
# this routine just alters that random id by 1 and similarly the id next to it.
def Snd_Part_Muta1(mutant):
    # This portion checks for any failed robots (windowed version) if there are any then there allocations will be hard 0
    # and shouldn't be tinkered with
    MUTPB = 0.2#0.5 / len(mutant.robots_assign)
    if random.random() < MUTPB:
        parameters = pickle.load(
            open("/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/parameters.pkl", 'rb'))
        allowed_ids = [i for i, x in enumerate(parameters['failed_robots']) if x == 1 ]  # picking ids of working robots.
        id, id2 = random.sample(allowed_ids, 2)     #picking two ids from the allowed robots
        if (mutant.robots_assign[id]) > 0:
            # print 'undergoing mutation', mutant.robots_assign
            mutant.robots_assign[id] -= 1
            mutant.robots_assign[id2] += 1
        elif (mutant.robots_assign[id2]) > 0:
            # print 'undergoing mutation', mutant.robots_assign
            mutant.robots_assign[id] += 1
            mutant.robots_assign[id2] -= 1
        # print 'exiting mutation', mutant.robots_assign
    return mutant

# this is similar to mutation 1 but this does things probabilistically.
# Checks for the probability for each robot_assign number to undergo mutation if the number is
# selected then another number is picked randomly and mutation takes place. Either its addition
# or subtraction and if both aren't feasible then nothing happens.
def Snd_Part_Muta1_Prob(mutant, MUTPB):
    MUTPB = 0.3/len(mutant.robots_assign)
    # This portion checks for any failed robots (windowed version) if there are any then there allocations will be hard 0
    # and shouldn't be tinkered with
    parameters = pickle.load(open("/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/parameters.pkl", 'rb'))
    allowed_ids = [i for i, x in enumerate(parameters['failed_robots']) if x == 1 ]  # picking ids of working robots.
    for id in allowed_ids:  # for each gene in the chromosome
        if random.random() < MUTPB:  # test the chances of it undergoing mutation.
            id2 = random.choice(allowed_ids)     #picking two ids from the allowed robots
            if (mutant.robots_assign[id]) > 0:
                mutant.robots_assign[id] -= 1
                mutant.robots_assign[id2] += 1
            elif (mutant.robots_assign[id2]) > 0:
                mutant.robots_assign[id] += 1
                mutant.robots_assign[id2] -= 1
    return mutant

# Almost same as Snd_Part_MutaBig but works with MUTPB. So if the selected robot_assign number has to undergo mutation
# based on the probability check then another id is picked and a subtraction or addition is made while making sure the
# operation doesn't create a negative value. There is a chance that nothing would happen if the value to be subtracted
# has the potential to make both id and id2 zero on subtraction (the two cases).
def Snd_Part_MutaBig_Prob(mutant, MUTPB):
    MUTPB = 0.2/len(mutant.robots_assign)
    # This portion checks for any failed robots (windowed version) if there are any then there allocations will be hard 0
    # and shouldn't be tinkered with
    parameters = pickle.load(
        open("/home/usman/research_gaz_ros/src/turtlebot_research/src/txt_files/ST_SR_TA/parameters.pkl", 'rb'))
    allowed_ids = [i for i, x in enumerate(parameters['failed_robots']) if x == 1] #picking ids of working robots.
    for id in allowed_ids:
        if random.random() < MUTPB:
            id2 = random.choice(allowed_ids)     #picking two ids from the allowed robots
            number = random.randint(0,int(min(mutant.robots_assign[id], mutant.robots_assign[id2])/2))  # Max 10% of the total length of the chromosome
            # if (mutant.robots_assign[id] - number) > 0:
            mutant.robots_assign[id] -= number
            mutant.robots_assign[id2] += number
            # elif (mutant.robots_assign[id2] - number) > 0:   # if the negative part doesn't work then go for the other way around, mostly works when the whole amount is
            #     subtracted if gene value is 8 and 8 is being subtracted then the top portion wouldn't work this one will.
                # mutant.robots_assign[id] += number
                # mutant.robots_assign[id2] -= number
    return mutant

# 2nd part of the chromosome only
# pick a sub-array from within the 2nd part of the chromosome, if this random subarray is larger then 1 then reverse
# and replace.
def Snd_Part_Cros_Rvrs(chromo):
    id = random.randint(0, len(chromo.robots_assign)) # get a random ID
    if len(chromo.robots_assign[0:id]) > 1:           # if the portion between 0:id is larger than 1 then inverse
        chromo.robots_assign[0:id] = reversed(chromo.robots_assign[0:id])
    if len(chromo.robots_assign[id:]) > 1:            # if 0:id wasn't larger than one then try id:end
        chromo.robots_assign[id:] = reversed(chromo.robots_assign[id:]) # reverse the array


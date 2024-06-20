## I am making this file passively. Haven't observed the correct working of things. If something suspecious then please check.
import difflib
import copy
import itertools

# Routine which alters the fitness (increases) of solutions depending upon how densely populated their vicinity is
def fitness_sharing(hamming_matrix, pop):
    absolute_size = len(pop[0])
    for i,item in enumerate(hamming_matrix):    #outer loop for each row of hamming matrix
        denominator = 1
        sharing_distance = absolute_size*.95    #if the solution is 95 percent similar to the solution in question
        for j,value in enumerate(item):         # inner loop checking each value within each row.
            value = absolute_size-value         # Before same elements are represented as 0 in hamming matrix. Now they will be rep. as max value
            if value > sharing_distance and i != j: # If the match number is more than 95% match
                denominator += abs(1-((value)/(sharing_distance)))  #the conversion above was done coz of the value thing here. before the value was coming as zero always
        pop[i].fitness.values = map(lambda x: x*denominator, pop[i].fitness.values) #alter the fitness values.
    return pop


def compute_hamming_matrix(IND_SIZE, POP_SIZE, pop):
    frequency = [0] * (IND_SIZE)  # measure how many times a particular gnene is repeated within the pop
    hamming_matrix = [[0] * POP_SIZE for i in range(POP_SIZE)]  #initiate an empty hamming matrix with zeros all around
    for i in range(POP_SIZE):   #for 2D array using the double addressing using i and j
        for j in range(POP_SIZE):
            if i > j:  # if lower diagonal them copy paste from upper diagonal
                hamming_matrix[i][j] = hamming_matrix[j][i]  # only computing the upper diagonal (lower is same as upper)
            elif i != j:  # hamming not to be computed between the same chromosome (diagonal of the matrix).
                hamming_matrix[i][j] = hamming_dist(pop[i], pop[j], frequency)
            #since the matrix is initialized to all zeros the i,i (diagonal) will remain zero as its never computed
    return hamming_matrix

# Routine for hamming distance computation between chromosomes. The frequency array saves the frequency of repetation
# at each individual gene.
def hamming_dist(ind1, ind2, frequency):    #frequency can be passed as an empty list of zeros equal to the size of individual
    hamming = 0
    for i in range(len(ind1)):
        # hamming += abs(int(ind1[i].split(",")[0]) - int(ind2[i].split(",")[0])) #magnitude based hamming
        if abs(int(ind1[i].split(",")[0]) - int(ind2[i].split(",")[0])) == 0:#ind1_split[i] - ind2_split[i]) == 0: # Binary hamming
            # print "Gene ", i, "is the culprit :D"
            frequency[i] += 1   #frequency basically marks how many times a particular gene is repeated in the whole population
        else:                   #introduced it to identify potential problems in my working during GA analysis.
            hamming += 1
    return hamming

# This is a simple sequence match based distance measure using a built in library from python. There is not really
# any literature behind this work. This one is based on individual robots ratio. Since each ratio comes out as a low number
# between 0 and 1 if the two chromosomes are different I use it with a max function in the correlative family crowding
# I have implemented.
def seq_match_ratio(ind1, ind2):
    rbt_count = len(ind1.robots_assign)
    rbt_job1 = [[None]] * rbt_count # complete plans from the first chromosome
    rbt_job2 = [[None]] * rbt_count # complete plans from the 2nd chromosome
    start = end = 0
    for i in range(rbt_count):  # for each and every robot read the tour
        end += ind1.robots_assign[i]
        rbt_job1[i] = ind1[start:end]
        rbt_job1[i] = [int(j.split(",")[0]) for j in rbt_job1[i]]  # get rid of all the commas and sub jobs only the job ids
        start = end
    start = end = 0
    for i in range(rbt_count):  # for each and every robot read the tours
        end += ind2.robots_assign[i]
        rbt_job2[i] = ind2[start:end]
        rbt_job2[i] = [int(j.split(",")[0]) for j in rbt_job2[i]]  # get rid of all the commas and sub jobs only the job ids
        start = end
    ratio = 0
    for i in range(rbt_count):
        sm = difflib.SequenceMatcher(None,rbt_job1[i], rbt_job2[i])
        ratio += sm.ratio()
    ratio = ratio/rbt_count
    return ratio

# This is a simple sequence match based distance measure using a built in library from python. There is not really
# any literature behind this work. This one is based on complete solution ratios. Since each ratio comes out as a low number
# between 0 and 1 if the two chromosomes are different I use it with a max function in the correlative family crowding
# I have implemented.
def seq_match_ratio_full(ind1, ind2):
    rbt_job1 = [int(j.split(",")[0]) for j in ind1]  # get rid of all the commas and sub jobs only the job ids
    rbt_job2 = [int(j.split(",")[0]) for j in ind2]  # get rid of all the commas and sub jobs only the job ids
    return difflib.SequenceMatcher(None,rbt_job1, rbt_job2)

# These are two functions based on the edit distance. These are two implementations copied from the internet with the
# hope that they would work fine. Have tested the two on some basic chromosomes copied from running instances of RoSTAM
def edit_dist(ind1, ind2):  #levenshtein distance. Watch video at youtube to understand (minimum edit distance dynamic programming)
    a = copy.deepcopy(ind1)
    b = copy.deepcopy(ind2)

    for i, value in enumerate(a):
        a[i] = int(value.split(",")[0])
    for i, value in enumerate(b):
        b[i] = int(value.split(",")[0])

    n, m = len(a), len(b)
    if n > m:
        a, b = b, a
        n, m = m, n

    current = range(n + 1)

    for i in range(1, m + 1):
        previous, current = current, [i] + [0] * n
        for j in range(1, n + 1):
            add, delete = previous[j] + 1, current[j - 1] + 1
            change = previous[j - 1]
            if a[j - 1] != b[i - 1]:
                change = change + 1
            current[j] = min(add, delete, change)
    return current[n]


# These are two functions based on the edit distance. These are two implementations copied from the internet with the
# hope that they would work fine. Have tested the two on some basic chromosomes copied from running instances of RoSTAM
def edit_dist2(ind1, ind2):  #levenshtein distance. Watch video at youtube to understand (minimum edit distance dynamic programming)
    a = copy.deepcopy(ind1)
    b = copy.deepcopy(ind2)
    for i, value in enumerate(a):
        a[i] = int(value.split(",")[0])
    for i, value in enumerate(b):
        b[i] = int(value.split(",")[0])

    if a == b:
        return 0
    if len(a) > len(b):
        a, b = b, a
    if not a:
        return len(b)

    previous_row = range(len(b) +1)
    for i, column1 in enumerate(a):
        current_row = [i+1]
        for j, column2 in enumerate(b):
            insertions = previous_row[j+1] +1
            deletions = current_row[j] +1
            substitutions = previous_row[j] + (column1 != column2)
            current_row.append(min(insertions, deletions, substitutions))
        previous_row = current_row
    return previous_row[-1]
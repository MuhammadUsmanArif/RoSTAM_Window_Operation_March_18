import random
import copy
import numpy

# RBS for two parents selection at a time
def roulette_select_RBS_ind(population, pop_size):
    sorted_population = sorted(population, key=lambda x: x.fitness.values[0], reverse= True)
    # for ind in sorted_population:
    #     print ind.fitness.values[0]
    rank = range(1, len(population)+1)
    # print rank
    total_fitness_rank = sum(range(1, len(population)+1))           # totalling the fitness for calculating the share of each ind.
    rel_fitness = [f/float(total_fitness_rank) for f in rank]  # relative fitness (share in the pie)
    # print 'relative fitness: ', rel_fitness
    # Generate probability intervals for each individual
    probs = [sum(rel_fitness[:i+1]) for i in range(len(rel_fitness))] # this is the pie :)
    # Draw new population
    new_population = []
    # print ' The pie chart: ', probs
    for n in xrange(pop_size):
        r = random.random()
        for (i, individual) in enumerate(sorted_population):
            if r <= probs[i]:           # looking for the first slot where the random # r fits
                # print 'rank selected: ', i
                new_population.append(individual)   # saving that value into our population
                break
    return new_population[0], new_population[1]

#this RBS is for population selection. Will return a whole population of solutions
def roulette_select_RBS(population, pop_size):
    sorted_population = sorted(population, key=lambda x: x.fitness.values[0], reverse= True)
    # for ind in sorted_population:
    #     print ind.fitness.values[0]
    rank = range(1, len(population)+1)
    # print rank
    total_fitness_rank = sum(range(1, len(population)+1))           # totalling the fitness for calculating the share of each ind.
    rel_fitness = [f/float(total_fitness_rank) for f in rank]  # relative fitness (share in the pie)
    # print 'relative fitness: ', rel_fitness
    # Generate probability intervals for each individual
    probs = [sum(rel_fitness[:i+1]) for i in range(len(rel_fitness))] # this is the pie :)
    # Draw new population
    new_population = []
    # print ' The pie chart: ', probs
    for n in xrange(pop_size):
        r = random.random()
        for (i, individual) in enumerate(sorted_population):
            if r <= probs[i]:           # looking for the first slot where the random # r fits
                # print 'rank selected: ', i
                new_population.append(individual)   # saving that value into our population
                break
    return new_population

# This FPS is majorly for parent selection, returns two parents.
def roulette_select_FPS_ind(population, num):
    sorted_population = sorted(population, key = lambda x: x.fitness.values[0])
    fitnesses = list()
    for ind in sorted_population:
        fitnesses.append(1.0/ind.fitness.values[0])     # populating the fitness list
    total_fitness = float(sum(fitnesses))           # totalling the fitness for calculating the share of each ind.
    rel_fitness = [f/total_fitness for f in fitnesses]  # relative fitness (share in the pie)
    # Generate probability intervals for each individual
    probs = [sum(rel_fitness[:i+1]) for i in range(len(rel_fitness))] # this is the pie :)
    # Draw new population
    new_population = []
    for n in xrange(num):
        r = random.random()
        for (i, individual) in enumerate(sorted_population):
            if r <= probs[i]:           # looking for the first slot where the random # r fits
                new_population.append(individual)   # saving that value into our population
                # print "parent id and fitness: ", i, individual.fitness.values[0]
                # print individual
                break
    return new_population[0], new_population[1]

# population based FPS. return a whole population of RBS selected individuals.
def roulette_select_FPS(population, num):
    fitnesses = list()
    for ind in population:
        fitnesses.append(1.0/ind.fitness.values[0])     # populating the fitness list
    total_fitness = float(sum(fitnesses))           # totalling the fitness for calculating the share of each ind.
    rel_fitness = [f/total_fitness for f in fitnesses]  # relative fitness (share in the pie)
    # Generate probability intervals for each individual
    probs = [sum(rel_fitness[:i+1]) for i in range(len(rel_fitness))] # this is the pie :)
    # Draw new population
    new_population = []
    for n in xrange(num):
        r = random.random()
        for (i, individual) in enumerate(population):
            if r <= probs[i]:           # looking for the first slot where the random # r fits
                new_population.append(individual)   # saving that value into our population
                # print "parent id and fitness: ", i, individual.fitness.values[0]
                break
    return new_population

# provides n individuals from the population using stochastic universal selection scheme.
def stochastic_universal_ind(population, num):
    fitnesses = list()
    for ind in population:  #Since my work is on minimization so gonna reciprocate all the fitness values to get even distribution
        fitnesses.append(1.0/ind.fitness.values[0])     # populating the fitness list
    total_fitness = float(sum(fitnesses))           # totalling the fitness for calculating the share of each ind.
    rel_fitness = [f/total_fitness for f in fitnesses]  # relative fitness (share in the pie)
    # Generate probability intervals for each individual
    probs = [sum(rel_fitness[:i+1]) for i in range(len(rel_fitness))] # this is the pie :)
    # Draw new population
    new_population = []
    pointer_spacing = 1/(num*1.0)   # pointer spacing depends on the number of chromosomes to be picked.
    pointer = random.uniform(0,pointer_spacing)     #draw one random number to pick the first parent. All other parents will be drawn by even distance based pointers
    for n in xrange(num):
        for (i, individual) in enumerate(population):
            if pointer <= probs[i]:           # looking for the first slot where the random # r fits
                new_population.append(individual)   # saving that value into our population
                # print "parent id and fitness: ", i, individual.fitness.values[0]
                # print individual
                break
        pointer += pointer_spacing #Evenly placed pointers
    return new_population[0], new_population[1]


def select_worst(population, size):
    new_population = sorted(population, key=lambda x: x.fitness.values[0])
    new_population = new_population[-size:]
    random.shuffle(new_population)
    return new_population

def truncation(population, size):
    new_population = sorted(population, key=lambda x: x.fitness.values[0])
    new_population = new_population[:size]
    random.shuffle(new_population)
    return new_population

def Binary_Tournament(population, pop_size, T_size):
    new_population = []
    for i in range(pop_size):
        Tour_teams = []
        for j in range(T_size):
            Tour_teams.append(random.choice(population))
        new_population.append(min(Tour_teams, key=lambda item: item.fitness.values[0]))
    return new_population
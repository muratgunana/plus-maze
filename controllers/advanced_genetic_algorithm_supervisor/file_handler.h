#ifndef FILEHANDLER_H
#define FILEHANDLER_H

#include <string.h>
#include <stdio.h>
#include "genotype.h"
#include "population.h"

void read_best_population(Population p);

void write_best_population(Population p);

void read_best_genotype(Genotype g);

void write_best_genotype(Genotype g);

#endif

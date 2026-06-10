#include <stdio.h>
#include <stdlib.h> // random
#include <time.h> // for random results
#include "workspaceAnalysis.h"

// OpenMP
#ifdef _OPENMP
#include <omp.h>
#endif

void workspaceAnalysis(RobotArm6DoF* ramie, size_t pointsNumber, double* punkty_x, double* punkty_y, double* punkty_z, double* yoshikawas)
{
    // 1. losujemy duzo roznych tet
    // 2. z tego zbieramy z fk punkty docelowe
    // i tyle

    // srand(time(NULL)); //use current time as seed for random generator

    // dyrektywa openMP
    #pragma omp parallel for
    for(size_t j = 0; j<pointsNumber; j++)
    {

        // EKLARACJE W ŚRODKU PĘTLI - dzięki temu każdy wątek dostaje własną, prywatną pamięć na te zmienne.
        double thetas[6];
        Matrix44 fkResults[6];
        Matrix66 jacobian;

        // losujemy zestaw katow
        unsigned int seed = (unsigned int)(time(NULL) ^ j);
        for(size_t i = 0; i<6; i++)
        {
            
            // double r = (double)rand_r(&seed) / RAND_MAX; // random number z zakresu <0,1>
            // WŁASNY GENERATOR LOSOWY 
            // Zadiała wszędzie: Linux, Windows, macOS. Brak konfliktów z rand() czy rand_r().
            seed = (seed * 1103515245 + 12345) & 0x7fffffff;
            double r = (double)seed / 0x7fffffff; // wynik od 0.0 do 1.0

            thetas[i] = r * M_PI * 2; // teraz zakres to <0,2pi>
            thetas[i] -= M_PI; // teraz zakres to <-pi,pi> to co chcielismy
        }

        // odpalamy na tym fk
        forwardKinematics(ramie, thetas, fkResults);

        // zbieramy pozycje koncowki
        punkty_x[j] = fkResults[5].data[3];
        punkty_y[j] = fkResults[5].data[7];
        punkty_z[j] = fkResults[5].data[11];

        // znajdujemy index yoshikawa tego punktu
        calculateJacobian(ramie, thetas, &jacobian);
        yoshikawas[j] = calculateManipulabilityYoshikawaIndex(&jacobian);
    }
}
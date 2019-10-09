#include <stdio.h>
#include <stdlib.h>

#include "CPLEXsolver_cwrapper.h"

int main(int argc, char **argv)
{
    int res;

    /* Init library */
    res = init_CPLEXsolver_cwrapper(3, 0, 0);
    printf("init_CPLEXsolver_cwrapper result: %d\n\n", res);

    /* Init problem */
    res = initProblem();
    printf("initProblem result: %d\n\n", res);

    /* Set problem */
    double H[9] = {
         3.0, 0.0, -1.0,
         0.0, 2.0,  0.0,
        -1.0, 0.0,  1.0
    };

    double f[3] = {-2.0, 3.0, 1.0};

    res = setProblem(H, f);
    printf("setProblem result: %d\n\n", res);

    /* Solve problem */
    double* result_ptr = (double*) malloc(sizeof(double)*3);

    res = solveProblem(result_ptr);
    printf("solveProblem result: %d\n", res);
    printf("result: %.2f %.2f %.2f\n\n", result_ptr[0], result_ptr[1], result_ptr[2]);

    /* Deinit library */
    res = deinit_CPLEXsolver_cwrapper();
    printf("deinit_CPLEXsolver_cwrapper result: %d\n\n", res);

    return 0;
}
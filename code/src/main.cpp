#include <cstdio>
#include <iostream>
#include <fstream>
#include <chrono>
#include "MAPFLoader.hpp"
#include "CBSSolver.hpp"
#include "TesterUtils.hpp"
#include <getopt.h>

int main(int argc, char *argv[])
{
    // parse commandline options ////////////////////////////////////////////
    int opt;
    std::string inputFile;
    std::string outputFile = "../outputs/result.txt";
    bool useParallel = true;

    static struct option long_options[] = {
        {"output",      1, 0, 'o'},
        {"input",       1, 0, 'i'},
        {"sequential",  1, 0, 's'},
        {0, 0, 0, 0}};

    while ((opt = getopt_long(argc, argv, "o:i:s", long_options, NULL)) != EOF)
    {
        switch (opt)
        {
        case 'o':
            outputFile = optarg;
            break;
        case 'i':
            inputFile = optarg;
            break;
        case 's':
            useParallel = false;
            break;
        default:
            printf("Command line arguments invalid\n");
            return 1;
        }
    }
    // end parsing of commandline options //////////////////////////////////////

    // Load MAPF problem
    MAPFLoader loader;

    MAPFInstance mapfProblem = loader.loadInstanceFromFile(inputFile);

    // Run specific version of CBS
    CBSSolver solver(mapfProblem);
    std::vector<std::vector<Point2>> solution;

    auto start = std::chrono::high_resolution_clock::now();

    if (useParallel)
    {
        printf("Using parallel CBS\n");
        solution = solver.solveParallel(mapfProblem);
    }
    else
    {
        printf("Using sequential CBS\n");
        solution = solver.solve(mapfProblem);
    }
    auto end = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

    int SOC = 0;
    for (const auto &path : solution)
        SOC += path.size() - 1;
    printf("CBS took: %.3fs\nSOC = %d\n", duration.count() * 1e-6, SOC);

    saveToFile(outputFile, inputFile, solution);
}
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <iomanip>
#include <string>
#define PATH_PLAN_FILE_NAME "C:/Users/Lachlan/Documents/MTRN4110/PathPlan.txt"

int main()
{
    //declare input file stream
    std::ifstream inFile(PATH_PLAN_FILE_NAME);
    
    //copy contents of inFile to path
    std::string path;
    getline(inFile,path);

    int InitialRow = (path[0]) - '0';
    int InitialColumn = (path[1]) - '0';
    char InitialHeading = path[2];
    int InitialStep = 00;
    
    std::cout << "Step: " << std::setfill('0') << std::setw(2) << InitialStep << ", Row: " << InitialRow << ", Column: " << InitialColumn << ", Heading: " << InitialHeading << std::endl;
    
    inFile.close();
    return 0;
};

// File:          z5166028_MTRN4110_PhaseB.cpp
// Date:          
// Description:   Controller of E-puck for Phase B - Path Planning
// Author:        Colin Li
// Modifications:
// Platform:      Windows
// Notes:         

#include <webots/Robot.hpp>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <vector>
#include <queue>

// All the webots classes are defined in the "webots" namespace
using namespace webots;

#define CELLROWS 5
#define CELLCOLS 9

class Cell {
    public:
        Cell();
        Cell(int, int, int, int, int, int);
        int value; // Flood Fill value/ distance from target
        int north,east,south,west; // existance of a wall w/regard to the cell
        int xPos,yPos; // row position, column position
};

Cell::Cell () {
    value = CELLROWS*CELLCOLS;
}

Cell::Cell (int a, int b, int N, int E, int S, int W) {
    value = CELLROWS*CELLCOLS;
    xPos = a;
    yPos = b;
    north = N;
    east = E;
    south = S;
    west = W;
}



int main(int argc, char **argv) {

    // Initialise file paths
    const std::string MAP_FILE_NAME = "../../Map.txt";
    const std::string PATH_PLAN_FILE_NAME = "../../PathPlan.txt";
    const std::string OUTPUT_FILE_NAME = "../../Output.txt";
    
    // Initialise variables
    std::string mapLine;
    char map[11][37];
    int hWall[CELLROWS+1][CELLCOLS];
    int vWall[CELLROWS][CELLCOLS+1];
    int i = 0;
    int j = 0;
    int x = 0;
    int xH = 0;
    int xV = 0;
    int y = 0;
    int targetCellPos[2] = {0,0};
    int initPos[2] = {0,0};
    char initHead;
    char head;
    Cell* maze[CELLROWS][CELLCOLS];
    
    // Write to Output.txt
    std::ofstream writeOutputFile(OUTPUT_FILE_NAME);
    
    std::cout << "[z5166028_MTRN4110_PhaseB] ";
    std::cout << "Reading in map from ../../Map.txt...\n";
    writeOutputFile << "[z5166028_MTRN4110_PhaseB] ";
    writeOutputFile << "Reading in map from ../../Map.txt...\n";
    
    // Reading Map.txt file
    std::ifstream readMapFile(MAP_FILE_NAME);
    while (std::getline(readMapFile,mapLine)) {
        std::cout << "[z5166028_MTRN4110_PhaseB] ";
        writeOutputFile << "[z5166028_MTRN4110_PhaseB] ";
        // Covert single string line into 2d arrray of char
        for (j = 0; j < 37; j++) {
            map[i][j] = mapLine[j];
            std::cout << map[i][j];
            writeOutputFile << map[i][j];
        }
        std::cout << "\n";
        writeOutputFile << "\n";
        i++;
    }
    readMapFile.close();
    
    std::cout << "[z5166028_MTRN4110_PhaseB] ";
    std::cout << "Map read in!" << std::endl;
    writeOutputFile << "[z5166028_MTRN4110_PhaseB] ";
    writeOutputFile << "Map read in!" << std::endl;

    // Convert map into values for walls and key positions
    for (i = 0; i < CELLROWS*2+1; i++) {    
        // Even Row == Horizontal Walls
        if (i % 2 == 0 ) {
        
            for (j = 0; j < CELLCOLS; j++) {
                // check for existance of a horizontal wall
                if (map[i][j*4+2] == '-') {
                    hWall[xH][y] = 1;
                } else {
                    hWall[xH][y] = 0;
                }
                y++;
            }
            y = 0;
            xH++;
        }
        // Odd Row == Vertical Walls and Key Positions
        else {
            
            for (j = 0; j < CELLCOLS+1; j++) {
                // check for existance of a verticle wall
                if (map[i][j*4] == '|') {
                    vWall[xV][y] = 1;
                } else {
                    vWall[xV][y] = 0;
                }
                // Locate key positions
                // x for target cell
                // v,<,>,^ for start cell & heading
                if (j < CELLCOLS && map[i][j*4+2] == 'x') {
                    targetCellPos[0] = xV;
                    targetCellPos[1] = y;
                } else if (j < CELLCOLS && map[i][j*4+2] == '^') {
                    initPos[0] = xV;
                    initPos[1] = y;
                    initHead = 'N';
                } else if (j < CELLCOLS && map[i][j*4+2] == '>') {
                    initPos[0] = xV;
                    initPos[1] = y;
                    initHead = 'E';
                } else if (j < CELLCOLS && map[i][j*4+2] == 'v') {
                    initPos[0] = xV;
                    initPos[1] = y;
                    initHead = 'S';
                } else if (j < CELLCOLS && map[i][j*4+2] == '<') {
                    initPos[0] = xV;
                    initPos[1] = y;
                    initHead = 'W';
                }
                y++;
            }
            y = 0;
            xV++;
        }
    }
    
    // Convert wall matrices to cell matrix
    for (x = 0; x < CELLROWS; x++) {
        for (y = 0; y < CELLCOLS; y++) {
            // Create a new cell with position,and walls that it has
            Cell* newCell = new Cell(x,y,hWall[x][y],vWall[x][y+1],hWall[x+1][y],vWall[x][y]);
            // If the cell is the target position, set the value to 0 for Flood Fill
            if (x == targetCellPos[0] && y == targetCellPos[1]) {
                newCell->value = 0;
            }
            // Add the cell to the grid of cells to build up a complete maze map
            maze[x][y] = newCell;
        }
    }

    // Flood Fill
    // Initialise
    // Cell values initialised, target cell initialised
    int currentExploredValue = 0;
    int mazeValueChanged = 1;
    while (mazeValueChanged != 0) {
        mazeValueChanged = 0;
        for (x = 0; x < CELLROWS; x++) {
            for (y = 0; y < CELLCOLS; y++) {
                if (maze[x][y]->value == currentExploredValue) {
                    if (maze[x][y]->north == 0) {
                        if (maze[x-1][y]->value == CELLROWS*CELLCOLS) {
                            maze[x-1][y]->value = maze[x][y]->value+1;
                            mazeValueChanged = 1;
                        }
                    }
                    if (maze[x][y]->east == 0) {
                        if (maze[x][y+1]->value == CELLROWS*CELLCOLS) {
                            maze[x][y+1]->value = maze[x][y]->value+1;
                            mazeValueChanged = 1;
                        }
                    }
                    if (maze[x][y]->south == 0) {
                        if (maze[x+1][y]->value == CELLROWS*CELLCOLS) {
                            maze[x+1][y]->value = maze[x][y]->value+1;
                            mazeValueChanged = 1;
                        }
                    }
                    if (maze[x][y]->west == 0) {
                        if (maze[x][y-1]->value == CELLROWS*CELLCOLS) {
                            maze[x][y-1]->value = maze[x][y]->value+1;
                            mazeValueChanged = 1;
                        }
                    }
                }
            }
        }
        currentExploredValue++;
    }
    
    // Pathfinder
    // BFS to find all paths
    std::cout << "[z5166028_MTRN4110_PhaseB] ";
    std::cout << "Finding shortest paths..." << std::endl;
    writeOutputFile << "[z5166028_MTRN4110_PhaseB] ";
    writeOutputFile << "Finding shortest paths..." << std::endl;
    
    std::vector<Cell*> start;
    std::vector<std::vector<Cell*>> foundPaths;
    std::queue<std::vector<Cell*>> q;
    // initialise the path at the starting position
    start.push_back(maze[initPos[0]][initPos[1]]);
    q.push(start);
    
    while (!q.empty()) {
        // access next cell in queue
        std::vector<Cell*> currentPath = q.front();
        q.pop();
        // access last cell in the path to explore
        Cell* exploredCell = currentPath.back();
        x = exploredCell->xPos;
        y = exploredCell->yPos;
        // when at target desination push complete path to vector of paths
        if (exploredCell->value == 0) {
            foundPaths.push_back(currentPath);
        }
        // check for valid move in north direction
        if (exploredCell->north == 0) {
            // if valid create a new path that contains the current path plus new cell
            if (maze[x-1][y]->value == exploredCell->value-1) {
                std::vector<Cell*> branchPath = currentPath;
                branchPath.push_back(maze[x-1][y]);
                q.push(branchPath);
            }
        }
        // check for valid move in east direction
        if (exploredCell->east == 0) {
            
            if (maze[x][y+1]->value == exploredCell->value-1) {
                std::vector<Cell*> branchPath = currentPath;
                branchPath.push_back(maze[x][y+1]);
                q.push(branchPath);
            }
        }
        // check for valid move in south direction
        if (exploredCell->south == 0) {
        
            if (maze[x+1][y]->value == exploredCell->value-1) {
                std::vector<Cell*> branchPath = currentPath;
                branchPath.push_back(maze[x+1][y]);
                q.push(branchPath);
            }
        }
        // check for valid move in west direction
        if (exploredCell->west == 0) {
        
            if (maze[x][y-1]->value == exploredCell->value-1) {
                std::vector<Cell*> branchPath = currentPath;
                branchPath.push_back(maze[x][y-1]);
                q.push(branchPath);
            }
        }
    }
    
    // Output paths found
    for (i = 0; i < (int)foundPaths.size(); i++) {
        std::cout << "[z5166028_MTRN4110_PhaseB] ";
        std::cout << "Path - " << i+1 << ":" << std::endl;
        writeOutputFile << "[z5166028_MTRN4110_PhaseB] ";
        writeOutputFile << "Path - " << i+1 << ":" << std::endl;

        // Path being outputed this iteration
        std::vector<Cell*> path = foundPaths.at(i);

        for (x = 0; x < 11; x++) {
            std::cout << "[z5166028_MTRN4110_PhaseB] ";
            writeOutputFile << "[z5166028_MTRN4110_PhaseB] ";
            // Even rows are for horizontal walls which stay the same
            if (x % 2 == 0) {
                
                for (y = 0; y < 37; y++) {
                    std::cout << map[x][y];
                    writeOutputFile << map[x][y];
                }
            }
            // Odd rows contain vertical walls and starting position that are the same
            // Must also print the FloodFill value of the path taken
            else {
                
                for (y = 0; y < 37; y++) {
                    // Printing vertical walls
                    if (y % 4 == 0) {
                        std::cout << map[x][y];
                        writeOutputFile << map[x][y];
                    }
                    else if (y % 2 == 0) {
                        // Boolian checking for empty spaces that need to be outputted
                        bool onPath = 0;
                        // Print starting position
                        if (map[x][y] == 'v' || map[x][y] == '>' || map[x][y] == '^' || map[x][y] == '<') {
                            std::cout << " " << map[x][y] << " ";
                            writeOutputFile << " " << map[x][y] << " ";
                            onPath = 1;
                        }
                        // Print values of cells on the path
                        for (j = 1; j < (int)path.size(); j++) {
                            //  j = 1, skipped start position
                            Cell* cell = path.at(j);
                            if (cell->xPos == (x-1)/2 && cell->yPos == (y-2)/4) {
                                std::cout << " " << std::setw(2) << std::left << cell->value;
                                writeOutputFile << " " << std::setw(2) << std::left << cell->value;
                                onPath = 1;
                            }
                        }
                        // If not start position or on the path, output appropriate spacing
                        if (onPath == 0) {
                            std::cout << "   ";
                            writeOutputFile << "   ";
                        }
                    }
                }
            }
            // next row
            std::cout << std::endl;
            writeOutputFile << std::endl;
        }
    }
    std::cout << "[z5166028_MTRN4110_PhaseB] ";
    std::cout << foundPaths.size() << " shortest paths found!" << std::endl;
    writeOutputFile << "[z5166028_MTRN4110_PhaseB] ";
    writeOutputFile << foundPaths.size() << " shortest paths found!" << std::endl;
    
    // Path planning and
    // Find path with the least turns
    std::cout << "[z5166028_MTRN4110_PhaseB] ";
    std::cout << "Finding shortest path with least turns..." << std::endl;
    writeOutputFile << "[z5166028_MTRN4110_PhaseB] ";
    writeOutputFile << "Finding shortest path with least turns..." << std::endl;
    
    std::vector<std::vector<char>> allPlans;

    for (i = 0; i < (int)foundPaths.size(); i++) {
    
        std::vector<char> plan;
        // Push initial position and heading on PathPlan
        plan.push_back(initPos[0]+'0');
        plan.push_back(initPos[1]+'0');
        plan.push_back(initHead);
        // accessing the path on current iteration
        std::vector<Cell*> path = foundPaths.at(i);
        head = initHead;
        
        for (j = 0; j < (int)path.size()-1; j++) {
            Cell* currCell = path.at(j);
            Cell* nextCell = path.at(j+1);
            // Next cell = north
            if (currCell->xPos-1 == nextCell->xPos) {
                if (head == 'N') {
                    plan.push_back('F');
                }
                else if (head == 'E') {
                    plan.push_back('L');
                    plan.push_back('F');
                }
                else if (head == 'S') {
                    plan.push_back('L');
                    plan.push_back('L');
                    plan.push_back('F');
                }
                else if (head == 'W') {
                    plan.push_back('R');
                    plan.push_back('F');
                }
                head = 'N';
            }
            // Next cell = east
            else if (currCell->yPos+1 == nextCell->yPos) {
                if (head == 'N') {
                    plan.push_back('R');
                    plan.push_back('F');
                }
                else if (head == 'E') {
                    plan.push_back('F');
                }
                else if (head == 'S') {
                    plan.push_back('L');
                    plan.push_back('F');
                }
                else if (head == 'W') {
                    plan.push_back('L');
                    plan.push_back('L');
                    plan.push_back('F');
                }
                head = 'E';
            }
            // Next cell = south
            else if (currCell->xPos+1 == nextCell->xPos) {
                if (head == 'N') {
                    plan.push_back('L');
                    plan.push_back('L');
                    plan.push_back('F');
                }
                else if (head == 'E') {
                    plan.push_back('R');
                    plan.push_back('F');
                }
                else if (head == 'S') {
                    plan.push_back('F');
                }
                else if (head == 'W') {
                    plan.push_back('L');
                    plan.push_back('F');
                }
                head = 'S';
            }
            // Next cell = west
            else if (currCell->yPos-1 == nextCell->yPos) {
                if (head == 'N') {
                    plan.push_back('L');
                    plan.push_back('F');
                }
                else if (head == 'E') {
                    plan.push_back('L');
                    plan.push_back('L');
                    plan.push_back('F');
                }
                else if (head == 'S') {
                    plan.push_back('R');
                    plan.push_back('F');
                }
                else if (head == 'W') {
                    plan.push_back('F');
                }
                head = 'W';
            }
        }
        allPlans.push_back(plan);
    }
    // Least turns == shortest path plan
    int shortestPathPlan = 0;
    int smallestPlanSize = 10000;
    
    for (i = 0; i < (int)allPlans.size(); i++) {
        std::vector<char> plan = allPlans.at(i);
        int planSize = plan.size();
        if (planSize < smallestPlanSize) {
            smallestPlanSize = planSize;
            shortestPathPlan = i;
        }
    }
    // Output same as lines 280-341 with 1 iteration at shortestPathPlan
    // Path being outputed this iteration
    std::vector<Cell*> path = foundPaths.at(shortestPathPlan);
    for (x = 0; x < 11; x++) {
        std::cout << "[z5166028_MTRN4110_PhaseB] ";
        writeOutputFile << "[z5166028_MTRN4110_PhaseB] ";
        // Even rows are for horizontal walls which stay the same
        if (x % 2 == 0) {
            for (y = 0; y < 37; y++) {
                std::cout << map[x][y];
                writeOutputFile << map[x][y];
            }
        }
        // Odd rows contain vertical walls and starting position that are the same
        // Must also print the FloodFill value of the path taken
        else {
            for (y = 0; y < 37; y++) {
                // Printing vertical walls
                if (y % 4 == 0) {
                    std::cout << map[x][y];
                    writeOutputFile << map[x][y];
                }
                else if (y % 2 == 0) {
                    // Boolian checking for empty spaces that need to be outputted
                    bool onPath = 0;
                    // Print starting position
                    if (map[x][y] == 'v' || map[x][y] == '>' || map[x][y] == '^' || map[x][y] == '<') {
                        std::cout << " " << map[x][y] << " ";
                        writeOutputFile << " " << map[x][y] << " ";
                        onPath = 1;
                    }
                    // Print values of cells on the path
                    for (j = 1; j < (int)path.size(); j++) {
                        //  j = 1, skipped start position
                        Cell* cell = path.at(j);
                        if (cell->xPos == (x-1)/2 && cell->yPos == (y-2)/4) {
                            std::cout << " " << std::setw(2) << std::left << cell->value;
                            writeOutputFile << " " << std::setw(2) << std::left << cell->value;
                            onPath = 1;
                        }
                    }
                    // If not start position or on the path, output appropriate spacing
                    if (onPath == 0) {
                        std::cout << "   ";
                        writeOutputFile << "   ";
                    }
                }
            }
        }
        // next row
        std::cout << std::endl;
        writeOutputFile << std::endl;
    }
    
    std::cout << "[z5166028_MTRN4110_PhaseB] ";
    std::cout << "Shortest path with least turns found!" << std::endl;
    std::cout << "[z5166028_MTRN4110_PhaseB] ";
    std::cout << "Path Plan (" << smallestPlanSize-3 << " steps): ";
    writeOutputFile << "[z5166028_MTRN4110_PhaseB] ";
    writeOutputFile << "Shortest path with least turns found!" << std::endl;
    writeOutputFile << "[z5166028_MTRN4110_PhaseB] ";
    writeOutputFile << "Path Plan (" << smallestPlanSize-3 << " steps): ";
    std::vector<char> plan = allPlans.at(shortestPathPlan);
    for (j = 0; j < (int)plan.size(); j++) {
        char c = plan.at(j);
        std::cout << c;
        writeOutputFile << c;
    }
    std::cout << std::endl;
    writeOutputFile << std::endl;
    // Write path plan to PathPlan.txt
    std::cout << "[z5166028_MTRN4110_PhaseB] ";
    std::cout << "Writing path plan to ../../PathPlan.txt..." << std::endl;
    writeOutputFile << "[z5166028_MTRN4110_PhaseB] ";
    writeOutputFile << "Writing path plan to ../../PathPlan.txt..." << std::endl;
    
    std::ofstream writePathPlan(PATH_PLAN_FILE_NAME);
    for (j = 0; j < (int)plan.size(); j++) {
        char c = plan.at(j);
        writePathPlan << c;
    }
    writePathPlan.close();
    
    std::cout << "[z5166028_MTRN4110_PhaseB] ";
    std::cout << "Path plan written to ../../PathPlan.txt!" << std::endl;
    writeOutputFile << "[z5166028_MTRN4110_PhaseB] ";
    writeOutputFile << "Path plan written to ../../PathPlan.txt!" << std::endl;
    
    writeOutputFile.close();
    
    /*
    std::vector<int> path;
    int pathFound = 0;
    int pathStep = 0;
    x = initPos[0];
    y = initPos[1];
    head = initHead;
    path.push_back(x);
    path.push_back(y);
    */

    
    /*for (int i = 0; i < path.size()/2; i++) {
        std::cout << "(" << path[2*i] << "," << path[2*i+1] << "), ";
    }*/
    /*
    while (pathFound == 0) {

        if (maze[x][y]->north == 0 && pathStep == 0 && x > 0) {
            if (maze[x-1][y]->value == maze[x][y]->value-1) {
                x--;
                pathStep = 1;
            }
        }
        if (maze[x][y]->east == 0 && pathStep == 0 && y < CELLCOLS-1) {
            if (maze[x][y+1]->value == maze[x][y]->value-1) {
                y++;
                pathStep = 1;
            }
        }
        if (maze[x][y]->south == 0 && pathStep == 0 && x < CELLROWS-1) {
            if (maze[x+1][y]->value == maze[x][y]->value-1) {
                x++;
                pathStep = 1;
            }
        }
        if (maze[x][y]->west == 0 && pathStep == 0 && y > 0) {
            if (maze[x][y-1]->value == maze[x][y]->value-1) {
                y--;
                pathStep = 1;
            }
        }
        path.push_back(x);
        path.push_back(y);
        pathStep = 0;
        if (maze[x][y]->value == 0){
            pathFound = 1;
        }
    }*/
    
    /*
    for (x = 0; x < CELLROWS; x++) {
        for (y = 0; y < CELLCOLS; y++) {
        
        }
    }
    */
    
    /*std::cout << "hWall: \n";
    for (xH = 0; xH < 6;xH++) {
        for (y = 0; y < 9; y++) {
            std::cout << hWall[xH][y];
        }
        std::cout << "\n";
    }

    std::cout << "vWall: \n";
    for (xV = 0; xV < 5;xV++) {
        for (y = 0; y < 10; y++) {
            std::cout << vWall[xV][y];
        }
        std::cout << "\n";
    }*/
    
    //std::cout << "target: " << targetCellPos[0] << "," << targetCellPos[1] << std::endl;
    //std::cout << "start: " << initPos[0] << "," << initPos[1] << ',' << initHead << std::endl;
    /*
    for (int i = 0; i < path.size()/2; i++) {
        std::cout << "(" << path[2*i] << "," << path[2*i+1] << ") , ";
    }
    
    std::cout << std::endl;
    */
    /*
    for (x = 0; x < CELLROWS; x++) {
        for (y = 0; y < CELLCOLS; y++) {
            std::cout << maze[x][y]->value << ",";
        }
        std::cout << std::endl;
    }*/
    /*
    for (i = 0; i < foundPaths.size(); i++) {
        std::vector<Cell*> path = foundPaths.at(i);
        for (j = 0; j < path.size(); j++)
        {
            Cell* c = path.at(j);
            std::cout << "(" << c->xPos << "," << c->yPos << ") ";
        }
        std::cout << std::endl;
    }
    */
    /*
    for (i = 0; i < allPlans.size(); i++) {
        std::vector<char> plan = allPlans.at(i);
        for (j = 0; j < plan.size(); j++) {
            char c = plan.at(j);
            std::cout << c;
        }
        std::cout << std::endl;
    }
    */
    
    return 0;
}


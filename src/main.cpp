#include <iostream>
#include <boost/algorithm/string.hpp>

#include "../include/pcd_processor.h"

void checkFirstInput(int argc, char *argv[]);
void askForToolInput();

enum State
{
    START,
    IDLE,
    WORKING,
    END
};

enum Operation
{
    VOXELGRID,
    OUTLIERSFILTER,
    MLSFILTER,
    TRIANGULATION,
    SAVEPCD,
    SAVESTL,
    FILEOPEN,
    ROUTESAVE,
    RESTART,
    END_OP,
    IDLE_OP,
    BLOCK
};

State state = START;
Operation operation = IDLE_OP;

PcdProcessor *processor;

int main(int argc, char *argv[])
{
    std::cout << "[INFO] Welcome to PCDPorcessorTool 1.0!" << std::endl;
    std::cout << "[INFO] Checking if you passed the correct parameters..." << std::endl;

    checkFirstInput(argc, argv);

    while (state != END)
    {
        askForToolInput();
    }

    return (0);
}

void checkFirstInput(int argc, char *argv[])
{
    if (argc != 3)
    {
        std::cout << "[WARN] Please:" << std::endl
                  << " - Set the file to open with [ " << FILEOPEN << " - Set pcd file to open ]" << std::endl
                  << " - Set the route where files are to be saved with [ " << ROUTESAVE << " - Set route to save files ]" << std::endl
                  << " - Restart the processor with [ " << RESTART << " - Restart Processor ]" << std::endl;

        operation = BLOCK;
        state = START;
        processor = new PcdProcessor(" ", " ");

    }
    else
    {
        processor = new PcdProcessor(argv[1], argv[2]);

        if (!processor->checkValidation())
        {
            std::cout << "[WARN] Please:" << std::endl
                      << " - Set the file to open with [ " << FILEOPEN << " - Set pcd file to open ]" << std::endl
                      << " - Set the route where files are to be saved with [ " << ROUTESAVE << " - Set route to save files ]" << std::endl
                      << " - Restart the processor with [ " << RESTART << " - Restart Processor ]" << std::endl;

            operation = BLOCK;
            state = START;
        }
        else
        {
            operation = IDLE_OP;
            state = IDLE;
            std::cout << "[INFO] The possible commands are:" << std::endl
                      << " - " << VOXELGRID << " - Execute a VoxelGrid filter" << std::endl
                      << " - " << OUTLIERSFILTER << " - Execute a Outliers filter" << std::endl
                      << " - " << MLSFILTER << " - Execute a MovingLeastSquares filter" << std::endl
                      << " - " << TRIANGULATION << " - Generate STL by triangulation" << std::endl
                      << " - " << SAVEPCD << " - Save the last PointCloud obtained" << std::endl
                      << " - " << SAVESTL << " - Save the last STL mesh obtained" << std::endl
                      << " - " << FILEOPEN << " - Set pcd file to open" << std::endl
                      << " - " << ROUTESAVE << " - Set route where the files will be saved" << std::endl
                      << " - " << RESTART << " - Restart the processor with given parameters" << std::endl
                      << " - " << END_OP << " - End the toolbox app" << std::endl;
        }
    }
}

void askForToolInput()
{
    float param_float = 0;
    int param_int = 0;
    std::string param_string = "default";
    int cin;

    std::cout << "[INFO] Please, input the command number of the desired action to perform, or type [ " << END_OP + 1 << " - Display help menu ]:" << std::endl;
    std::cin >> cin;
    operation = static_cast<Operation>(cin);

    switch (operation)
    {
    case VOXELGRID:
        if (state != IDLE)
        {
            std::cout << "[ERROR] Not available until good start" << std::endl;
            break;
        }
        state = WORKING;
        std::cout << "[INFO] A VoxelGrid Filter is goig to be performed" << std::endl
                  << "Input the size of the cubic loaf [float], or left it empty to perform with the default [0.1]" << std::endl;
        std::cin >> param_float;
        if (param_float <= 0)
        {
            processor->addVoxelGrid(0.1);
            std::cout << "[INFO] A VoxelGrid Filter has been performed with DEFAULT parameters" << std::endl;
        }
        else
        {
            processor->addVoxelGrid(param_float);
            std::cout << "[INFO] A VoxelGrid Filter has been performed with GIVEN parameters" << std::endl;
        }

        break;

    case OUTLIERSFILTER:
        if (state != IDLE)
        {
            std::cout << "[ERROR] Not available until good start" << std::endl;
            break;
        }
        state = WORKING;
        std::cout << "[INFO] A Outliers Filter is goig to be performed" << std::endl
                  << "Input the number of neighbors [int] and the standard deviations for a point to be considered an outlier [float] or left it empty to perform with the default [120, 0.01]" << std::endl;
        std::cin >> param_int;
        std::cin >> param_float;
        if (param_int <= 0 || param_float <= 0)
        {
            processor->addOutliersFilter(120, 0.01);
            std::cout << "[INFO] A Outliers Filter has been performed with DEFAULT parameters" << std::endl;
        }
        else
        {
            processor->addOutliersFilter(param_int, param_float);
            std::cout << "[INFO] A Outliers Filter has been performed with GIVEN parameters" << std::endl;
        }

        break;

    case MLSFILTER:
        if (state != IDLE)
        {
            std::cout << "[ERROR] Not available until good start" << std::endl;
            break;
        }
        state = WORKING;
        std::cout << "[INFO] A MovingLeastSquares Filter is goig to be performed" << std::endl
                  << "Input the order of the polynomial to fit [int] and the size of the neighborhood around a point [float] or left it empty to perform with the default [2, 0.3]" << std::endl;
        std::cin >> param_int;
        std::cin >> param_float;
        if (param_int <= 0 || param_float <= 0)
        {
            processor->addMlsFilter(2, 0.3);
            std::cout << "[INFO] A MovingLeastSquares Filter has been performed with DEFAULT parameters" << std::endl;
        }
        else
        {
            processor->addMlsFilter(param_int, param_float);
            std::cout << "[INFO] A MovingLeastSquares Filter has been performed with GIVEN parameters" << std::endl;
        }

        break;
    case TRIANGULATION:
        if (state != IDLE)
        {
            std::cout << "[ERROR] Not available until good start" << std::endl;
            break;
        }
        state = WORKING;
        std::cout << "[INFO] A mesh by triangulation is goig to be generated form the last PointCLoud generated!" << std::endl
                  << "Input the number of points to use for the nearest neighbor search [int] and the maximum distance between connected points [float] or left it empty to perform with the default [300, 40.0]" << std::endl;
        std::cin >> param_int;
        std::cin >> param_float;
        if (param_int <= 0 || param_float <= 0)
        {
            processor->makeTriangulation(300, 40.0, 10);
            std::cout << "[INFO] The mesh has been generated with DEFAULT parameters" << std::endl;
        }
        else
        {
            processor->makeTriangulation(param_int, param_float, 10);
            std::cout << "[INFO] The mesh has been generated with GIVEN parameters" << std::endl;
        }

        break;

    case SAVEPCD:
        if (state != IDLE)
        {
            std::cout<<"[ERROR] Not available until good start"<<std::endl;
            break;
        }
        state = WORKING;
        std::cout << "[INFO] The last PCD File generated will be saved in " << processor->getRouteToSave() << std::endl
                  << "Please enter a name: " << std::endl;
        std::cin >> param_string;

        processor->savePCD(param_string);

        break;

    case SAVESTL:
        if (state != IDLE)
        {
            std::cout<<"[ERROR] Not available until good start"<<std::endl;
            break;
        }
        state = WORKING;
        std::cout << "[INFO] The last STL File generated will be saved in " << processor->getRouteToSave() << std::endl
                  << "Please enter a name: " << std::endl;
        std::cin >> param_string;

        processor->saveSTL(param_string);

        break;

    case FILEOPEN:
        // state = WORKING;
        
        std::cout << "[INFO] Change original file from " << processor->getFileOpened() << std::endl
                  << "Please enter the route to the pcd file: " << std::endl;
        std::cin >> param_string;

        processor->setFileToOpen(param_string);

        std::cout << "[WARN] A restart will be needed! " << std::endl;

        break;

    case ROUTESAVE:
        // state = WORKING;

        std::cout << "[INFO] Change path to save files from " << processor->getFileOpened() << std::endl
                  << "Please enter the route to the pcd file: " << std::endl;
        std::cin >> param_string;

        processor->setRouteToSave(param_string);

        break;
    case RESTART:
        // state = WORKING;
        std::cout << "[INFO] All processor will be restarted. All files not saved will be restored" << processor->getFileOpened() << std::endl;

        processor->restartProcessor();

        if (!processor->checkValidation())
        {
            std::cerr << "[WARN] Bad restart: " << std::endl
                      << " - Set the file to open with [ " << FILEOPEN << " - Set pcd file to open ]" << std::endl
                      << " - Set the route where files are to be saved with [ " << ROUTESAVE << " - Set route to save files ]" << std::endl
                      << " - Restart the processor with [ " << RESTART << " - Restart Processor ]" << std::endl;

            operation = BLOCK;
            state = START;
        }
        else
        {
            operation = IDLE_OP;
            state = IDLE;
            std::cout << "[INFO] Restart is Succesful! Reminder that the possible commands now are:" << std::endl
                      << " - " << VOXELGRID << " - Execute a VoxelGrid filter" << std::endl
                      << " - " << OUTLIERSFILTER << " - Execute a Outliers filter" << std::endl
                      << " - " << MLSFILTER << " - Execute a MovingLeastSquares filter" << std::endl
                      << " - " << TRIANGULATION << " - Generate STL by triangulation" << std::endl
                      << " - " << SAVEPCD << " - Save the last PointCloud obtained" << std::endl
                      << " - " << SAVESTL << " - Save the last STL mesh obtained" << std::endl
                      << " - " << FILEOPEN << " - Set pcd file to open" << std::endl
                      << " - " << ROUTESAVE << " - Set route where the files will be saved" << std::endl
                      << " - " << RESTART << " - Restart the processor with given parameters" << std::endl
                      << " - " << END_OP << " - End the toolbox app" << std::endl;
        }

        break;

    case END_OP:
        std::cout << "[INFO] The application will end now" << std::endl;
        state = END;

        break;

    case END_OP + 1:
        std::cout << "[INFO] The possible commands are:" << std::endl
                  << " - " << VOXELGRID << " - Execute a VoxelGrid filter" << std::endl
                  << " - " << OUTLIERSFILTER << " - Execute a Outliers filter" << std::endl
                  << " - " << MLSFILTER << " - Execute a MovingLeastSquares filter" << std::endl
                  << " - " << TRIANGULATION << " - Generate STL by triangulation" << std::endl
                  << " - " << SAVEPCD << " - Save the last PointCloud obtained" << std::endl
                  << " - " << SAVESTL << " - Save the last STL mesh obtained" << std::endl
                  << " - " << FILEOPEN << " - Set pcd file to open" << std::endl
                  << " - " << ROUTESAVE << " - Set route where the files will be saved" << std::endl
                  << " - " << RESTART << " - Restart the processor with given parameters" << std::endl
                  << " - " << END_OP << " - End the toolbox app" << std::endl;
        break;

    default:
        std::cout << "[WARN] Your input is not recognized as a command. The possible commands are:" << std::endl
                  << " - " << VOXELGRID << " - Execute a VoxelGrid filter" << std::endl
                  << " - " << OUTLIERSFILTER << " - Execute a Outliers filter" << std::endl
                  << " - " << MLSFILTER << " - Execute a MovingLeastSquares filter" << std::endl
                  << " - " << TRIANGULATION << " - Generate STL by triangulation" << std::endl
                  << " - " << SAVEPCD << " - Save the last PointCloud obtained" << std::endl
                  << " - " << SAVESTL << " - Save the last STL mesh obtained" << std::endl
                  << " - " << FILEOPEN << " - Set pcd file to open" << std::endl
                  << " - " << ROUTESAVE << " - Set route where the files will be saved" << std::endl
                  << " - " << RESTART << " - Restart the processor with given parameters" << std::endl
                  << " - " << END_OP << " - End the toolbox app" << std::endl;
        break;
    }

    if (operation != END_OP && state==WORKING)
    {
        operation = IDLE_OP;
        state = IDLE;
    }
}

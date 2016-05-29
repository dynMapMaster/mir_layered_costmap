#include "pmac_learner.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"

Pmac_learner::Pmac_learner(int sizeX=2, int sizeY =2, double resolution=2)
    : grid(sizeX, sizeY, resolution), sse_score(0.0), scored_observations(0),
      initial_occupancy(2), initial_free(2)
{
    update_time = ros::Time::now().toNSec();
    ros::NodeHandle nh;
    currentErrorPub = nh.advertise<std_msgs::Float32>("current_map_error",10);
    lambda_entry_pub = nh.advertise<std_msgs::Float32>("lambda_entry",10);
    lambda_exit_pub = nh.advertise<std_msgs::Float32>("lambda_exit",10);
    scoreData_pub = nh.advertise<std_msgs::Float32MultiArray>("scoreData",10);
}

bool Pmac_learner::getCellValue(int x, int y, unsigned char& cellValueOutput)
{
    double occ_value = getOccupancyPrabability(x,y);
    // For generating a AMCL map
/*
    if(occ_value == -1)
    {
        return false;
    }
    else if(occ_value == Pmac_cell::STATIC_OCCUPIED_VALUE)
    {
        cellValueOutput = 100;
        return true;
    }
    else if(occ_value >= 0)
    {
        cellValueOutput = 0;
        return true;
    }

*/
    if(occ_value >= 0)
    {
        cellValueOutput = (OBSTACLE_THRESHOLD-1-10) * occ_value + 10;
        translateOcc(cellValueOutput);
        return true;
    }
    else if(occ_value == -1)
    {
        return false;
    }
    else if(Pmac_cell::STATIC_OCCUPIED_VALUE)
    {
        cellValueOutput = 255;
        return true;
    }
    else {
        ROS_ERROR("Unknown occupied value");
        throw -1;
    }
}

double Pmac_learner::getPredictScore()
{
    if(scored_observations > 0)
        return sse_score / ((double)scored_observations);
    else
        return 0.0;
}

bool isWithinTestLimit(int x, int y)
{   // 0 pos:
     // TL :
           //x=141, y=77            x=159   y=98 -> OBS1
    //      x=200 y = 77            x=223 y=96  -> OBS2
    //      X=158,Y=254       BR:  X=180 Y=274  -> OBS4
    //      x = 122 y = 250         x=141 y=273 -> OBS 5
    //      x = 50 y=233            x = 71 y=259 -> OBS 6
    //      x = 94 y = 147          x=110  y=170 -> OBS7

    //      x=114 y=143             x=128  y=160 -> OBS 8
    //      x=114 y=162             x=129  y=175 -> OBS 9
    //      x=164 y=148             x=174  y=169 -> OBS 10
    int xl=141, yl=77, xr=159, yr=98;
    if(x >= xl && x < xr && y >= 318-yr && y < 318-yl){
    //if(x >= 158 && x < 180 && y >= 318-274 && y < 318-254){
        return true;
    }
    return false;
}



int getObstacleNumber(int x , int y)
{
    int xl=141, yl=77, xr=159, yr=98;
    if(x >= xl && x < xr && y >= 318-yr && y < 318-yl){
        return 1; // 1
    }
    xl=200, yl=77, xr=223, yr=96;
    if(x >= xl && x < xr && y >= 318-yr && y < 318-yl){
        return 2; // 2
    }

    xl=158, yl=254, xr=180, yr=274;
    if(x >= xl && x < xr && y >= 318-yr && y < 318-yl){
        return 3; // 4
    }

    xl=122, yl=250, xr=141, yr=273;
    if(x >= xl && x < xr && y >= 318-yr && y < 318-yl){
        return 4; //5
    }

    xl=50, yl=233, xr=71, yr=259;
    if(x >= xl && x < xr && y >= 318-yr && y < 318-yl){
        return 5; //6
    }

    xl=94, yl=147, xr=110, yr=170;
    if(x >= xl && x < xr && y >= 318-yr && y < 318-yl){
        return 6; //7
    }

    xl=114, yl=143, xr=128, yr=160;
    if(x >= xl && x < xr && y >= 318-yr && y < 318-yl){
        return 7; //8
    }

    xl=114, yl=162, xr=129, yr=175;
    if(x >= xl && x < xr && y >= 318-yr && y < 318-yl){
        return 8; //9
    }

    xl=164, yl=148, xr=174, yr=169;
    if(x >= xl && x < xr && y >= 318-yr && y < 318-yl){
        return 9; //10
    }


    return -1;
}

void Pmac_learner::addObservationMap(Observation_interface* observation)
{
    //ROS_ERROR("time since update %i",(ros::Time::now().toNSec() - update_time));
    ros::Time t = ros::Time::now();
    //if( (t.toNSec()  - update_time) > UPDATE_INTERVAL)
    {

        std::vector<double> current_sse, current_obs_number, lambda_exit_sum, lambda_exit_cnt, lambda_entry_sum, lambda_entry_cnt;
        for(int i = 0; i < 10;i++)
        {
            lambda_exit_sum.push_back(0);
            lambda_exit_cnt.push_back(0);
            lambda_entry_sum.push_back(0);
            lambda_entry_cnt.push_back(0);
            current_sse.push_back(0);
            current_obs_number.push_back(0);
        }

        update_time = ros::Time::now().toNSec();
        int max_x, max_y, min_y, min_x;
        observation->loadUpdateBounds( min_x,  max_x,  min_y,  max_y);
        if(max_x >= 0 && min_y >= 0 && max_y >= 0 && min_x >= 0)
        {
            for(int y = min_y; y <= max_y ; y++){
                for(int x = min_x; x <= max_x; x++){
                    double occupancy_prob = observation->getOccupancyPrabability(x,y);
                    if (occupancy_prob >= 0){
                        double predicted_occupancy_prob = getOccupancyPrabability(x,y);
                        Pmac_cell* cell = grid.editCell(x,y);
                        int obsNo = getObstacleNumber(x,y);
                        if(obsNo >= 0)
                        {
                            if( predicted_occupancy_prob >= 0){ //(predicted_occupancy_prob > 0. || occupancy_prob > 0.5) &&
                                sse_score += std::pow(occupancy_prob - predicted_occupancy_prob,2);
                                scored_observations++;
                                current_sse[obsNo] += std::pow(occupancy_prob - predicted_occupancy_prob,2);
                                current_obs_number[obsNo]++;
                            }
                            std::vector<double> params = cell->serialize();
                            if( (params[0] > 0.5 || params[1] > 0.5) && (params[2] > 0.5 || params[3] > 0.5) )
                            {
                                lambda_exit_sum[obsNo] += cell->getLambdaExit();
                                lambda_exit_cnt[obsNo]++;
                                lambda_entry_sum[obsNo] += cell->getLambdaEntry();
                                lambda_entry_cnt[obsNo]++;
                            }

                        }

                            cell->addProbability(occupancy_prob,t.toSec());

                    }
                }
            }
        }

        //if(current_obs_number > 0 )
        {           
            /*std_msgs::Float32 msg;
            //msg.data = (float)(current_sse / current_obs_number);
            currentErrorPub.publish(msg);
            ROS_ERROR("SUM OF CELLS: %f",current_sse);
            ROS_ERROR("NUMBER OF CELLS: %f",current_obs_number);
            */
            std_msgs::Float32MultiArray dataMsg;
            dataMsg.data.clear();


            for(int i = 1; i < 10;i++)
            {
                double score = (current_obs_number[i] > 0 ? (float)(current_sse[i] / current_obs_number[i]) : -1);
                double lambda_exit_res = (lambda_exit_cnt[i] > 0 ? lambda_exit_sum[i] / lambda_exit_cnt[i] : -1 );
                double lambda_entry_res = (lambda_entry_cnt[i] > 0 ? lambda_entry_sum[i] / lambda_entry_cnt[i] : -1 );
                dataMsg.data.push_back(score);
                dataMsg.data.push_back(lambda_entry_res);
                dataMsg.data.push_back(lambda_exit_res);
            }
            scoreData_pub.publish(dataMsg);

        }
        observation->resetEditLimits();
    }
}

void Pmac_learner::loadUpdateBounds(int& xMin, int& xMax, int& yMin, int& yMax)
{
    grid.loadUpdateBounds(xMin, xMax, yMin, yMax);
}

double Pmac_learner::getOccupancyPrabability(int x, int y)
{
    Pmac_cell* cell = grid.readCell(x,y);
    if (cell != NULL)
    {
        uint64_t prev_reading_time = cell->getLastObservationTime() * 1e9;
        uint64_t time_now = ros::Time::now().toNSec();
        uint64_t time_span = time_now - prev_reading_time;
        unsigned steps = time_span / UPDATE_INTERVAL + 0.5;

        if(steps == 0)
            steps = 1;

        double occupancy_prob = 0;

        if(steps > 20 || (steps > 5 && steps > cell->getMixingTime()))
        {
            occupancy_prob = cell->getLongTermOccupancyProb();
        }
        else
        {
           occupancy_prob = cell->getProjectedOccupancyProbability(steps);
        }
        return occupancy_prob;
    }
    else
        return -1;
}

void Pmac_learner::resetEditLimits()
{
    grid.resetUpdateBounds();
}

void Pmac_learner::initCell(int x, int y, Initial_values value)
{
    switch(value)
    {
        case Free:
            grid.editCell(x,y)->init(0,initial_free);
            break;
        case Unknown:
            break;
        case Obstacle:
            grid.editCell(x,y)->init(initial_occupancy,0);
            break;
        default:
            break;
    }
}

void Pmac_learner::translateOcc(unsigned char& value)
{
    if(value == 95) // Avoid making critical regions
    {
        value = 96;
    }
}

std::vector<std::vector<double> > Pmac_learner::serialize()
{
    std::vector<std::vector<double> > result;
    result.reserve(grid.sizeX()*grid.sizeY()+1);
    // Store size of grid and resolution
    std::vector<double> size;
    size.push_back(grid.sizeX());
    size.push_back(grid.sizeY());
    size.push_back(grid.resolution());
    result.push_back(size);

    // Store grid values
    for(int y = 0; y < grid.sizeY();y++){
        for(int x = 0; x < grid.sizeX();x++){
            Pmac_cell* cell = grid.readCell(x,y);
            std::vector<double> cellVal;
            if(cell)
            {
                cellVal = cell->serialize();
            }
            else
            {
                // OBS IF SINGLE VECTOR SERIALIZATION - CONSIDER
                cellVal.push_back(0);
            }
            result.push_back(cellVal);
        }
    }
    return result;
}

void Pmac_learner::deserialize(const std::vector<std::vector<double> >& values)
{
    if(values.size() < 1 || values[0].size() < 3)
    {
        ROS_ERROR("Pmac_learner - FIRST LINE IN WRONG FORMAT!!");
        return;
    }
    // create gridstructure in correct size
    //grid = Grid_structure<Pmac_cell>(values[0][0],values[0][1],values[0][2]);

    // Make sure grid sizes match
    if(grid.sizeX() != (int)values[0][0] || grid.sizeY() != (int)values[0][1] || std::fabs(grid.resolution()-values[0][2]) > 0.01)
    {
        ROS_ERROR("LOADED MAP AND ORIGINAL DOES NOT MATCH IN SIZE OR RESOLUTION %f %f %f ",values[0][0], values[0][1],  values[0][2]);
        ROS_ERROR("CURRENT %i %i %f ",grid.sizeX(), grid.sizeY(), grid.resolution());
        return;
    }

    for(int i = 1; i < values.size();i++){
        if(values[i].size() == 5)
        {

            // calculate x-y Value
            int y = (i-1) / grid.sizeX();
            int x = (i-1) % grid.sizeX();
            Pmac_cell* cell = grid.editCell(x,y);
            if(cell)
                cell->deserialize(values[i]);
            else
                ROS_ERROR("CELL WAS NULL!");
        }
        else if(values[i].size() == 1)
        {

        }
        else
        {
            ROS_ERROR("LOADED CELL WRONG SIZE %i", values[i].size());
        }

    }
}

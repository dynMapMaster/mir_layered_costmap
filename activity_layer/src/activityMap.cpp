#include "activityMap.hpp"

#include <stddef.h>
#include <iostream>
#include <cmath>
#include <limits>

using namespace std;

activityMap::activityMap(int sizeX, int sizeY, bool useOverlay)
{
    _sizeX = sizeX;
    _sizeY = sizeY;
    _useOverlay = useOverlay;
    if(useOverlay)
    {
        ROS_INFO("Using overlay");
        int stdDivisor = 30;
        int xDivisor = stdDivisor;
        for(int i = xDivisor; i < xDivisor+50;i++){
            if(sizeX % i == 0){
                xDivisor = i;
                break;
            }
        }
        if(sizeX % xDivisor != 0){
            xDivisor = stdDivisor;
            while(sizeX % xDivisor != 0){
                xDivisor--;
            }
        }

        int yDivisor = stdDivisor;
        for(int i = yDivisor; i < yDivisor+50;i++){
            if(sizeY % i == 0){
                yDivisor = i;
                break;
            }
        }
        if(sizeY % yDivisor != 0){
            yDivisor = stdDivisor;
            while(sizeY % yDivisor != 0){
                yDivisor--;
            }
        }


        _mapPatchXSize = xDivisor;
        _mapPatchYSize = yDivisor;
        _patchesAlongX = _sizeX / _mapPatchXSize;
        _patchesAlongY = _sizeY / _mapPatchYSize;

        cout << "Patch size x: " << _mapPatchXSize << "\nPatch size y: " << _mapPatchYSize << endl;

        _mapWithOverlay = new activityMapComponent**[(_patchesAlongX)*(_patchesAlongY)];
        for(int i = 0; i < (_patchesAlongX)*(_patchesAlongY); i++){
            _mapWithOverlay[i] = NULL;
        }


    }
    else
    {
        ROS_INFO("No Overlay SIze x=%i  y=%i",sizeX, sizeY);
        _map = new activityMapComponent*[sizeX * sizeY];
        // initialize pointers to null
        for(int i = 0; i < _sizeX * _sizeY ; i++){
            _map[i] = NULL;
        }
    }
    this->resetEditLimits();
    ROS_INFO("QT CHANGE!");
}


activityMap::~activityMap()
{
    if(_useOverlay)
    {
        for(int k = 0; k < _patchesAlongX*_patchesAlongY;k++){
            if(_mapWithOverlay[k] != NULL)
            {
                for(int i = 0; i < _mapPatchXSize*_mapPatchYSize;i++){
                    if(_mapWithOverlay[k][i] != NULL)
                    {
                        delete _mapWithOverlay[k][i];
                    }
                }
                delete _mapWithOverlay[k];
            }
        }
        delete [] _mapWithOverlay;
    }
    else
    {
        for(int i = 0;i < _sizeX * _sizeY ; i++){
            delete _map[i];
        }
        delete [] _map;
    }
}



void activityMap::resetEditLimits()
{
    _editMaxX = -1;
    _editMaxY = -1;
    _editMinX = -1;
    _editMinY = -1;
}


void activityMap::pointObservedFree(int x, int y, long unsigned time)
{
    getDataPointer(x,y)->observedFree(time);
    this->updateEditLimits(x,y);
}

void activityMap::pointObservedOccupied(int x, int y, long unsigned time)
{
    getDataPointer(x,y)->observedOccupied(time);
    this->updateEditLimits(x,y);
}

void activityMap::updateEditLimits(int x, int y)
{
    if(_editMinX > x || _editMinX < 0)
        _editMinX = x;

    if(_editMaxX < x || _editMaxX < 0)
        _editMaxX = x;

    if(_editMinY > y || _editMinY < 0)
        _editMinY = y;

    if(_editMaxY < y || _editMaxY < 0)
        _editMaxY = y;
}

void activityMap::addObservationMap(activityMap& newMap, collapsingMethod method)
{
    for(int y = newMap._editMinY; y < newMap._editMaxY+1 ; y++){
        for(int x = newMap._editMinX; x < newMap._editMaxX+1; x++){
            if(newMap.pointContainsData(x,y))
            {
                this->getDataPointer(x,y)->add(newMap.getDataPointer(x,y),method);
            }
            this->updateEditLimits(x,y);
        }
    }
}

bool activityMap::getEditLimits(int &minX, int &maxX, int &minY, int &maxY)
{
    bool newLimits = false;
    ROS_INFO("EDIT LIMITS: %i %i %i %i", _editMinX, _editMaxX, _editMinY, _editMaxY);
    if(_editMinX >= 0 && _editMaxX >= 0 && _editMinY >= 0 && _editMaxY >= 0)
    {
        minX = _editMinX;
        maxX = _editMaxX;
        minY = _editMinY;
        maxY = _editMaxY;
    }

    return newLimits;
}

void activityMap::printDebugInfo()
{
    cout << "--------- DEBUG INFO -------------\nPointer info:" << endl;
    for(int y = -1; y < _sizeY ; y++){
        //cout << y << endl;
        for(int x = -1; x < _sizeX; x++){
            //cout << x << endl;
            if(y < 0 || x < 0){
                if(y < 0 && x < 0){cout << "\t";}
                else{
                    if(x >= 0)
                        cout << x << "\t";
                    if(x < 0)
                        cout << y << "\t";

                }
            }
            else
            {
                if(!pointContainsData(x,y))
                    cout << "-\t" ;
                else
                    cout << "*\t" ;
            }
        }
        cout << endl;
    }

    cout << "\n\nMap info (Free observations, Occupy observeation, Changes to free, Changes to occupied, Timestamp):" << endl;
    for(int y = 0; y < _sizeY ; y++){
        for(int x = 0; x < _sizeX; x++){
            if(!pointContainsData(x,y))
                cout << "-\t\t" ;
            else
            {
                getDataPointer(x,y)->printValues();
                cout << "\t";
            }
        }
        cout << endl;
    }

    cout <<"\n\nLimits:\nX: " << _editMinX << " -> " << _editMaxX << "\nY: " << _editMinY << " -> " << _editMaxY << endl;
}

int activityMap::getXSize()
{
    return _sizeX;
}

int activityMap::getYSize()
{
    return _sizeY;
}

bool activityMap::getCellValue(int x, int y, costmapGenerationMethods method, unsigned int& cellValueOutput)
{
    cellValueOutput = 0;
    return pointContainsData(x,y);
}


void activityMap::traceLine(int x0, int y0, int x1, int y1)
{
   bresenham2D(x0,y0,x1,y1);
   pointObservedOccupied(x1,y1,0);
}

inline void activityMap::bresenham2D(int x1, int y1, const int x2, const int y2)
{
    int delta_x(x2 - x1);
    // if x1 == x2, then it does not matter what we set here
    signed char const ix((delta_x > 0) - (delta_x < 0));
    delta_x = std::abs(delta_x) *2;

    int delta_y(y2 - y1);
    // if y1 == y2, then it does not matter what we set here
    signed char const iy((delta_y > 0) - (delta_y < 0));
    delta_y = std::abs(delta_y) * 2;

    if (delta_x >= delta_y)
    {
        // error may go below zero
        int error(delta_y - (delta_x >> 1));

        while (x1 != x2)
        {
            //ROS_INFO("Clearing point (%i,%i)",x1,y1);
            pointObservedFree(x1,y1,0);
            if ((error >= 0) && (error || (ix > 0)))
            {
                error -= delta_x;
                y1 += iy;
            }
            // else do nothing

            error += delta_y;
            x1 += ix;


        }
    }
    else
    {
        // error may go below zero
        int error(delta_x - (delta_y >> 1));

        while (y1 != y2)
        {
            //ROS_INFO("Clearing point (%i,%i)",x1,y1);
            pointObservedFree(x1,y1,0);
            if ((error >= 0) && (error || (iy > 0)))
            {
                error -= delta_y;
                x1 += ix;
            }
            // else do nothing

            error += delta_x;
            y1 += iy;


        }
    }
}



// internal data fidling
bool activityMap::pointContainsData(int x, int y)
{
    if(_useOverlay)
    {
        int patchPointerY = y/_mapPatchYSize;
        int patchPointerX = x/_mapPatchXSize;

        if(_mapWithOverlay[patchPointerY * _patchesAlongX + patchPointerX] != NULL)
        {
            int internalY = y % _mapPatchYSize;
            int internalX = x % _mapPatchXSize;
            int internalYO = (y /_mapPatchYSize);
            int internalXO = (x / _mapPatchXSize);
            //cout << x << "  " << y << "\t" << internalX << "  " << internalXO << "\t" << internalY << "  " << internalYO << endl;


            if(_mapWithOverlay[patchPointerY * _patchesAlongX + patchPointerX][internalY * _mapPatchXSize + internalX] != NULL)
                return true;
        }
    }
    else
    {
        //ROS_INFO("Checking non overlay: %i", _map[y*_sizeX+x]);
        if(_map[y*_sizeX+x] != NULL)
            return true;
    }
    return false;
}



activityMapComponent* activityMap::getDataPointer(int x, int y)
{
    if(x > _sizeX || y > _sizeY )
        throw "Attempt to access non-existting map component in activity map";

    if(_sizeX <= 0 || _sizeY <= 0)
    {
        throw "ERROR: the size of the activity map is: ";
    }

    this->checkValidityOfCell(x,y);
    if(_useOverlay)
    {
        int patchPointerY = y/_mapPatchYSize;
        int patchPointerX = x/_mapPatchXSize;
        int internalY = y % _mapPatchYSize;
        int internalX = x % _mapPatchXSize;
        return _mapWithOverlay[patchPointerY * _patchesAlongX + patchPointerX][internalY * _mapPatchXSize + internalX];
    }
    else
    {
        //ROS_INFO("returning non overlay pointer");
        return _map[y * _sizeX + x];
    }

    return NULL;
}


void activityMap::checkValidityOfCell(int x, int y)
{
    //ROS_DEBUG("Checking validity");
    if(!pointContainsData(x,y))
    {
       // ROS_DEBUG"Need to create cell");
        if(_useOverlay)
        {
            int patchPointerY = y/_mapPatchYSize;
            int patchPointerX = x/_mapPatchXSize;

            if(_mapWithOverlay[patchPointerY * _patchesAlongX + patchPointerX] == NULL)
            {
                _mapWithOverlay[patchPointerY * _patchesAlongX + patchPointerX] = new activityMapComponent*[_mapPatchXSize * _mapPatchYSize];
                for(int i = 0;i < _mapPatchXSize*_mapPatchYSize ;i++){
                    _mapWithOverlay[patchPointerY * _patchesAlongX + patchPointerX][i] = NULL;
                }
            }

            int internalY = y % _mapPatchYSize;
            int internalX = x % _mapPatchXSize;

            _mapWithOverlay[patchPointerY * _patchesAlongX + patchPointerX][internalY * _mapPatchXSize + internalX] = new activityMapComponent();

        }
        else
        {
            //ROS_DEBUG("Creating non overlay");
            _map[y * _sizeX + x] = new activityMapComponent();
        }
    }
}



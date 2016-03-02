#ifndef GRID_STRUCTURE_H
#define GRID_STRUCTURE_H

#include <stddef.h>
#include "ros/ros.h"
#include <iostream>

template<class T> class Grid_structure
{
public:
    // Defines number of cells and resolution in m/cell
    Grid_structure(int sizeX, int sizeY, double resolution, double origin_x=0, double origin_y=0);

    ~Grid_structure();

    // Clear all updateBounds to -1
    void resetUpdateBounds();

    // Removes a cell completely from the structure
    void deleteCell(int x, int y);

    // Loads the updateBounds into parameters
    void loadUpdateBounds(int& xMin, int& xMax, int& yMin, int& yMax);

    // Used for reading a cell -> no changes and update bounds not invoked
    // If cell does not exists it return NULL
    // Throws exception if x or y is out of bounds
    T *readCell(int x, int y) const;

    // Used for editing a cell -> edit bounds are updated
    // If cell does not exists it is created
    // Throws exception if x or y is out of bounds
    T* editCell(int x, int y);

    //  Returns the size of the grid in x direction
    int sizeX(){return _sizeX;}

    //  Returns the size of the grid in y direction
    int sizeY(){return _sizeY;}

    // Returns the resolution i units pr cell
    double resolution(){return _resolution;}

    // mx and my is grid index for wx and wy after call
    bool worldToMap(double wx, double wy, int& mx, int& my) const;
    // mx and my is grid index for wx and wy where mx is constrained to be within the map after call
    void worldToMapEnforceBounds(double wx, double wy, int& mx, int& my) const;
    // wx and wy is world coordinates for mx and my after call
    void mapToWorld(int mx, int my, double& wx, double& wy) const;
private:
    T** _map;
    int _sizeX, _sizeY;
    int _editMinX, _editMaxX, _editMinY, _editMaxY; // to be sat to -1 for reset
    double _resolution;
    double _origin_x, _origin_y;

    void updateEditLimits(int x, int y);

};

template<class T>
Grid_structure<T>::Grid_structure(int sizeX=2,int sizeY =2, double resolution = 0.05, double origin_x, double origin_y)
{
    _sizeX = sizeX;
    _sizeY = sizeY;
    _resolution = resolution;
    _origin_x = origin_x;
    _origin_y = origin_y;
    _map = new T*[sizeX * sizeY];

    // initialize pointers to null
    for(int i = 0; i < _sizeX * _sizeY ; i++){
        _map[i] = NULL;
    }

    resetUpdateBounds();
}


template<class T>
Grid_structure<T>::~Grid_structure()
{
    for(int i = 0;i < _sizeX * _sizeY ; i++){
                delete _map[i];
    }
    delete [] _map;
}

template<class T>
void Grid_structure<T>::mapToWorld(int mx, int my, double& wx, double& wy) const
{
    wx = _origin_x + (mx + 0.5) * _resolution;
    wy = _origin_y + (my + 0.5) * _resolution;
}

template<class T>
void Grid_structure<T>::resetUpdateBounds()
{
    _editMaxX = -1;
    _editMaxY = -1;
    _editMinX = -1;
    _editMinY = -1;
}

template<class T>
bool Grid_structure<T>::worldToMap(double wx, double wy, int &mx, int &my) const
{
    if (wx < _origin_x || wy < _origin_y)
      return false;

    mx = (int)((wx - _origin_x) / _resolution);
    my = (int)((wy - _origin_y) / _resolution);

    if (mx < _sizeX && my < _sizeY)
      return true;

    return false;
}

template<class T>
void Grid_structure<T>::worldToMapEnforceBounds(double wx, double wy, int& mx, int& my) const
{
    // Here we avoid doing any math to wx,wy before comparing them to
    // the bounds, so their values can go out to the max and min values
    // of double floating point.
    if (wx < _origin_x)
    {
      mx = 0;
    }
    else if (wx > _resolution * _sizeY + _origin_x)
    {
      mx = _sizeY - 1;
    }
    else
    {
      mx = (int)((wx - _origin_x) / _resolution);
    }

    if (wy < _origin_y)
    {
      my = 0;
    }
    else if (wy > _resolution * _sizeY + _origin_y)
    {
      my = _sizeY - 1;
    }
    else
    {
      my = (int)((wy - _origin_y) / _resolution);
    }
}

template<class T>
T* Grid_structure<T>::readCell(int x, int y) const
{
    if(x > _sizeX || y > _sizeY || x < 0  || y < 0)
    {
        ROS_ERROR("Attempt to access non-existting map component in grid structure of size %i x %i:   x=%i  y=%i",_sizeX, _sizeY, x,y);
        throw "Attempt to access non-existting map component in grid structure";
    }
    //ROS_INFO("returning non overlay pointer");
    return _map[y * _sizeX + x];
}

template<class T>
T* Grid_structure<T>::editCell(int x, int y)
{

    if(x > _sizeX || y > _sizeY || x < 0  || y < 0)
    {
        ROS_ERROR("Attempt to access non-existting map component in grid structure of size %i x %i:   x=%i  y=%i",_sizeX, _sizeY, x,y);
        throw "Attempt to access non-existting map component in grid structure";
    }

    if(_map[y * _sizeX + x] == NULL)
    {
        try
        {
             _map[y * _sizeX + x] = new T();
        }
        catch(std::bad_alloc& ba)
        {
            std::cout << "New operator failed: " << ba.what() << std::endl;
        }
    }

    updateEditLimits(x,y);

    if(_map[y * _sizeX + x] == NULL)
    {
        std::cout << "Edit Cell - was null AFTER creating" << std::endl;
        throw "NULL ALERT";
    }


    return  _map[y * _sizeX + x];
}

template<class T>
void Grid_structure<T>::updateEditLimits(int x, int y)
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

template<class T>
void Grid_structure<T>::deleteCell(int x, int y)
{
    if(x > _sizeX || y > _sizeY || x < 0  || y < 0)
    {
       // ROS_ERROR("Attempt to access non-existting map component in activity map x=%i  y=%i", x,y);
        throw "Attempt to access non-existting map component in activity map";
    }

    delete _map[y * _sizeX + x];
    _map[y * _sizeX + x] = NULL;
}

template<class T>
void Grid_structure<T>::loadUpdateBounds(int& xMin, int& xMax, int& yMin, int& yMax)
{
    xMin = _editMinX;
    xMax = _editMaxX;
    yMin = _editMinY;
    yMax = _editMaxY;
}


#endif // GRID_STRUCTURE_H

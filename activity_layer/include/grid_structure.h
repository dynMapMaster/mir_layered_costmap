#ifndef GRID_STRUCTURE_H
#define GRID_STRUCTURE_H

#include <stddef.h>
#include "ros/ros.h"
#include <iostream>

template<class T> class Grid_structure
{
public:
    // Defines number of cells and resolution in m/cell
    Grid_structure(int sizeX, int sizeY, double resolution);

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

private:
    T** _map;
    int _sizeX, _sizeY;
    int _editMinX, _editMaxX, _editMinY, _editMaxY; // to be sat to -1 for reset
    double _resolution;

    void updateEditLimits(int x, int y);

};


template<class T>
Grid_structure<T>::Grid_structure(int sizeX=2,int sizeY =2, double resolution = 0.05)
{
    _sizeX = sizeX;
    _sizeY = sizeY;
    _resolution = resolution;

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
void Grid_structure<T>::resetUpdateBounds()
{
    _editMaxX = -1;
    _editMaxY = -1;
    _editMinX = -1;
    _editMinY = -1;
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

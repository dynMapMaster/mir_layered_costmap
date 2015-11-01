#ifndef _ACTIVITYMAP_HPP
#define _ACTIVITYMAP_HPP

#include "activityMapComponent.hpp"
#include <ros/ros.h>

#include <vector>

enum costmapGenerationMethods {lastKnownObservation, mostFrequentObservation};



class activityMap
{
public:
    activityMap(int sizeX,int sizeY, bool useOverlay );
    ~activityMap();
    void resetEditLimits();
    void addObservationMap(activityMap& newMap, collapsingMethod method = rawCopy);
    void pointObservedFree(int x, int y, long unsigned time);
    void pointObservedOccupied(int x, int y, long unsigned time);
    void printDebugInfo();
    int getXSize();
    int getYSize();
    bool getCellValue(int x, int y, costmapGenerationMethods method, unsigned int &cellValueOutput);
    bool getEditLimits(unsigned int &minX, unsigned int &maxX, unsigned int &minY, unsigned int &maxY);
    void traceLine(int x0, int y0, int x1, int y1);
    void useForgetting(double forgettingFactor, double maxValue);

protected:
    // Wrap map access
    bool pointContainsData(int x, int y);
    activityMapComponent* getDataPointer(int x, int y);
    void convertToEndOrigin(int& x, int& y);

    void checkValidityOfCell(int x, int y);
    void updateEditLimits(int x, int y);
    inline void bresenham2D(int x1,int y1,int const x2, int const y2);

    int _sizeX, _sizeY;
    activityMapComponent** _map;
    int _editMinX, _editMaxX, _editMinY, _editMaxY; // to be sat to -1 for reset


    // overlay
    bool _useOverlay;
    activityMapComponent*** _mapWithOverlay;
    int _mapPatchXSize, _mapPatchYSize;
    int _patchesAlongX, _patchesAlongY;

    // Settings for forgetting
    double _forgetFactor, _maxValue;
    bool _useForgetting;
};

#endif

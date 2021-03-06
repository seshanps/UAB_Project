/*
 * Simulator.h
 *
 *  Created on: Sep 1, 2014
 *      Author: alejandro
 */

#ifndef SIMULATOR_H_
#define SIMULATOR_H_

#include <stdio.h>
#include <cmath>

#include "opencv2/opencv.hpp"
//#include "cv.h"

#include "ObstacleScenario.h"
#include "PathPlaner.h"

using namespace std;
using namespace cv;


class Simulator {
	static const Scalar COLOR_EMPTY;
    static const Scalar COLOR_OCCUPIED;
	static const Scalar COLOR_POSITION;
	static const Scalar COLOR_TARGET;
    static const Scalar COLOR_SHADOW;
	static const Scalar COLOR_GRID;
    static const Scalar COLOR_SEPARATOR_LINE;
    
    string dataDirectory;
    
	int XSquares;
	int YSquares;	//in square count
	Size windowSize;
	void markSquare(int markType, Point2i square, bool isShadow);
    void clearColumn(int column);
    void drawShadow(Point2i square);
    void drawGrid();

public:
    float fov;  //Used only for avoidance simulator
    float depth;//Used only for avoidance simulator
    ObstacleScenario scenario;
    float squareRealSize;
    int squarePixelSize;
    Mat display;
    
	Simulator(float width, float height, float squareSize, int windowWidth, string _dataDirectory = "");	//Use real world measures
    void runSimulation(bool autoEraseColumns = false);
    void navigationSimulator();
    Mat drawScenario(vector<Point2f> points = {}, float fov = -1);  // Returns image with scenario drawn
};

#endif /* SIMULATOR_H_ */

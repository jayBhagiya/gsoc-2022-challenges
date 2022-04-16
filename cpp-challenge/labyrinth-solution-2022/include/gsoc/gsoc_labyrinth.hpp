/**
	Jderobot CPP Challenge: Forward declaration of labyrinth library
	@file gsoc_labyrinth.hpp
	@author Jay Bhagiya
*/

#ifndef _GSOC_LABYRINTH_HPP_
#define _GSOC_LABYRINTH_HPP_

#include <string>
#include <vector>

using std::string;
using std::vector;

class Labyrinth
{
	public:
		Labyrinth();

		// Data structure for storing location of hole point
		struct point{
			int x, y;
		};

		// Data structure for storing found path information
		struct path{
			point start, end;
			int len;
			int **connection;
		};

		// Number of rows and cols
		int row;
		int col;
		int length;

		vector<string> readInputFile(string inFilePath);  // Reads the input file data
		int** convertInputData(vector<string> input_data); //Converts the maze holes into (0) and Walls into (1)
		vector<point> getSourcePoints(int **input_data); // Finds holes in first row (Source points)
		vector<point> getDestinationPoints(int **input_data); // Finds holes in Last row (Destination points)
		bool checkPointsAvailable(vector<point> srcPoints, vector<point> destPoints); // Checks if source or destination points are in maze or not.
		void printMaze(vector<string> maze); // Prints the maze on console.
		void copyResult(int **visitor, int **output_data); // Copies the visitor data to output_data
		void getPath(int **input_data, int **visitor, int **output_data, int x1, int y1, int x2, int y2, int counter); // Gets the paths between two points
		path getLargestPath(int **input_data, point src, point dest); // Gets the longes path between two points
		vector<string> convertOutputData(vector<string> input_data, int **output_data); // Converts the mapped maze into string maze data
		bool writeOutputData(vector<string> output_data, string outFilePath); // Wirtes found maze path to the output file.
};

#endif
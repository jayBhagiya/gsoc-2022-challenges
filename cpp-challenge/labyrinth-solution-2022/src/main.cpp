/**
	Jderobot CPP Challenge: Main Source file, call to Labyrinth class.
	@file main.cpp
	@author Jay Bhagiya
*/
#include <iostream>
#include "labyrinth.h"

int main() {

	Labyrinth lb = Labyrinth();

	vector<string> input_data = lb.readInputFile("../input.txt");

	std::cout << "=====> Input Maze <=====" << std::endl;
	lb.printMaze(input_data);

	int **mapped_input = lb.convertInputData(input_data);
	
	vector<Labyrinth::point> srcPoints = lb.getSourcePoints(mapped_input);
	vector<Labyrinth::point> destPoints = lb.getDestinationPoints(mapped_input);
	
	bool isCheck = lb.checkPointsAvailable(srcPoints, destPoints);
	if(!isCheck){
		std::cout << "=====> Input Maze <=====" << std::endl;
		std::cout << -1 << std::endl;
		lb.printMaze(input_data);
		bool temp = lb.writeOutputData(input_data, "../output.txt");
		return -1;
	}
	else {
		int maxlen = 0;
		int index = -1;
		vector<Labyrinth::path> paths;

		for(int i = 0; i < srcPoints.size(); i++) {
			for(int j = 0; j < destPoints.size(); j++) {
				Labyrinth::path pathData = lb.getLargestPath(mapped_input, srcPoints[i], destPoints[j]);
				if(pathData.len > maxlen) {
					paths.push_back(pathData);
					maxlen = pathData.len;
					index += 1;
				}
			}
		}

		Labyrinth::path longestPath = paths[index];
		vector<string> output_data = lb.convertOutputData(input_data, longestPath.connection);
		std::cout << "=====> Output Maze <=====" << std::endl;
		std::cout << longestPath.len + 1 << std::endl;
		lb.printMaze(output_data);
		bool temp = lb.writeOutputData(output_data, "../output.txt");
		return 0;
	}
}
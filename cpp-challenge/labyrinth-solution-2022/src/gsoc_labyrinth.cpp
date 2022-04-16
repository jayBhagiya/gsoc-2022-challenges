/**
	Jderobot CPP Challenge: Defination of classes and functions of labyrinth library
	@file gsoc_labyrinth.cpp
	@author Jay Bhagiya
*/
#include <iostream>
#include <fstream>
#include <cstdlib>
#include "labyrinth.h"

/**
 * Implementation of a labyrinth for solving maze problem.
 *
 * In this problem, maze has holes(1) and walls(0). With the 
 * help of DFS(Depth First Search) algorithm finding longest 
 * possible path in maze.
 *
 */
Labyrinth::Labyrinth() {
	this->row = 0;
	this->col = 0;
	this->length = -1;
}

/**
 * Reads the data from given input file path.
 *
 * @param: Input file path as string
 * @return: Stored maze as vector<string>
 * 
 */
vector<string> Labyrinth::readInputFile(string inFilePath) {
	std::fstream infile;
	infile.open(inFilePath, std::ios::in);

	if(!infile.is_open()) {
		std::cout << "Error while opening input file." << std::endl;
		std::exit(-1);
	}

	int row = 0, col = 0;
	vector<string> input_data;

	string tp;
	while(getline(infile, tp)){
		col = tp.length();
		input_data.push_back(tp);
		row += 1;
	}

	this->row = row;
	this->col = col;

	return input_data;
}

/**
 * Converts the string input maze to integer input maze [holes-->1, walls-->0]
 *
 * @param: Input Maze as vector<string>
 * @return Input Maze as 2D-Integer Array
 * 
 */
int** Labyrinth::convertInputData(vector<string> input_data) {
	int **mapped_input;

	mapped_input = new int *[this->row];
	for(int i = 0; i < this->row; i++) {
		mapped_input[i] = new int [this->col];
	}

	for(int i = 0; i < this->row; i++) {
		for(int j = 0; j < this->col; j++) {
			if(input_data[i].compare(j, 1, "#") == 0) {
				mapped_input[i][j] = 1;
			}
			else {
				mapped_input[i][j] = 0;
			}
		}
	}
	return mapped_input;
}

/**
 * Gets the holes from first row as srource points
 *
 * @param: Input maze as 2D-Integer Array
 * @return Found Points as vector<point>
 * 
 */
vector<Labyrinth::point> Labyrinth::getSourcePoints(int **input_data) {
	vector<Labyrinth::point> temp_pts;
	for(int i = 0; i < this->col; i++) {
		Labyrinth::point temp;
		if(input_data[0][i] == 0){
			temp.x = 0;
			temp.y = i;
			temp_pts.push_back(temp);
		}
	}
	return temp_pts;
}


/**
 * Gets the holes from last row as destination points
 *
 * @param: Input maze as 2D-Integer Array
 * @return Found Points as vector<point>
 * 
 */
vector<Labyrinth::point> Labyrinth::getDestinationPoints(int **input_data) {
	vector<Labyrinth::point> temp_pts;
	for(int i = 0; i < this->col; i++) {
		Labyrinth::point temp;
		if(input_data[this->row - 1][i] == 0){
			temp.x = this->row - 1;
			temp.y = i;
			temp_pts.push_back(temp);
		}
	}
	return temp_pts;
}

/**
 * Checks if Source or Destination holes (Points) are available
 *
 * @param: Source & Destination points as vector<point>
 * @return: Boolean 
 * 
 */
bool Labyrinth::checkPointsAvailable(vector<Labyrinth::point> srcPoints, vector<Labyrinth::point> destPoints) {
	if(srcPoints.size() < 1) {
		// std::cout << "No Source Holes are available." << std::endl;
		return false;
	}
	if(destPoints.size() < 1) {
		// std::cout << "No Destination Holes are available." << std::endl;
		return false;
	}
	return true;
}

/**
 * Prints the maze to console
 *
 * @param Maze input as vector<string>
 * @return: None
 * 
 */
void Labyrinth::printMaze(vector<string> maze) {
	for(int i = 0; i < this->row; i++) {
		std::cout << maze[i] << std::endl;
	}
	std::cout << std::endl;
}

/**
 * Copies the data from first param to second param
 *
 * @param: Visitor & Output Data as 2D-Integer Array
 * @return: None
 * 
 */
void Labyrinth::copyResult(int **visitor, int **output_data) {
	for(int i = 0; i < this->row; i++) { 
		for(int j = 0; j < this->col; j++) {
			output_data[i][j] = visitor[i][j];
		}
	}
}

/**
 * Gets the path between given params (x1,y1)-->(x2,y2) using DFS algorithm.
 *
 * @param: Input Data, Visitor, Output Data as 2D-Integer Array & Source m Destination as Point x1,y1,x2,y2 & Counter as interger
 * @return: None
 * 
 */
void Labyrinth::getPath(int **input_data, int **visitor, int **output_data, int x1, int y1, int x2, int y2, int counter) {
	if(x1 < 0 or x1 >= this->row or y1 < 0 or y1 >= this->col){
		return;
	}
	else if(x1 == x2 and y1 == y2) {
		if(visitor[x1][y1] == -1 and this->length < counter) {
			this->length = counter + 1;
			this->copyResult(visitor, output_data);
			output_data[x1][y1] = counter + 1;
		}
	}
	if(visitor[x1][y1] != -1) {
		return;
	}
	if(input_data[x1][y1] == 0) {
		visitor[x1][y1] = counter + 1;
		//Test eight possible direction
		this->getPath(input_data, visitor, output_data, x1, y1 - 1, x2, y2, counter + 1);		// Left
		this->getPath(input_data, visitor, output_data, x1 - 1, y1, x2, y2, counter + 1);		// Up 
		this->getPath(input_data, visitor, output_data, x1, y1 + 1, x2, y2, counter + 1);		// Right
		this->getPath(input_data, visitor, output_data, x1 + 1, y1, x2, y2, counter + 1);		// Down
		this->getPath(input_data, visitor, output_data, x1 - 1, y1 - 1, x2, y2, counter + 1);	// Up Left
		this->getPath(input_data, visitor, output_data, x1 - 1, y1 + 1, x2, y2, counter + 1);	// Up Right
		this->getPath(input_data, visitor, output_data, x1 + 1, y1 + 1, x2, y2, counter + 1);	// Down Right
		this->getPath(input_data, visitor, output_data, x1 + 1, y1 - 1, x2, y2, counter + 1);	// Down Left
		// Deactivate visited node status
		visitor[x1][y1] = -1;
	}
}

/**
 * Gets the longest path between given two points
 *
 * @param: Input Data as 2D-Interger Array & Source, Destination as Point
 * @return Path Information as strct [Point]path.start, [Point]path.end, [Integer]path.len, [2D-Interger Array]path.connection
 * 
 */
Labyrinth::path Labyrinth::getLargestPath(int **input_data, Labyrinth::point src, Labyrinth::point dest) {
	int x1{ src.x }, y1{ src.y }, x2{ dest.x }, y2{ dest.y };
	Labyrinth::path pathData;

	if(x1 < 0 or x1 >= this->row or y1 < 0 or y1 >= this->col){
		std::cout << "Source Hole invalid." << std::endl;
		std::exit(-1);
	}
	if(x2 < 0 or x2 >= this->row or y2 < 0 or y2 >= this->col){
		std::cout << "Destination Hole invalid." << std::endl;
		std::exit(-1);
	}

	if(input_data[x1][y1] == 1 or input_data[x2][y2] == 1) {
		std::cout << "Source or Destination hole not available." << std::endl;
		pathData.start = src;
		pathData.end = dest;
		pathData.len = -1;
		return pathData;
	}

	int **visitor;

	this->length = -1;

	visitor = new int *[this->row];
	pathData.connection = new int *[this->row];
	for(int i = 0; i < this->row; i++) {
		visitor[i] = new int [this->col];
		pathData.connection[i] = new int [this->col];
	}

	for(int i = 0; i < this->row; i++) {
		for(int j = 0; j < this->col; j++) {
			visitor[i][j] = -1;
			pathData.connection[i][j] = 0;
		}
	}

	this->getPath(input_data, visitor, pathData.connection, x1, y1, x2, y2, -1);

	if(this->length != -1) {
		pathData.start = src;
		pathData.end = dest;
		pathData.len = this->length;
		return pathData;
	}
	else {
		// std::cout << "No path found." << std::endl;
		pathData.start = src;
		pathData.end = dest;
		pathData.len = -1;
		return pathData;
	}
}

/**
 * Coverts the mapped output maze path to string of vector
 *
 * @param: Input maze data as vector<string> & Output maze path as 2D-Integer Array
 * @return: Ouput maze path as vector<string>
 * 
 */
vector<string> Labyrinth::convertOutputData(vector<string> input_data, int **output_data) {
	vector<string> output_str = input_data;
	for(int i = 0; i < this->row; i++) {
		for(int j = 0; j < this->col; j++) {
			if(output_data[i][j] >= 0) {
				string str_num = std::to_string(output_data[i][j]);
				output_str[i].replace(j, 1, str_num);
			}
		}
	}
	return output_str;
}

/**
 * Writes the found path to Given output file path
 *
 * @param: Output maze path as vector<string> & Output file path as string
 * @return Boolean 
 * 
 */
bool Labyrinth::writeOutputData(vector<string> output_data, string outFilePath) {
	std::fstream outfile;
	outfile.open(outFilePath, std::ios::out);

	if(!outfile.is_open()) {
		std::cout << "Error while opening input file." << std::endl;
		return false;
	}
	else {
		for(int i = 0; i < this->row; i++) {
			outfile << output_data[i];
			outfile << std::endl;
		}
		outfile.close();
		return true;
	}
}
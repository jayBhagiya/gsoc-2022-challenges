/**
    Jderobot CPP Challenge: Forward declaration of labyrinth library
    @file gsoc_labyrinth.hpp
    @author Jay Bhagiya
*/

#ifndef GOSC_LABYRINTH_HPP_
#define GOSC_LABYRINTH_HPP_

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

using std::string;
using std::vector;

class Labyrinth
{
	public:
		Labyrinth(string input_file_path, string output_file_path);

		// Input and Output file path
		string input_file_path;
		string output_file_path;

		// Data structure for storing found paths
		struct point_map{
			int point_n, line_n;
			int **connection;
			vector<int> max_path;
		};

		// Data structure for storing location of hole point
		struct point{
			int x, y;
		};

		// Number of rows and cols
		int row;
		int col;

		// For storign input and output maze
		vector<string> input_data;
		vector<string> output_data;

		// For storig Mapped Input Maze
		int **mapped_input;

		// For storing founded paths
		point_map *pointed_map;

		// For storing founded path points
		vector<vector<point>> path_points;

		// For storing Largest parh points
		vector<point> largest_path;

		void read_input_file(); // Reads the input file
		void map_input_data();  // Converts the maze holes into (0) and Walls into (1)
		void find_path_points(); // Finds all the holes(0) in maze
		void find_adjacent(int x, int y, vector<point> &point); // Finds the adjacent holes for particular hole
		bool is_adjacent(point pt1, point pt2); // Checkes if the two holes are adjacent
		void build_path(int seq); // Build the path for given sequence of holes.
		void dfs(int seq, int x, vector<int> visited, vector<int> &path_start); // Depth First Search Algorithm
		void find_largest_path(); // Finds the largest path of holes
		void convert_output_data(); // Converts the largest found path holes to numbered path
		void print_data(); // Prints the input and output data
		void write_output_file(); // Writes the found path to output file
};

#endif
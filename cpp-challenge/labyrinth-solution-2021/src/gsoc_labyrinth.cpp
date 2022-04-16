/**
    Jderobot CPP Challenge: Defination of classes and functions of labyrinth library
    @file gsoc_labyrinth.cpp
    @author Jay Bhagiya
*/

#include "labyrinth.h"

/**
 * Implementation of a labyrinth for solving maze problem.
 *
 * In this problem, maze has holes(1) and walls(0). With the 
 * help of DFS(Depth First Search) algorithm finding longest 
 * possible path in maze.
 *
 */
Labyrinth::Labyrinth(string input_file_path, string output_file_path) {
	this->input_file_path = input_file_path;
	this->output_file_path = output_file_path;

	this->row = 0;
	this->col = 0;
	this->input_data;
	this->output_data;
	this->path_points;
	this->pointed_map;
	this->largest_path;

	read_input_file();

	this->mapped_input = new int *[this->row];
	for(int i = 0; i < this->row; i++) {
		this->mapped_input[i] = new int [this->col];
	}
}

/**
 * Reads the given input file.
 *
 * @variable Stores data in input_data class variable.
 * 
 */
void Labyrinth::read_input_file() {
	std::fstream infile;
	infile.open(this->input_file_path, std::ios::in);

	if(!infile.is_open()) {
		std::cout << "Error while opening input file." << std::endl;
		return;
	}

	int row = 0, col = 0;

	string tp;
	while(getline(infile, tp)){
		col = tp.length();
		this->input_data.push_back(tp);
		row += 1;
	}

	this->row = row;
	this->col = col;
}

/**
 * Maps the input data into O if walls and 1 if holes.
 *
 * @varibale Stores data in mapped_input class variable
 */
void Labyrinth::map_input_data() {
	// std::cout << "Mapping maze to 0 and 1." << std::endl;
	for(int i = 0; i < this->row; i++) {
		for(int j = 0; j < this->col; j++) {
			if(this->input_data[i].compare(j, 1, "#") == 0) {
				this->mapped_input[i][j] = 0;
			}
			else {
				this->mapped_input[i][j] = 1;
			}
			// std::cout << this->mapped_input[i][j];
		}
		// std::cout << std::endl;
	}
}

/**
 * Finds the adjacent holes of every holes in maze.
 *
 * @varibale Stores all the possible path points(holes) in path_points list variable.
 */
void Labyrinth::find_path_points() {
	// std::cout << "Getting all path points (Points with 1 value.)" << std::endl;

	for(int i = 0; i < this->row; i++) {
		for(int j = 0; j < this->col; j++) {
			if(this->mapped_input[i][j] == 1) {
				vector<point> temp_points;
				find_adjacent(i, j, temp_points);

				// for(int i=0; i<temp_points.size(); ++i)
				// 	std::cout << "("<< temp_points[i].x << "," << temp_points[i].y << ") ";
				// std::cout << std::endl;
				this->path_points.push_back(temp_points);
			}
		}
	}
	int num = this->path_points.size();
	this->pointed_map = new point_map[num];
}

/**
 * Finds the adjacent holes for given point(hole).
 *
 * @params x, y: location of point & points: list of points
 * @return None
 */
void Labyrinth::find_adjacent(int x, int y, vector<point> &points) {
	if (this->mapped_input[x][y] != 1){
		return;
	}
	this->mapped_input[x][y] = 0;
	point add;
	add.x = x;
	add.y = y;
	points.push_back(add);
	if ( x < this->row - 1){
		find_adjacent(x + 1, y, points);
	}
	if (y < this->col - 1){
		find_adjacent(x, y + 1, points);
	}
	if ( x > 0){
		find_adjacent(x - 1, y, points);
	}
	if ( y > 0){
		find_adjacent(x, y - 1, points);
	}
}


/**
 * Checks if two points are adjacents or not
 *
 * @params point1 and point2
 * @return boolean
 */
bool Labyrinth::is_adjacent(point pt1, point pt2){
	int diff = std::abs(pt1.x - pt2.x) + std::abs(pt1.y - pt2.y);
	if(diff == 1) {
		return true;
	}
	else {
		return false;
	}
}

/**
 * Builds the path given sequences holes
 *
 * @params sequence number
 * @return none
 */
void Labyrinth::build_path(int seq) {
	vector<point> temp_points(this->path_points[seq]);
	this->pointed_map[seq].point_n = temp_points.size();
	this->pointed_map[seq].line_n = 0;

	this->pointed_map[seq].connection = new int *[this->pointed_map[seq].point_n];
	for(int i = 0; i<this->pointed_map[seq].point_n; i++){
		this->pointed_map[seq].connection[i] = new int [this->pointed_map[seq].point_n];
	}

	// std::cout << "point_num = " << this->pointed_map[seq].point_n << std::endl;
	// std::cout << "line_map" << std::endl;

	for (int i = 0; i < this->pointed_map[seq].point_n; i++) {
		for (int j = 0; j < this->pointed_map[seq].point_n; j++) {
			this->pointed_map[seq].connection[i][j] = this->pointed_map[seq].connection[j][i] = 0;
			if (is_adjacent(temp_points[i],temp_points[j])){
				this->pointed_map[seq].connection[i][j] = this->pointed_map[seq].connection[j][i] = 1;
				this->pointed_map[seq].line_n += 1;
			}
			// std::cout << this->pointed_map[seq].connection[i][j];
		}
		// std::cout <<std::endl;
	}
}

/**
 * DFS (Depth First Search) Algorithm implementation 
 *
 * @params sequence number, point location x, visited vector, path_start vector
 * @return None
 */
void Labyrinth::dfs(int seq, int x, vector<int> visited, vector<int> &path_start) {

	visited.push_back(x);
	vector<int> path_point(visited);

	for(int i=0; i < this->pointed_map[seq].point_n; i++){
		vector<int>::iterator iter = find(visited.begin(), visited.end(), i);
		if(iter == visited.end() && this->pointed_map[seq].connection[x][i] == 1)
		{
			vector<int> path_tmp;
			dfs(seq, i, visited, path_tmp);
			if (path_point.size() < path_tmp.size()){
				path_point.assign(path_tmp.begin(), path_tmp.end());
			}
		}
	}
	path_start.assign(path_point.begin(),path_point.end());
}

/**
 * Finds the largest possible path between two holes in maze
 *
 * @variable Stores the largest possible path in largest_path class variable
 */
void Labyrinth::find_largest_path(){

	int sequences = this->path_points.size();
	for(int seq = 0; seq < sequences; seq++){

		build_path(seq);
		for(int start_i = 0; start_i<this->pointed_map[seq].point_n; start_i++){
			vector<int> visited;
			vector<int> path_start;
			dfs(seq, start_i, visited, path_start); //start search from strat_i

			if (this->pointed_map[seq].max_path.size() < path_start.size()){
				this->pointed_map[seq].max_path.assign(path_start.begin(), path_start.end());
			}
		}

		// std::cout << "Path Sequence" << std::endl;
		// for(int i=0; i<this->pointed_map[seq].max_path.size(); ++i)
		// 	std::cout << this->pointed_map[seq].max_path[i] << ' ';
		// std::cout << std::endl;


		vector<point> long_path;
		for (int i = 0; i<this->pointed_map[seq].max_path.size(); ++i){
			int point_seq = this->pointed_map[seq].max_path[i];
			vector<point> point_tmp(this->path_points[seq]);
			point add;
			add.x = point_tmp[point_seq].x;
			add.y = point_tmp[point_seq].y;
			long_path.push_back(add);
			// std::cout << "("<< long_path[i].x << "," << long_path[i].y << ") ";
		}
		// std::cout << std::endl;

		if (this->largest_path.size() < long_path.size()){
			this->largest_path.assign(long_path.begin(),long_path.end());
		}
	}

	// std::cout<< "Largest Pathway" <<std::endl;
	// for(int i=0; i < this->largest_path.size(); ++i)
	// 	std::cout << "("<< this->largest_path[i].x << "," << this->largest_path[i].y << ") ";
	// std::cout << std::endl;

}

/**
 * Converts holes to numbers in maze for largest path 
 *
 * @variable Stores the data in output_path class variable.
 */
void Labyrinth::convert_output_data() {
	this->output_data = this->input_data;
	for(int i=0; i < this->largest_path.size(); ++i)
	{
		string str_num = std::to_string(i);
		this->output_data[this->largest_path[i].x].replace(this->largest_path[i].y, 1, str_num);
	}
}

/**
 * Prints input maze and output maze with largest path
 */
void Labyrinth::print_data() {
	std::cout << "Rows, " << this->row << " & Columns, " << this->col << std::endl;

	std::cout << "\n=====> Input Maze <=====" << std::endl;
	for(const auto &row : this->input_data) {
		std::cout << row << std::endl;
	}

	convert_output_data();

	std::cout << "\n=====> Output Maze <=====" << std::endl;
	std::cout << this->largest_path.size() << std::endl;
	for(int i = 0; i < this->row; i++) {
		for (int j = 0; j < this->col; j++) {
			std::cout << this->output_data[i][j];
		}
		std::cout << std::endl;
	}

}

/**
 * Writes the output of maze in given output file path
 *
 */
void Labyrinth::write_output_file() {
	std::fstream outfile;
	outfile.open(this->output_file_path, std::ios::out);

	if(!outfile.is_open()) {
		std::cout << "Error while opening input file." << std::endl;
		return;
	}
	else {
		for(int i = 0; i < this->row; i++) {
			for (int j = 0; j < this->col; j++) {
				outfile << this->output_data[i][j];
			}
			outfile << std::endl;
		}
		outfile.close();
	}
}
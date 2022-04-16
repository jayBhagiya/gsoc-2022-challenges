/**
    Jderobot CPP Challenge: Main Source file, call to Labyrinth class.
    @file main.cpp
    @author Jay Bhagiya
*/
#include "labyrinth.h"

int main() {

	Labyrinth lb("../input.txt", "../output.txt");

	lb.map_input_data();
	lb.find_path_points();
	lb.find_largest_path();
	lb.print_data();
	lb.write_output_file();

	return 0;
}
/*
 * TODOS: TRY TO GET IT TO GO TO UNCLEANED CELLS, IF CANT FIND PATH, THEN GO THROUGH BAD CELLS ***** WRITE ANOTHER METHOD *****
 * Modified by Chuong Le
 * method pathToGoal -> aStarSearch -> tracePath or pathDistance
 * since we use index 0,1,2 when getting result we need to multiply by the scale
 * ***** SCALE CONVERSION IS AT pathToGoal && aStarSearch ***** So other method will insert in the actually value (0,1,2 * scale)
 * row = x, col = y
 * - Map in cpp:			- Map in real life:			However, direction result from map in cpp is good like the real life
 * 0 1 2 -> y					^ x
 * 1							|
 * 2							2
 * ||                           1
 * \/				   y <- 2 1 0
 * x
 *
 *
 */

#include <ros/ros.h>
#include <publish_goals/goals.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <stack>
#include <set>
#include <vector>
#include <utility>

#define ROW 9 
#define COL 10 
#define scale 0.4
#define reverse 1/scale

/*
#define ROW 3
#define COL 4
*/

// Creating a shortcut for int, int pair type 
typedef std::pair<int, int> Pair;

// Creating a shortcut for pair<int, pair<int, int>> type 
typedef std::pair<double, std::pair<int, int> > pPair;

// A structure to hold the neccesary parameters 
struct cell
{
	// Row and Column index of its parent 
	//  0 <= i <= ROW-1 & 0 <= j <= COL-1 
	int parent_i, parent_j;
	// f = g + h 
	double f, g, h;
};
class A_star
{
private:
	
	ros::NodeHandle n;
	ros::Publisher pub_goal_;
	
	/* For level search gives all (x,y) for a level */
	std::vector<double> x;	// lvl search
	std::vector<double> y;
	std::vector<int> weight;
	std::vector<double> x_single;	// all points from one to another
	std::vector<double> y_single;
	std::vector<double> x_path;	// all point path
	std::vector<double> y_path;
	publish_goals::goals goal_path;
	//int goal_counter = 0;

	int grid[ROW][COL] =
	{	        //0 0.4 0.8  3  4  5  6  7  8  9
		/*0*/	{ 1, 0, 1, 1, 1, 1, 0, 1, 1, 2 },
		/*0.4*/	{ 1, 1, 1, 0, 1, 3, 1, 1, 1, 1 },
		/*0.8*/	{ 1, 1, 1, 0, 1, 1, 0, 1, 0, 1 },
		/*1.2*/	{ 0, 0, 1, 0, 1, 0, 0, 0, 0, 1 },
		/*1.6*/	{ 1, 1, 1, 0, 1, 1, 1, 1, 1, 0 },
		/*2.0*/	{ 4, 0, 1, 1, 1, 1, 0, 1, 0, 0 },
		/*2.4*/	{ 4, 0, 0, 0, 0, 1, 0, 0, 0, 2 },
		/*2.8*/	{ 1, 0, 4, 1, 1, 1, 1, 3, 1, 1 },
		/*3.2*/	{ 1, 1, 1, 0, 0, 0, 1, 0, 0, 1 }
	};
/*
	int grid[ROW][COL] = 
	{
			{4,2,0,1},
			{1,0,3,4},
			{2,4,1,2}
	};
*/
	double distance_;

public:
	A_star(ros::NodeHandle &nh)
	{
		n = nh;
		pub_goal_ = n.advertise<publish_goals::goals>("/goal", 10);			
	//	pub_goal_2_ = n.advertise<publish_goal::goal>("/goal2", 10);

	}
	/* Returns true if row number and column numberis in range  */
	bool isValid(int row, int col)
	{
		return (row >= 0) && (row < ROW) &&
			(col >= 0) && (col < COL);
	}

/* Whether it is an obstacle */
bool isUnBlocked(int row, int col)
{
	// Returns true if the cell is not blocked else false 
	if (grid[row][col] == 0)
		return (false);
	else
		return (true);
}

bool cleaned(int row, int col)
{
	if (grid[row][col] > 0)
		return (false);
	else
		return (true);
}

// A Utility Function to check whether destination cell has 
// been reached or not 
bool isDestination(int row, int col, Pair dest)
{
	if (row == dest.first && col == dest.second)
		return (true);
	else
		return (false);
}

// A Utility Function to calculate the 'h' heuristics. 
double calculateHValue(int row, int col, Pair dest)
{
	// Return using the distance formula 
	return ((double)sqrt((row - dest.first)*(row - dest.first)
		+ (col - dest.second)*(col - dest.second)));
}

// A Utility Function to trace the path from the source 
// to destination 
void tracePath(cell cellDetails[][COL], Pair dest)
{
	int c = 0;
	//printf("\nThe Path is ");
	int row = dest.first;
	int col = dest.second;

//	stack<Pair> Path;
	std::stack<Pair> Path1;

	while (!(cellDetails[row][col].parent_i == row
		&& cellDetails[row][col].parent_j == col))
	{
//		Path.push(make_pair(row, col));
		Path1.push(std::make_pair(row, col));	// for scale
		int temp_row = cellDetails[row][col].parent_i;
		int temp_col = cellDetails[row][col].parent_j;
		row = temp_row;
		col = temp_col;
	}

//	Path.push(make_pair(row, col));
	Path1.push(std::make_pair(row, col));	// for scale
/*	while (!Path.empty())
	{
		pair<int, int> p = Path.top();
		Path.pop();
		printf("-> (%d,%d) ", p.first, p.second);
	}*/
	// for scale
	printf("\nThe Path is ");
	while (!Path1.empty())
	{
		std::pair<int, int> p = Path1.top();
		Path1.pop();
		//cout << "-> (" << (double)p.first * scale << "," << (double)p.second*scale << ") ";
		if (c == 0)
		{
			c++;
		}
		else
		{
			x_path.push_back(((double)p.first) * scale);	// put the coordinate leading to goal
			y_path.push_back(((double)p.second) * scale);
			weight.push_back(grid[p.first][p.second]);
		}
		grid[p.first][p.second] = -1;	// **** SET TO -1 CUZ ALREADY CLEANED but not an obstacle
	}

	return;
}
double distanceFormula(double x, double y, double xx, double yy)
{
	return sqrt((xx - x)*(xx - x) + (yy - y)*(yy - y));
}

void pathDistance(cell cellDetails[][COL], Pair dest)
{
	double distance = 0;
	//printf("\nThe Path is ");
	int row = dest.first;
	int col = dest.second;

	std::stack<Pair> Path;
	//std::stack<Pair> Path1;

	while (!(cellDetails[row][col].parent_i == row
		&& cellDetails[row][col].parent_j == col))
	{
		Path.push(std::make_pair(row, col));
		//Path1.push(make_pair(row, col));	// for scale
		int temp_row = cellDetails[row][col].parent_i;
		int temp_col = cellDetails[row][col].parent_j;
		row = temp_row;
		col = temp_col;
	}

	Path.push(std::make_pair(row, col));
	//Path1.push(std::make_pair(row, col));	// for scale
	double x_first;
	double y_first;
	double x_second;
	double y_second;
	int counter = 0;
	//printf("\nThe distance is: ");
	while (!Path.empty())
	{
		std::pair<int, int> p = Path.top();
		Path.pop();
		if (counter == 0)
		{
			x_first = ((double)p.first) * scale; y_first = ((double)p.second) * scale;
			counter++;
		}
		else
		{
			x_second = ((double)p.first) * scale; y_second = ((double)p.second) * scale;
			distance += distanceFormula(x_first, y_first, x_second, y_second);
			x_first = ((double)p.first) * scale; y_first = ((double)p.second) * scale;
		}
	}
	distance_ = distance;
	return;
}

// A Function to find the shortest path between 
// a given source cell to a destination cell according 
// to A* Search Algorithm 
void aStarSearch(Pair src, Pair dest, bool getDistance)
{
	// If the source is out of range 
	if (isValid(src.first, src.second) == false)
	{
		//printf("Source is invalid\n");
		return;
	}

	// If the destination is out of range 
	if (isValid(dest.first, dest.second) == false)
	{
		//printf("Destination is invalid\n");
		return;
	}

	// Either the source or the destination is blocked 
	if (isUnBlocked(src.first, src.second) == false ||
		isUnBlocked(dest.first, dest.second) == false)
	{
		//printf("Source or the destination is blocked\n");
		return;
	}

	// If the destination cell is the same as source cell 
	if (isDestination(src.first, src.second, dest) == true)
	{
		//printf("We are already at the destination\n");
		return;
	}

	// Create a closed list and initialise it to false which means 
	// that no cell has been included yet 
	// This closed list is implemented as a boolean 2D array 
	bool closedList[ROW][COL];
	memset(closedList, false, sizeof(closedList));

	// Declare a 2D array of structure to hold the details 
	//of that cell 
	cell cellDetails[ROW][COL];

	int i, j;

	for (i = 0; i < ROW; i++)
	{
		for (j = 0; j < COL; j++)
		{
			cellDetails[i][j].f = FLT_MAX;
			cellDetails[i][j].g = FLT_MAX;
			cellDetails[i][j].h = FLT_MAX;
			cellDetails[i][j].parent_i = -1;
			cellDetails[i][j].parent_j = -1;
		}
	}

	// Initialising the parameters of the starting node 
	i = src.first, j = src.second;
	cellDetails[i][j].f = 0.0;
	cellDetails[i][j].g = 0.0;
	cellDetails[i][j].h = 0.0;
	cellDetails[i][j].parent_i = i;
	cellDetails[i][j].parent_j = j;

	/*
	 Create an open list having information as-
	 <f, <i, j>>
	 where f = g + h,
	 and i, j are the row and column index of that cell
	 Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1
	 This open list is implenented as a set of pair of pair.*/
	std::set<pPair> openList;

	// Put the starting cell on the open list and set its 
	// 'f' as 0 
	openList.insert(std::make_pair(0.0, std::make_pair(i, j)));

	// We set this boolean value as false as initially 
	// the destination is not reached. 
	bool foundDest = false;

	while (!openList.empty())
	{
		pPair p = *openList.begin();

		// Remove this vertex from the open list 
		openList.erase(openList.begin());

		// Add this vertex to the closed list 
		i = p.second.first;
		j = p.second.second;
		closedList[i][j] = true;

		/*
		 Generating all the 8 successor of this cell

			 N.W   N   N.E
			   \   |   /
				\  |  /
			 W----Cell----E
				  / | \
				/   |  \
			 S.W    S   S.E

		 Cell-->Popped Cell (i, j)
		 N -->  North       (i-1, j)
		 S -->  South       (i+1, j)
		 E -->  East        (i, j+1)
		 W -->  West           (i, j-1)
		 N.E--> North-East  (i-1, j+1)
		 N.W--> North-West  (i-1, j-1)
		 S.E--> South-East  (i+1, j+1)
		 S.W--> South-West  (i+1, j-1)*/

		 // To store the 'g', 'h' and 'f' of the 8 successors 
		double gNew, hNew, fNew;

		//----------- 1st Successor (North) ------------ 

		// Only process this cell if this is a valid one 
		if (isValid(i - 1, j) == true)
		{
			// If the destination cell is the same as the 
			// current successor 
			if (isDestination(i - 1, j, dest) == true)
			{
				// Set the Parent of the destination cell 
				cellDetails[i - 1][j].parent_i = i;
				cellDetails[i - 1][j].parent_j = j;
				printf("The destination cell is found\n");
				if (getDistance)
				{
					pathDistance(cellDetails, dest);
				}
				else
				{
					tracePath(cellDetails, dest);
				}
				foundDest = true;
				return;
			}
			// If the successor is already on the closed 
			// list or if it is blocked, then ignore it. 
			// Else do the following 
			else if (closedList[i - 1][j] == false &&
				isUnBlocked(i - 1, j) == true)
			{
				gNew = cellDetails[i][j].g + 1.0;
				hNew = calculateHValue(i - 1, j, dest);
				fNew = gNew + hNew;

				// If it isn't on the open list, add it to 
				// the open list. Make the current square 
				// the parent of this square. Record the 
				// f, g, and h costs of the square cell 
				//                OR 
				// If it is on the open list already, check 
				// to see if this path to that square is better, 
				// using 'f' cost as the measure. 
				if (cellDetails[i - 1][j].f == FLT_MAX ||
					cellDetails[i - 1][j].f > fNew)
				{
					openList.insert(std::make_pair(fNew,
						std::make_pair(i - 1, j)));

					// Update the details of this cell 
					cellDetails[i - 1][j].f = fNew;
					cellDetails[i - 1][j].g = gNew;
					cellDetails[i - 1][j].h = hNew;
					cellDetails[i - 1][j].parent_i = i;
					cellDetails[i - 1][j].parent_j = j;
				}
			}
		}

		//----------- 2nd Successor (South) ------------ 

		// Only process this cell if this is a valid one 
		if (isValid(i + 1, j) == true)
		{
			// If the destination cell is the same as the 
			// current successor 
			if (isDestination(i + 1, j, dest) == true)
			{
				// Set the Parent of the destination cell 
				cellDetails[i + 1][j].parent_i = i;
				cellDetails[i + 1][j].parent_j = j;
				//printf("The destination cell is found\n");
				if (getDistance)
				{
					pathDistance(cellDetails, dest);
				}
				else
				{
					tracePath(cellDetails, dest);
				}
				foundDest = true;
				return;
			}
			// If the successor is already on the closed 
			// list or if it is blocked, then ignore it. 
			// Else do the following 
			else if (closedList[i + 1][j] == false &&
				isUnBlocked(i + 1, j) == true)
			{
				gNew = cellDetails[i][j].g + 1.0;
				hNew = calculateHValue(i + 1, j, dest);
				fNew = gNew + hNew;

				// If it isn't on the open list, add it to 
				// the open list. Make the current square 
				// the parent of this square. Record the 
				// f, g, and h costs of the square cell 
				//                OR 
				// If it is on the open list already, check 
				// to see if this path to that square is better, 
				// using 'f' cost as the measure. 
				if (cellDetails[i + 1][j].f == FLT_MAX ||
					cellDetails[i + 1][j].f > fNew)
				{
					openList.insert(std::make_pair(fNew, std::make_pair(i + 1, j)));
					// Update the details of this cell 
					cellDetails[i + 1][j].f = fNew;
					cellDetails[i + 1][j].g = gNew;
					cellDetails[i + 1][j].h = hNew;
					cellDetails[i + 1][j].parent_i = i;
					cellDetails[i + 1][j].parent_j = j;
				}
			}
		}

		//----------- 3rd Successor (East) ------------ 

		// Only process this cell if this is a valid one 
		if (isValid(i, j + 1) == true)
		{
			// If the destination cell is the same as the 
			// current successor 
			if (isDestination(i, j + 1, dest) == true)
			{
				// Set the Parent of the destination cell 
				cellDetails[i][j + 1].parent_i = i;
				cellDetails[i][j + 1].parent_j = j;
				//printf("The destination cell is found\n");
				if (getDistance)
				{
					pathDistance(cellDetails, dest);
				}
				else
				{
					tracePath(cellDetails, dest);
				}
				foundDest = true;
				return;
			}

			// If the successor is already on the closed 
			// list or if it is blocked, then ignore it. 
			// Else do the following 
			else if (closedList[i][j + 1] == false &&
				isUnBlocked(i, j + 1) == true)
			{
				gNew = cellDetails[i][j].g + 1.0;
				hNew = calculateHValue(i, j + 1, dest);
				fNew = gNew + hNew;

				// If it isn't on the open list, add it to 
				// the open list. Make the current square 
				// the parent of this square. Record the 
				// f, g, and h costs of the square cell 
				//                OR 
				// If it is on the open list already, check 
				// to see if this path to that square is better, 
				// using 'f' cost as the measure. 
				if (cellDetails[i][j + 1].f == FLT_MAX ||
					cellDetails[i][j + 1].f > fNew)
				{
					openList.insert(std::make_pair(fNew,
						std::make_pair(i, j + 1)));

					// Update the details of this cell 
					cellDetails[i][j + 1].f = fNew;
					cellDetails[i][j + 1].g = gNew;
					cellDetails[i][j + 1].h = hNew;
					cellDetails[i][j + 1].parent_i = i;
					cellDetails[i][j + 1].parent_j = j;
				}
			}
		}

		//----------- 4th Successor (West) ------------ 

		// Only process this cell if this is a valid one 
		if (isValid(i, j - 1) == true)
		{
			// If the destination cell is the same as the 
			// current successor 
			if (isDestination(i, j - 1, dest) == true)
			{
				// Set the Parent of the destination cell 
				cellDetails[i][j - 1].parent_i = i;
				cellDetails[i][j - 1].parent_j = j;
				//printf("The destination cell is found\n");
				if (getDistance)
				{
					pathDistance(cellDetails, dest);
				}
				else
				{
					tracePath(cellDetails, dest);
				}
				foundDest = true;
				return;
			}

			// If the successor is already on the closed 
			// list or if it is blocked, then ignore it. 
			// Else do the following 
			else if (closedList[i][j - 1] == false &&
				isUnBlocked(i, j - 1) == true)
			{
				gNew = cellDetails[i][j].g + 1.0;
				hNew = calculateHValue(i, j - 1, dest);
				fNew = gNew + hNew;

				// If it isn't on the open list, add it to 
				// the open list. Make the current square 
				// the parent of this square. Record the 
				// f, g, and h costs of the square cell 
				//                OR 
				// If it is on the open list already, check 
				// to see if this path to that square is better, 
				// using 'f' cost as the measure. 
				if (cellDetails[i][j - 1].f == FLT_MAX ||
					cellDetails[i][j - 1].f > fNew)
				{
					openList.insert(std::make_pair(fNew,
						std::make_pair(i, j - 1)));

					// Update the details of this cell 
					cellDetails[i][j - 1].f = fNew;
					cellDetails[i][j - 1].g = gNew;
					cellDetails[i][j - 1].h = hNew;
					cellDetails[i][j - 1].parent_i = i;
					cellDetails[i][j - 1].parent_j = j;
				}
			}
		}

		//----------- 5th Successor (North-East) ------------ 

		// Only process this cell if this is a valid one		
		if (isValid(i - 1, j + 1) == true && isUnBlocked(i - 1, j) == true && isUnBlocked(i, j + 1) == true)
		{
			// If the destination cell is the same as the 
			// current successor 
			if (isDestination(i - 1, j + 1, dest) == true)
			{
				// Set the Parent of the destination cell 
				cellDetails[i - 1][j + 1].parent_i = i;
				cellDetails[i - 1][j + 1].parent_j = j;
				//printf("The destination cell is found\n");
				if (getDistance)
				{
					pathDistance(cellDetails, dest);
				}
				else
				{
					tracePath(cellDetails, dest);
				}
				foundDest = true;
				return;
			}

			// If the successor is already on the closed 
			// list or if it is blocked, then ignore it. 
			// Else do the following 
			else if (closedList[i - 1][j + 1] == false &&
				isUnBlocked(i - 1, j + 1) == true)
			{
				gNew = cellDetails[i][j].g + 1.414;
				hNew = calculateHValue(i - 1, j + 1, dest);
				fNew = gNew + hNew;

				// If it isn't on the open list, add it to 
				// the open list. Make the current square 
				// the parent of this square. Record the 
				// f, g, and h costs of the square cell 
				//                OR 
				// If it is on the open list already, check 
				// to see if this path to that square is better, 
				// using 'f' cost as the measure. 
				if (cellDetails[i - 1][j + 1].f == FLT_MAX ||
					cellDetails[i - 1][j + 1].f > fNew)
				{
					openList.insert(std::make_pair(fNew,
						std::make_pair(i - 1, j + 1)));

					// Update the details of this cell 
					cellDetails[i - 1][j + 1].f = fNew;
					cellDetails[i - 1][j + 1].g = gNew;
					cellDetails[i - 1][j + 1].h = hNew;
					cellDetails[i - 1][j + 1].parent_i = i;
					cellDetails[i - 1][j + 1].parent_j = j;
				}
			}
		}

		//----------- 6th Successor (North-West) ------------ 

		// Only process this cell if this is a valid one 
		if (isValid(i - 1, j - 1) == true && isUnBlocked(i - 1, j) == true && isUnBlocked(i, j - 1) == true)
		{
			// If the destination cell is the same as the 
			// current successor 
			if (isDestination(i - 1, j - 1, dest) == true)
			{
				// Set the Parent of the destination cell 
				cellDetails[i - 1][j - 1].parent_i = i;
				cellDetails[i - 1][j - 1].parent_j = j;
				//printf("The destination cell is found\n");
				if (getDistance)
				{
					pathDistance(cellDetails, dest);
				}
				else
				{
					tracePath(cellDetails, dest);
				}
				foundDest = true;
				return;
			}

			// If the successor is already on the closed 
			// list or if it is blocked, then ignore it. 
			// Else do the following 
			else if (closedList[i - 1][j - 1] == false &&
				isUnBlocked(i - 1, j - 1) == true)
			{
				gNew = cellDetails[i][j].g + 1.414;
				hNew = calculateHValue(i - 1, j - 1, dest);
				fNew = gNew + hNew;

				// If it isn on the open list, add it to 
				// the open list. Make the current square 
				// the parent of this square. Record the 
				// f, g, and h costs of the square cell 
				//                OR 
				// If it is on the open list already, check 
				// to see if this path to that square is better, 
				// using 'f' cost as the measure. 
				if (cellDetails[i - 1][j - 1].f == FLT_MAX ||
					cellDetails[i - 1][j - 1].f > fNew)
				{
					openList.insert(std::make_pair(fNew, std::make_pair(i - 1, j - 1)));
					// Update the details of this cell 
					cellDetails[i - 1][j - 1].f = fNew;
					cellDetails[i - 1][j - 1].g = gNew;
					cellDetails[i - 1][j - 1].h = hNew;
					cellDetails[i - 1][j - 1].parent_i = i;
					cellDetails[i - 1][j - 1].parent_j = j;
				}
			}
		}

		//----------- 7th Successor (South-East) ------------ 

		// Only process this cell if this is a valid one 
		if (isValid(i + 1, j + 1) == true && isUnBlocked(i + 1, j) == true && isUnBlocked(i, j + 1) == true)
		{
			// If the destination cell is the same as the 
			// current successor 
			if (isDestination(i + 1, j + 1, dest) == true)
			{
				// Set the Parent of the destination cell 
				cellDetails[i + 1][j + 1].parent_i = i;
				cellDetails[i + 1][j + 1].parent_j = j;
				//printf("The destination cell is found\n");
				if (getDistance)
				{
					pathDistance(cellDetails, dest);
				}
				else
				{
					tracePath(cellDetails, dest);
				}
				foundDest = true;
				return;
			}

			// If the successor is already on the closed 
			// list or if it is blocked, then ignore it. 
			// Else do the following 
			else if (closedList[i + 1][j + 1] == false &&
				isUnBlocked(i + 1, j + 1) == true)
			{
				gNew = cellDetails[i][j].g + 1.414;
				hNew = calculateHValue(i + 1, j + 1, dest);
				fNew = gNew + hNew;

				// If it isn\92t on the open list, add it to 
				// the open list. Make the current square 
				// the parent of this square. Record the 
				// f, g, and h costs of the square cell 
				//                OR 
				// If it is on the open list already, check 
				// to see if this path to that square is better, 
				// using 'f' cost as the measure. 
				if (cellDetails[i + 1][j + 1].f == FLT_MAX ||
					cellDetails[i + 1][j + 1].f > fNew)
				{
					openList.insert(std::make_pair(fNew,
						std::make_pair(i + 1, j + 1)));

					// Update the details of this cell 
					cellDetails[i + 1][j + 1].f = fNew;
					cellDetails[i + 1][j + 1].g = gNew;
					cellDetails[i + 1][j + 1].h = hNew;
					cellDetails[i + 1][j + 1].parent_i = i;
					cellDetails[i + 1][j + 1].parent_j = j;
				}
			}
		}

		//----------- 8th Successor (South-West) ------------ 

		// Only process this cell if this is a valid one 
		if (isValid(i + 1, j - 1) == true && isUnBlocked(i + 1, j) == true && isUnBlocked(i, j - 1) == true)
		{
			// If the destination cell is the same as the 
			// current successor 
			if (isDestination(i + 1, j - 1, dest) == true)
			{
				// Set the Parent of the destination cell 
				cellDetails[i + 1][j - 1].parent_i = i;
				cellDetails[i + 1][j - 1].parent_j = j;
				//printf("The destination cell is found\n");
				if (getDistance)
				{
					pathDistance(cellDetails, dest);
				}
				else
				{
					tracePath(cellDetails, dest);
				}
				foundDest = true;
				return;
			}

			// If the successor is already on the closed 
			// list or if it is blocked, then ignore it. 
			// Else do the following 
			else if (closedList[i + 1][j - 1] == false &&
				isUnBlocked(i + 1, j - 1) == true)
			{
				gNew = cellDetails[i][j].g + 1.414;
				hNew = calculateHValue(i + 1, j - 1, dest);
				fNew = gNew + hNew;

				// If it isnt on the open list, add it to 
				// the open list. Make the current square 
				// the parent of this square. Record the 
				// f, g, and h costs of the square cell 
				//                OR 
				// If it is on the open list already, check 
				// to see if this path to that square is better, 
				// using 'f' cost as the measure. 
				if (cellDetails[i + 1][j - 1].f == FLT_MAX ||
					cellDetails[i + 1][j - 1].f > fNew)
				{
					openList.insert(std::make_pair(fNew,
						std::make_pair(i + 1, j - 1)));

					// Update the details of this cell 
					cellDetails[i + 1][j - 1].f = fNew;
					cellDetails[i + 1][j - 1].g = gNew;
					cellDetails[i + 1][j - 1].h = hNew;
					cellDetails[i + 1][j - 1].parent_i = i;
					cellDetails[i + 1][j - 1].parent_j = j;
				}
			}
		}
	}

	// When the destination cell is not found and the open 
	// list is empty, then we conclude that we failed to 
	// reach the destiantion cell. This may happen when the 
	// there is no way to destination cell (due to blockages) 
	if (foundDest == false)
		printf("Failed to find the Destination Cell\n");

	return;
}

/*
 * In this aStar method, -1 and 0 is obtacle because we want to prioritize uncleaned space
 * If it can't find one then it will use the aStar above
 */
bool aStarSearch2(Pair src, Pair dest, bool getDistance)
{
	// If the source is out of range 
	if (isValid(src.first, src.second) == false)
	{
		//printf("Source is invalid\n");
		return false;
	}

	// If the destination is out of range 
	if (isValid(dest.first, dest.second) == false)
	{
		//printf("Destination is invalid\n");
		return false;
	}

	// Either the source or the destination is blocked 
	if (isUnBlocked(src.first, src.second) == false ||
		isUnBlocked(dest.first, dest.second) == false)
	{
		//printf("Source or the destination is blocked\n");
		return false;
	}

	// If the destination cell is the same as source cell 
	if (isDestination(src.first, src.second, dest) == true)
	{
		printf("We are already at the destination\n");
		return true;
	}

	// Create a closed list and initialise it to false which means 
	// that no cell has been included yet 
	// This closed list is implemented as a boolean 2D array 
	bool closedList[ROW][COL];
	memset(closedList, false, sizeof(closedList));

	// Declare a 2D array of structure to hold the details 
	//of that cell 
	cell cellDetails[ROW][COL];

	int i, j;

	for (i = 0; i < ROW; i++)
	{
		for (j = 0; j < COL; j++)
		{
			cellDetails[i][j].f = FLT_MAX;
			cellDetails[i][j].g = FLT_MAX;
			cellDetails[i][j].h = FLT_MAX;
			cellDetails[i][j].parent_i = -1;
			cellDetails[i][j].parent_j = -1;
		}
	}

	// Initialising the parameters of the starting node 
	i = src.first, j = src.second;
	cellDetails[i][j].f = 0.0;
	cellDetails[i][j].g = 0.0;
	cellDetails[i][j].h = 0.0;
	cellDetails[i][j].parent_i = i;
	cellDetails[i][j].parent_j = j;

	/*
	 Create an open list having information as-
	 <f, <i, j>>
	 where f = g + h,
	 and i, j are the row and column index of that cell
	 Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1
	 This open list is implenented as a set of pair of pair.*/
	std::set<pPair> openList;

	// Put the starting cell on the open list and set its 
	// 'f' as 0 
	openList.insert(std::make_pair(0.0, std::make_pair(i, j)));

	// We set this boolean value as false as initially 
	// the destination is not reached. 
	bool foundDest = false;

	while (!openList.empty())
	{
		pPair p = *openList.begin();

		// Remove this vertex from the open list 
		openList.erase(openList.begin());

		// Add this vertex to the closed list 
		i = p.second.first;
		j = p.second.second;
		closedList[i][j] = true;

		/*
		 Generating all the 8 successor of this cell

			 N.W   N   N.E
			   \   |   /
				\  |  /
			 W----Cell----E
				  / | \
				/   |  \
			 S.W    S   S.E

		 Cell-->Popped Cell (i, j)
		 N -->  North       (i-1, j)
		 S -->  South       (i+1, j)
		 E -->  East        (i, j+1)
		 W -->  West           (i, j-1)
		 N.E--> North-East  (i-1, j+1)
		 N.W--> North-West  (i-1, j-1)
		 S.E--> South-East  (i+1, j+1)
		 S.W--> South-West  (i+1, j-1)*/

		 // To store the 'g', 'h' and 'f' of the 8 successors 
		double gNew, hNew, fNew;

		//----------- 1st Successor (North) ------------ 

		// Only process this cell if this is a valid one 
		if (isValid(i - 1, j) == true)
		{
			// If the destination cell is the same as the 
			// current successor 
			if (isDestination(i - 1, j, dest) == true)
			{
				// Set the Parent of the destination cell 
				cellDetails[i - 1][j].parent_i = i;
				cellDetails[i - 1][j].parent_j = j;
				printf("The destination cell is found\n");
				if (getDistance)
				{
					pathDistance(cellDetails, dest);
				}
				else
				{
					tracePath(cellDetails, dest);
				}
				foundDest = true;
				return true;
			}
			// If the successor is already on the closed 
			// list or if it is blocked, then ignore it. 
			// Else do the following 
			else if (closedList[i - 1][j] == false &&
				isUnBlocked(i - 1, j) == true && cleaned(i - 1,j) == false)
			{
				gNew = cellDetails[i][j].g + 1.0;
				hNew = calculateHValue(i - 1, j, dest);
				fNew = gNew + hNew;

				// If it isn't on the open list, add it to 
				// the open list. Make the current square 
				// the parent of this square. Record the 
				// f, g, and h costs of the square cell 
				//                OR 
				// If it is on the open list already, check 
				// to see if this path to that square is better, 
				// using 'f' cost as the measure. 
				if (cellDetails[i - 1][j].f == FLT_MAX ||
					cellDetails[i - 1][j].f > fNew)
				{
					openList.insert(std::make_pair(fNew,
						std::make_pair(i - 1, j)));

					// Update the details of this cell 
					cellDetails[i - 1][j].f = fNew;
					cellDetails[i - 1][j].g = gNew;
					cellDetails[i - 1][j].h = hNew;
					cellDetails[i - 1][j].parent_i = i;
					cellDetails[i - 1][j].parent_j = j;
				}
			}
		}

		//----------- 2nd Successor (South) ------------ 

		// Only process this cell if this is a valid one 
		if (isValid(i + 1, j) == true)
		{
			// If the destination cell is the same as the 
			// current successor 
			if (isDestination(i + 1, j, dest) == true)
			{
				// Set the Parent of the destination cell 
				cellDetails[i + 1][j].parent_i = i;
				cellDetails[i + 1][j].parent_j = j;
				printf("The destination cell is found\n");
				if (getDistance)
				{
					pathDistance(cellDetails, dest);
				}
				else
				{
					tracePath(cellDetails, dest);
				}
				foundDest = true;
				return true;
			}
			// If the successor is already on the closed 
			// list or if it is blocked, then ignore it. 
			// Else do the following 
			else if (closedList[i + 1][j] == false &&
				isUnBlocked(i + 1, j) == true && cleaned(i - 1, j) == false)
			{
				gNew = cellDetails[i][j].g + 1.0;
				hNew = calculateHValue(i + 1, j, dest);
				fNew = gNew + hNew;

				// If it isn't on the open list, add it to 
				// the open list. Make the current square 
				// the parent of this square. Record the 
				// f, g, and h costs of the square cell 
				//                OR 
				// If it is on the open list already, check 
				// to see if this path to that square is better, 
				// using 'f' cost as the measure. 
				if (cellDetails[i + 1][j].f == FLT_MAX ||
					cellDetails[i + 1][j].f > fNew)
				{
					openList.insert(std::make_pair(fNew, std::make_pair(i + 1, j)));
					// Update the details of this cell 
					cellDetails[i + 1][j].f = fNew;
					cellDetails[i + 1][j].g = gNew;
					cellDetails[i + 1][j].h = hNew;
					cellDetails[i + 1][j].parent_i = i;
					cellDetails[i + 1][j].parent_j = j;
				}
			}
		}

		//----------- 3rd Successor (East) ------------ 

		// Only process this cell if this is a valid one 
		if (isValid(i, j + 1) == true)
		{
			// If the destination cell is the same as the 
			// current successor 
			if (isDestination(i, j + 1, dest) == true)
			{
				// Set the Parent of the destination cell 
				cellDetails[i][j + 1].parent_i = i;
				cellDetails[i][j + 1].parent_j = j;
				printf("The destination cell is found\n");
				if (getDistance)
				{
					pathDistance(cellDetails, dest);
				}
				else
				{
					tracePath(cellDetails, dest);
				}
				foundDest = true;
				return true;
			}

			// If the successor is already on the closed 
			// list or if it is blocked, then ignore it. 
			// Else do the following 
			else if (closedList[i][j + 1] == false &&
				isUnBlocked(i, j + 1) == true && cleaned(i - 1, j) == false)
			{
				gNew = cellDetails[i][j].g + 1.0;
				hNew = calculateHValue(i, j + 1, dest);
				fNew = gNew + hNew;

				// If it isn't on the open list, add it to 
				// the open list. Make the current square 
				// the parent of this square. Record the 
				// f, g, and h costs of the square cell 
				//                OR 
				// If it is on the open list already, check 
				// to see if this path to that square is better, 
				// using 'f' cost as the measure. 
				if (cellDetails[i][j + 1].f == FLT_MAX ||
					cellDetails[i][j + 1].f > fNew)
				{
					openList.insert(std::make_pair(fNew,
						std::make_pair(i, j + 1)));

					// Update the details of this cell 
					cellDetails[i][j + 1].f = fNew;
					cellDetails[i][j + 1].g = gNew;
					cellDetails[i][j + 1].h = hNew;
					cellDetails[i][j + 1].parent_i = i;
					cellDetails[i][j + 1].parent_j = j;
				}
			}
		}

		//----------- 4th Successor (West) ------------ 

		// Only process this cell if this is a valid one 
		if (isValid(i, j - 1) == true)
		{
			// If the destination cell is the same as the 
			// current successor 
			if (isDestination(i, j - 1, dest) == true)
			{
				// Set the Parent of the destination cell 
				cellDetails[i][j - 1].parent_i = i;
				cellDetails[i][j - 1].parent_j = j;
				printf("The destination cell is found\n");
				if (getDistance)
				{
					pathDistance(cellDetails, dest);
				}
				else
				{
					tracePath(cellDetails, dest);
				}
				foundDest = true;
				return true;
			}

			// If the successor is already on the closed 
			// list or if it is blocked, then ignore it. 
			// Else do the following 
			else if (closedList[i][j - 1] == false &&
				isUnBlocked(i, j - 1) == true && cleaned(i - 1, j) == false)
			{
				gNew = cellDetails[i][j].g + 1.0;
				hNew = calculateHValue(i, j - 1, dest);
				fNew = gNew + hNew;

				// If it isn't on the open list, add it to 
				// the open list. Make the current square 
				// the parent of this square. Record the 
				// f, g, and h costs of the square cell 
				//                OR 
				// If it is on the open list already, check 
				// to see if this path to that square is better, 
				// using 'f' cost as the measure. 
				if (cellDetails[i][j - 1].f == FLT_MAX ||
					cellDetails[i][j - 1].f > fNew)
				{
					openList.insert(std::make_pair(fNew,
						std::make_pair(i, j - 1)));

					// Update the details of this cell 
					cellDetails[i][j - 1].f = fNew;
					cellDetails[i][j - 1].g = gNew;
					cellDetails[i][j - 1].h = hNew;
					cellDetails[i][j - 1].parent_i = i;
					cellDetails[i][j - 1].parent_j = j;
				}
			}
		}

		//----------- 5th Successor (North-East) ------------ 

		// Only process this cell if this is a valid one		
		if (isValid(i - 1, j + 1) == true && isUnBlocked(i - 1, j) == true && isUnBlocked(i, j + 1) == true && cleaned(i - 1, j) == false)
		{
			// If the destination cell is the same as the 
			// current successor 
			if (isDestination(i - 1, j + 1, dest) == true)
			{
				// Set the Parent of the destination cell 
				cellDetails[i - 1][j + 1].parent_i = i;
				cellDetails[i - 1][j + 1].parent_j = j;
				printf("The destination cell is found\n");
				if (getDistance)
				{
					pathDistance(cellDetails, dest);
				}
				else
				{
					tracePath(cellDetails, dest);
				}
				foundDest = true;
				return true;
			}

			// If the successor is already on the closed 
			// list or if it is blocked, then ignore it. 
			// Else do the following 
			else if (closedList[i - 1][j + 1] == false &&
				isUnBlocked(i - 1, j + 1) == true)
			{
				gNew = cellDetails[i][j].g + 1.414;
				hNew = calculateHValue(i - 1, j + 1, dest);
				fNew = gNew + hNew;

				// If it isn't on the open list, add it to 
				// the open list. Make the current square 
				// the parent of this square. Record the 
				// f, g, and h costs of the square cell 
				//                OR 
				// If it is on the open list already, check 
				// to see if this path to that square is better, 
				// using 'f' cost as the measure. 
				if (cellDetails[i - 1][j + 1].f == FLT_MAX ||
					cellDetails[i - 1][j + 1].f > fNew)
				{
					openList.insert(std::make_pair(fNew,
						std::make_pair(i - 1, j + 1)));

					// Update the details of this cell 
					cellDetails[i - 1][j + 1].f = fNew;
					cellDetails[i - 1][j + 1].g = gNew;
					cellDetails[i - 1][j + 1].h = hNew;
					cellDetails[i - 1][j + 1].parent_i = i;
					cellDetails[i - 1][j + 1].parent_j = j;
				}
			}
		}

		//----------- 6th Successor (North-West) ------------ 

		// Only process this cell if this is a valid one 
		if (isValid(i - 1, j - 1) == true && isUnBlocked(i - 1, j) == true && isUnBlocked(i, j - 1) == true && cleaned(i - 1, j) == false)
		{
			// If the destination cell is the same as the 
			// current successor 
			if (isDestination(i - 1, j - 1, dest) == true)
			{
				// Set the Parent of the destination cell 
				cellDetails[i - 1][j - 1].parent_i = i;
				cellDetails[i - 1][j - 1].parent_j = j;
				printf("The destination cell is found\n");
				if (getDistance)
				{
					pathDistance(cellDetails, dest);
				}
				else
				{
					tracePath(cellDetails, dest);
				}
				foundDest = true;
				return true;
			}

			// If the successor is already on the closed 
			// list or if it is blocked, then ignore it. 
			// Else do the following 
			else if (closedList[i - 1][j - 1] == false &&
				isUnBlocked(i - 1, j - 1) == true)
			{
				gNew = cellDetails[i][j].g + 1.414;
				hNew = calculateHValue(i - 1, j - 1, dest);
				fNew = gNew + hNew;

				// If it isn't on the open list, add it to 
				// the open list. Make the current square 
				// the parent of this square. Record the 
				// f, g, and h costs of the square cell 
				//                OR 
				// If it is on the open list already, check 
				// to see if this path to that square is better, 
				// using 'f' cost as the measure. 
				if (cellDetails[i - 1][j - 1].f == FLT_MAX ||
					cellDetails[i - 1][j - 1].f > fNew)
				{
					openList.insert(std::make_pair(fNew, std::make_pair(i - 1, j - 1)));
					// Update the details of this cell 
					cellDetails[i - 1][j - 1].f = fNew;
					cellDetails[i - 1][j - 1].g = gNew;
					cellDetails[i - 1][j - 1].h = hNew;
					cellDetails[i - 1][j - 1].parent_i = i;
					cellDetails[i - 1][j - 1].parent_j = j;
				}
			}
		}

		//----------- 7th Successor (South-East) ------------ 

		// Only process this cell if this is a valid one 
		if (isValid(i + 1, j + 1) == true && isUnBlocked(i + 1, j) == true && isUnBlocked(i, j + 1) == true && cleaned(i - 1, j) == false)
		{
			// If the destination cell is the same as the 
			// current successor 
			if (isDestination(i + 1, j + 1, dest) == true)
			{
				// Set the Parent of the destination cell 
				cellDetails[i + 1][j + 1].parent_i = i;
				cellDetails[i + 1][j + 1].parent_j = j;
				printf("The destination cell is found\n");
				if (getDistance)
				{
					pathDistance(cellDetails, dest);
				}
				else
				{
					tracePath(cellDetails, dest);
				}
				foundDest = true;
				return true;
			}

			// If the successor is already on the closed 
			// list or if it is blocked, then ignore it. 
			// Else do the following 
			else if (closedList[i + 1][j + 1] == false &&
				isUnBlocked(i + 1, j + 1) == true)
			{
				gNew = cellDetails[i][j].g + 1.414;
				hNew = calculateHValue(i + 1, j + 1, dest);
				fNew = gNew + hNew;

				// If it isn't on the open list, add it to 
				// the open list. Make the current square 
				// the parent of this square. Record the 
				// f, g, and h costs of the square cell 
				//                OR 
				// If it is on the open list already, check 
				// to see if this path to that square is better, 
				// using 'f' cost as the measure. 
				if (cellDetails[i + 1][j + 1].f == FLT_MAX ||
					cellDetails[i + 1][j + 1].f > fNew)
				{
					openList.insert(std::make_pair(fNew,
						std::make_pair(i + 1, j + 1)));

					// Update the details of this cell 
					cellDetails[i + 1][j + 1].f = fNew;
					cellDetails[i + 1][j + 1].g = gNew;
					cellDetails[i + 1][j + 1].h = hNew;
					cellDetails[i + 1][j + 1].parent_i = i;
					cellDetails[i + 1][j + 1].parent_j = j;
				}
			}
		}

		//----------- 8th Successor (South-West) ------------ 

		// Only process this cell if this is a valid one 
		if (isValid(i + 1, j - 1) == true && isUnBlocked(i + 1, j) == true && isUnBlocked(i, j - 1) == true && cleaned(i - 1, j) == false)
		{
			// If the destination cell is the same as the 
			// current successor 
			if (isDestination(i + 1, j - 1, dest) == true)
			{
				// Set the Parent of the destination cell 
				cellDetails[i + 1][j - 1].parent_i = i;
				cellDetails[i + 1][j - 1].parent_j = j;
				printf("The destination cell is found\n");
				if (getDistance)
				{
					pathDistance(cellDetails, dest);
				}
				else
				{
					tracePath(cellDetails, dest);
				}
				foundDest = true;
				return true;
			}

			// If the successor is already on the closed 
			// list or if it is blocked, then ignore it. 
			// Else do the following 
			else if (closedList[i + 1][j - 1] == false &&
				isUnBlocked(i + 1, j - 1) == true)
			{
				gNew = cellDetails[i][j].g + 1.414;
				hNew = calculateHValue(i + 1, j - 1, dest);
				fNew = gNew + hNew;

				// If it isn't on the open list, add it to 
				// the open list. Make the current square 
				// the parent of this square. Record the 
				// f, g, and h costs of the square cell 
				//                OR 
				// If it is on the open list already, check 
				// to see if this path to that square is better, 
				// using 'f' cost as the measure. 
				if (cellDetails[i + 1][j - 1].f == FLT_MAX ||
					cellDetails[i + 1][j - 1].f > fNew)
				{
					openList.insert(std::make_pair(fNew,
						std::make_pair(i + 1, j - 1)));

					// Update the details of this cell 
					cellDetails[i + 1][j - 1].f = fNew;
					cellDetails[i + 1][j - 1].g = gNew;
					cellDetails[i + 1][j - 1].h = hNew;
					cellDetails[i + 1][j - 1].parent_i = i;
					cellDetails[i + 1][j - 1].parent_j = j;
				}
			}
		}
	}

	// When the destination cell is not found and the open 
	// list is empty, then we conclude that we failed to 
	// reach the destiantion cell. This may happen when the 
	// there is no way to destination cell (due to blockages) 
	if (foundDest == false)
	{
		printf("Failed to find the Destination Cell without clean\n");
		return false;
	}
}

void pathToGoal(double x_current, double y_current, double x_goal, double y_goal, bool getDistance)
{

	// Source is the left-most bottom-most corner 	
	Pair src = std::make_pair(x_current * reverse, y_current * reverse);

	// Destination is the left-most top-most corner 
	Pair dest = std::make_pair(x_goal * reverse, y_goal * reverse);

	bool path = aStarSearch2(src, dest, getDistance);
	/* If can't find a good path, then run the second option */
	if (path == false)	
	{
		aStarSearch(src, dest, getDistance);
	}
}

/*void printToScale(int xx, int yy, double s)
{
	cout << "(" << ((double)xx) * s << "," << ((double)yy) * s << ")";
}*/

/*
 * @param level		the dirt level
 *		find the (x,y) coordinate of all cell with the dirt level and put it into the vector x & y
 */
void levelSearch(int level)
{
	/* clear vector to get a clean start */
	x.clear();
	y.clear();
	//cout << "Level: ";
	for (int i = 0; i < ROW; i++)
	{
		for (int j = 0; j < COL; j++)
		{
			if (grid[i][j] == level)
			{
				x.push_back(((double)i) * scale);
				y.push_back(((double)j) * scale);
				//cout << "(" << i << "," << j << ")";
			}
		}
	}
	//cout << endl;
}

void pointOrder(int highest_lvl, double current_x, double current_y)
{
	
	double x_small, y_small; 
	int index;	
	while (highest_lvl > 0)	// ***JUST FOR NOW*** since zero means no dirt/cleaned
	{
		levelSearch(highest_lvl);	// put all points with that level 4 into lvl x & y array
		while (!x.empty())	
		{
			double smallest = DBL_MAX;
			/* determin which point is closest to current position */
			for (int i = 0; i < x.size(); i++)
			{
				pathToGoal(current_x, current_y, x.at(i), y.at(i), true);	// get distance between current pose and each of the coordinate
				if (smallest > distance_)	// find smallest distance to go to
				{
					x_small = x.at(i); y_small = y.at(i);
					smallest = distance_;
					index = i;
				}
			}			
/*			for (int i = 0; i < x.size(); i++)
			{
				cout << "(" << x.at(i) << "," << y.at(i) << ") ";
			}*/

			/* put (x,y) into vector */
			
			pathToGoal(current_x, current_y, x_small, y_small, false);	// put path point into x_path & y_path
			/* set current x,y to find the next point */
			printf("Erase");
			current_x = x_small;
			current_y = y_small;
			/* delete the point from the lvl array */
			if (x.size() == 1)
			{
				x.clear();	
				y.clear();
			}
			else
			{
				x.erase(x.begin() + index);	
				y.erase(y.begin() + index);
			}			
			/*for (int i = 0; i < x.size(); i++)
			{
				cout << "(" << x.at(i) << "," << y.at(i) << ") ";
			}
			cout << endl;*/
			printf("Empty: %d", x.empty());

			
			//printf("Highest lvl: %d", highest_lvl);
		}
		
		
		/* find best path for the next highest cell*/
		highest_lvl--;
		printf("Highest lvl: %d", highest_lvl);
	}
	//ROS_INFO("Number of goals: %d\n", x_path.size());
	//cout << "\n Number of goals: " << x_path.size();

	ros::Rate r(10);
	goal_path.x.resize(x_path.size());
	goal_path.y.resize(y_path.size());
	goal_path.weight.resize(weight.size());
	goal_path.size = x_path.size();
	for (int i = 0; i < x_path.size(); i++)
	{
		goal_path.x[i] = x_path.at(i);
		goal_path.y[i] = y_path.at(i);
		goal_path.weight[i] = weight.at(i);
	}
	//ROS_INFO("Number of goals: %d\n", x_path.size());
	while (ros::ok())
	{
		pub_goal_.publish(goal_path);
		r.sleep();
		ros::spinOnce();
	}
	//ROS_INFO("Number of goals: %d\n", x_path.size());
}

void printPath()
{
	//cout << "Path: ";
	for (int i = 0; i < x_path.size(); i++)
	{
		//cout << "(" << x_path.at(i) << "," << y_path.at(i) << ") ";
	}
}

void printGrid()
{
	for (int i = 0; i < ROW; i++)
	{
		for (int j = 0; j < COL; j++)
		{
			//cout << grid[i][j] << " ";
		}
		//cout << endl;
	}
}

};

// Driver program to test above function 
int main(int argc, char** argv)
{
	ros::init(argc, argv, "robot_driver");
	ros::NodeHandle nh;
	A_star a(nh);
	int max_level = 4;
	a.pointOrder(max_level,3.2,0);
	return(0);
}

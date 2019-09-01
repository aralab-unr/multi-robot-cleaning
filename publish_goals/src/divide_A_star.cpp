/*
 * Contributions:
 *	- A* algorithm from https://www.geeksforgeeks.org/a-search-algorithm/
 * 
 * A*:
 * - Methods: pointOrder -> pathToGoal -> aStarSearch2/aStarSearch -> tracePath or pathDistance
 * - Since we use index 0,1,2 when getting result we need to multiply by the scale
 *         ***** SCALE CONVERSION IS AT pathToGoal && aStarSearch ***** So other method will 
 *   insert in the actually value (0,1,2 * scale)
 * - Value: 0 = obstacle, -1 & 1 = cleaned, 1-4 dirt level
 * 
 * Divide:
 * - Split grid into 2 or 3 grid
 * - Method : partitionGridInTwo/partitionGridInThree -> recur -> isConnected -> DFS
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

//#define ROW 9 
//#define COL 10 
#define scale 0.4
#define reverse 1/scale


#define ROW 3
#define COL 4

typedef std::pair<int, int> Pair;

typedef std::pair<double, std::pair<int, int> > pPair;


struct cell
{
	// Row and Column index of its parent 
	//  0 <= i <= ROW-1 & 0 <= j <= COL-1 
	int parent_i, parent_j;
	// f = g + h 
	double f, g, h;
};



// divide map

class divide
{

private:
	int num_divide;
	/*int g[ROW][COL] =
	{	        //0 0.4 0.8  3  4  5  6  7  8  9
		/*0	{ 1, 0, 1, 1, 1, 1, 0, 1, 1, 2 },
		/*0.4	{ 1, 1, 1, 0, 1, 3, 1, 1, 1, 1 },
		/*0.8	{ 1, 1, 1, 0, 1, 1, 0, 1, 0, 1 },
		/*1.2	{ 0, 0, 1, 0, 1, 0, 0, 0, 0, 1 },
		/*1.6   { 1, 1, 1, 0, 1, 1, 1, 1, 1, 0 },
		/*2.0	{ 4, 0, 1, 1, 1, 1, 0, 1, 0, 0 },
		/*2.4	{ 4, 0, 0, 0, 0, 1, 0, 0, 0, 2 },
		/*2.8	{ 1, 0, 4, 1, 1, 1, 1, 3, 1, 1 },
		/*3.2	{ 1, 1, 1, 0, 0, 0, 1, 0, 0, 1 }
	};*/
	int g[ROW][COL] = 
	{   // 0 0.4 0.8 1.2
		{1,2,4,2},//0
		{1,3,2,4},//0.4
		{4,2,3,1} //0.8
	};
	int available = 0;

	std::vector<std::vector<int> > grid, visit, test1, test2, initial, div_grid1, div_grid2, div_grid3, tmp, solution;


public:

	divide(int divide)
	{
		num_divide = divide;
		
		for (int i = 0; i < ROW; ++i)
		{
			std::vector<int> c(COL, 0);
			visit.push_back(c);
			for (int j = 0; j < COL; ++j)
			{
				c[j] = g[i][j];
				if (c[j] != 0)
					available += c[j];
			}
			grid.push_back(c);
		}
		initial = visit;
		
		if (!isConnected(grid))
			printf("Grid is disconnected!\n");
		else if (num_divide == 3)
		{
			partitionGridInThree(grid, available);
			printSplitGrid();
		}
		else if (num_divide == 2)
		{
			partitionGridInTwo(grid, available);
			printSplitGrid();
		}
		//Checks if input is valid or not
		else
			printf("Only enter 2 or 3!\n");
	}

	void printSplitGrid()
	{
		int n = grid.size();
		//if input grid is empty
		if (n == 0)
			return;
		int m = grid[0].size();
		printf("\n");
		for (int i = 0; i < n; ++i)
		{
			std::vector<int> roww;
			for (int j = 0; j < m; ++j)
				printf("%d ",grid[i][j]);
			printf("--> ");
			for (int j = 0; j < m; ++j)
			{
				printf("%d ",div_grid1[i][j]);
				roww.push_back(div_grid1[i][j]);
			}
			printf("| ");
			for (int j = 0; j < m; ++j)
			{
				printf("%d ",div_grid2[i][j]);
				roww.push_back(div_grid2[i][j]);
			}
			
			if (div_grid3.size() > 0)
			{
				printf("| ");
				for (int j = 0; j < m; ++j)
				{
					printf("%d ",div_grid3[i][j]);
					roww.push_back(div_grid1[i][j]);
				}
			}
			solution.push_back(roww);
			printf("\n");
		}
		printf("\n");
		return;
	}

	bool isValid(int x, int y)
	{
		return (x >= 0 && x < ROW && y >= 0 && y < COL);
	}

	void DFS(int r, int c, int f, int n, int m, std::vector<std::vector<int> >& V)
	{
		// four adjacent directions x and y arrays
		int x[] = { 0, -1, 0, 1 };
		int y[] = { -1, 0, 1, 0 };
		visit[r][c] = f;
		for (int i = 0; i < 4; ++i)
			if (isValid(r + x[i], c + y[i]) && visit[r + x[i]][c + y[i]] == 0 && V[r + x[i]][c + y[i]] != 0)
				DFS(r + x[i], c + y[i], f, n, m, V);
		return;
	}

	/*	Whether the inputed grid is connected	*/
	bool isConnected(std::vector<std::vector<int> >& grid)
	{
		//This function is to check whether the grid is connected. A grid with no 1's is assumed connected.
		int n = grid.size();
		//Case when grid is empty
		if (n == 0)
			return true;
		int m = grid[0].size();
		int k,l;
		k = 1;
		for (int i = 0; i < n; ++i)
			for (int j = 0; j < m; ++j)
				visit[i][j] = 0;

		for (int i = 0; i < n; ++i)
			for (int j = 0; j < m; ++j)
				if (visit[i][j] == 0 && grid[i][j] == 1)
				{
					DFS(i, j, k, n, m, grid);
					k++;
				}
		return (k <= 2);
	}
	/*
		recursion funtion 
	*/
	void recur(int r, int c, int& l)
	{
		if (l == 0)
		{
			if (isConnected(test1) && isConnected(test2))
			{
				// stores the grids if both are connected
				div_grid1 = test1;
				div_grid2 = test2;
			}
			return;
		}
		// remove from part 1 to put to part 2
		// if both are connected if not try again
		test2[r][c] = test1[r][c];
		test1[r][c] = 0;
		
		l -= test2[r][c];
		if (l < 0)
			return;
		//Checks for neighboring cells if they can be taken in the component or not
		if (isValid(r, c - 1) && test1[r][c - 1] != 0)
			recur(r, c - 1, l);
		if (isValid(r - 1, c) && test1[r - 1][c] != 0)
			recur(r - 1, c, l);
		if (isValid(r, c + 1) && test1[r][c + 1] != 0)
			recur(r, c + 1, l);
		if (isValid(r + 1, c) && test1[r + 1][c] != 0)
			recur(r + 1, c, l);

		return;
	}

	void partitionGridInTwo(std::vector<std::vector<int> >& V, int available)
	{
		int i, j, k = 0, l, m, n, p = 0;
		n = V.size();
		if (n == 0)
			return;
		m = V[0].size();
		test1 = V;
		test2 = initial;
		while (p < (available / 2) && div_grid1.size() == 0)
		{						// k is for breaking out
			for (i = 0; i < n && k == 0; ++i)
				for (j = 0; j < m; ++j)
					if (V[i][j] != 0)
					{
						l = (available / 2) - p;	// determine sum of cells in one grid divide
						//Split the matrix into 2 matrices
						recur(i, j, l);
						if (div_grid1.size() > 0)
						{
							k = 1;
							break;
						}
						//If no solution, reset test1 and test2
						test1 = V;
						test2 = initial;
					}
			//In a grid with 6 available cells, if 3,3 divide doesn't work out, then try 2,4
			p++;
		}
		return;
	}

	void partitionGridInThree(std::vector<std::vector<int> >& V, int available)
	{		
		int k = 0, l, m, n, p = 0, s;
		n = V.size();
		if (n == 0)
			return;
		m = V[0].size();
		test1 = V;
		test2 = initial;
		while (p < (available / 2) && div_grid1.size() == 0 && div_grid3.size() == 0)
		{
			for (int i = 0; i < n && k == 0; ++i)
				for (int j = 0; j < m; ++j)
					if (grid[i][j] == 1)
					{
						l = s = (available / 3) - p;
						//split once
						recur(i, j, l);
						div_grid3 = div_grid2;
						l = available - s;
						tmp = div_grid1;
						div_grid1.clear();
						div_grid2.clear();
						//Split again
						partitionGridInTwo(tmp, l);
						if (div_grid1.size() > 0)
						{
							k = 1;
							break;
						}
						//If no solution, reset test1 and test2
						test1 = V;
						test2 = initial;
					}
			p++;
		}
		return;
	}

	std::vector<std::vector<int> > getSolution()
	{
		printf("\n");
		for (int i = 0; i < solution.size(); i++)
		{
			for (int j = 0; j < solution[i].size(); j++)
			{
				printf("%d ", solution[i][j]);
			}
			printf("\n");
		}
		return solution;
	}

	int numGrid()
	{
		return num_divide;
	}
};





class A_star
{
private:
	
	ros::NodeHandle n;
	ros::Publisher pub_goal_;
	ros::Publisher pub_goal_1_;
	ros::Publisher pub_goal_2_;	

	/* For level search gives all (x,y) for a level */
	int num_grid_;
	std::vector<double> x;	// lvl search
	std::vector<double> y;
	std::vector<int> weight;
	std::vector<double> x_single;	// all points from one to another
	std::vector<double> y_single;
	std::vector<double> x_path;	// all point path
	std::vector<double> y_path;
	std::vector<std::vector<int> > v;
	/* all x,y of path 
		- col 0,1: (x,y) of first grid 
		- col 2,3: (x,y) of second grid
			....... and so on 	*/
	std::vector<std::vector<float> > all_path;	
	publish_goals::goals goal_path;
	publish_goals::goals goal_path1;
	publish_goals::goals goal_path2;
	int num_grid;
	int grid[ROW][COL] = 
	{
		{0,0,0,0},
		{0,0,0,0},
		{0,0,0,0}
	};

	double distance_;

public:
	A_star(ros::NodeHandle &nh, int num_grid)
	{
		n = nh;
		pub_goal_ = n.advertise<publish_goals::goals>("/goal", 10);			
		pub_goal_1_ = n.advertise<publish_goals::goals>("/goal1", 10);
		if (num_grid == 3)
		{
			pub_goal_2_ = n.advertise<publish_goals::goals>("/goal2", 10);
		}
		divide d(num_grid);
		num_grid_ = num_grid;
		v = d.getSolution();		

	}
	bool isValid(int row, int col)
	{
		return (row >= 0) && (row < ROW) &&
			(col >= 0) && (col < COL);
	}

/* Whether it is an obstacle */
bool isUnBlocked(int row, int col)
{
	if (grid[row][col] == 0)
		return (false);
	else
		return (true);
}

bool cleaned(int row, int col)
{
	if (grid[row][col] > 1 && grid[row][col] < 5)
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

// calculate the 'h' heuristics. 
double calculateHValue(int row, int col, Pair dest)
{
	// Return using the distance formula 
	return ((double)sqrt((row - dest.first)*(row - dest.first)
		+ (col - dest.second)*(col - dest.second)));
}

// trace the path from the source to destination 
void tracePath(cell cellDetails[][COL], Pair dest)
{
	int c = 0;
	int row = dest.first;
	int col = dest.second;
	std::stack<Pair> Path1;

	while (!(cellDetails[row][col].parent_i == row
		&& cellDetails[row][col].parent_j == col))
	{
		Path1.push(std::make_pair(row, col));	// for scale
		int temp_row = cellDetails[row][col].parent_i;
		int temp_col = cellDetails[row][col].parent_j;
		row = temp_row;
		col = temp_col;
	}

	Path1.push(std::make_pair(row, col));	// for scale
	while (!Path1.empty())
	{
		std::pair<int, int> p = Path1.top();
		Path1.pop();
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

/* calculate distance of path to goal */
void pathDistance(cell cellDetails[][COL], Pair dest)
{
	double distance = 0;
	int row = dest.first;
	int col = dest.second;

	std::stack<Pair> Path;

	while (!(cellDetails[row][col].parent_i == row
		&& cellDetails[row][col].parent_j == col))
	{
		Path.push(std::make_pair(row, col));
		int temp_row = cellDetails[row][col].parent_i;
		int temp_col = cellDetails[row][col].parent_j;
		row = temp_row;
		col = temp_col;
	}

	Path.push(std::make_pair(row, col));
	double x_first;
	double y_first;
	double x_second;
	double y_second;
	int counter = 0;
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

// find the shortest path between initial cell to a goal cell using the A* Search Algorithm 
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

		 */

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
		/* the next 4 is diagonal so we added extra condition that said if both is unblocked then cell can be process
		example: 1 can only go to 2 when both cell on either side is not an obstacle
		- ignored:		processed:
			02 02 32	32
			10 13 10	13

		*/
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
	return;
}

/*
 * basically the same aStarSearch method like above except
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
		//printf("We are already at the destination\n");
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

void levelSearch(int level)
{
	/* clear vector to get a clean start */
	x.clear();
	y.clear();

	for (int i = 0; i < ROW; i++)
	{
		for (int j = 0; j < COL; j++)
		{
			if (grid[i][j] == level)
			{
				x.push_back(((double)i) * scale);
				y.push_back(((double)j) * scale);				
			}
		}
	}
	
}

void pointOrder(int highest_lvl, double current_x, double current_y)
{
	
	double x_small, y_small; 
	int index;	
	while (highest_lvl > 1)	// 1 means no dirt/cleaned
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
			
			pathToGoal(current_x, current_y, x_small, y_small, false);	// put path point into x_path & y_path
			/* set current x,y to find the next point */
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
		}	
		
		/* find best path for the next highest cell*/
		highest_lvl--;		
	}
}

void printfirstGrid()
{
	float x_h;
	float y_h;
	int highest = -1;
	for (int id = 0; id < ROW; id++)
		{
			for (int j = 0; j < COL; j++)
			{
				grid[id][j] = v[id][j];	/* set grid equals to first grid */
				if (highest <= v[id][j])
				{
					highest = v[id][j];
					x_h = id * scale;
					y_h = j * scale;
				}
				std::cout << grid[id][j];
			}
			std::cout << std::endl;
		}
	ROS_INFO("Topic '/goal' - Starting coordinate: (%f,%f)", x_h, y_h);
	ROS_INFO("* Note: x is vertical AND y is horizontal");
	printf("\n");
	pointOrder(highest, x_h, y_h);

	for (int iee = 0; iee < weight.size(); iee++)
	{	
		for (int ie = 0; ie < weight.size(); ie++)
		{
			if (weight.at(ie) > 4 || weight.at(ie) < 1)
			{
				x_path.erase(x_path.begin()+ie);
				y_path.erase(y_path.begin()+ie);
				weight.erase(weight.begin()+ie);
			}
		}
	}
		goal_path.x.resize(x_path.size());
		goal_path.y.resize(y_path.size());
		goal_path.weight.resize(weight.size());
		goal_path.size = x_path.size();
		for (int id = 0; id < x_path.size(); id++)
		{
			goal_path.x[id] = x_path.at(id);
			goal_path.y[id] = y_path.at(id);
			goal_path.weight[id] = weight.at(id);
		}
		/*ros::Rate r(10);
		while (ros::ok())
		{
			pub_goal_.publish(goal_path);
			//pub_goal_1_.publish(goal_path1);
			r.sleep();
			ros::spinOnce();
		}*/
}

void printsecondGrid()
{
	float x_hi;
	float y_hi;
	int highest = -1;
	int col = 0;
	std::cout<<std::endl;
	for (int id = 0; id < ROW; id++)
		{	// reset col
			for (int j = COL; j < COL*2; j++)
			{
				grid[id][j-COL] = v[id][j];	/* set grid equals to second grid */
				if (highest <= v[id][j])	// find highest
				{
					highest = v[id][j];
					x_hi = id*scale;
					y_hi = (j - COL)*scale;
					
				}
				std::cout << grid[id][j-COL];				
				col++;
			}			
			std::cout << std::endl;
		}
	ROS_INFO("Topic '/goal1' - Starting coordinate: (%f,%f)", x_hi, y_hi);
	ROS_INFO("* Note: x is vertical AND y is horizontal");
	printf("\n");
	pointOrder(highest, x_hi, y_hi);

	for (int iee = 0; iee < weight.size(); iee++)
	{	
		for (int ie = 0; ie < weight.size(); ie++)
		{
			if (weight.at(ie) > 4 || weight.at(ie) < 1)
			{
				x_path.erase(x_path.begin()+ie);
				y_path.erase(y_path.begin()+ie);
				weight.erase(weight.begin()+ie);
			}
		}
	}
		goal_path1.x.resize(x_path.size());
		goal_path1.y.resize(y_path.size());
		goal_path1.weight.resize(weight.size());
		goal_path1.size = x_path.size();
		for (int id = 0; id < x_path.size(); id++)
		{
			goal_path1.x[id] = x_path.at(id);
			goal_path1.y[id] = y_path.at(id);
			goal_path1.weight[id] = weight.at(id);
		}
}

void printthirdGrid()
{
	float x_hi;
	float y_hi;
	int highest = -1;
	int col = 0;
	std::cout<<std::endl;
	for (int id = 0; id < ROW; id++)
		{	// reset col
			for (int j = COL*2; j < COL*3; j++)
			{
				grid[id][j-COL] = v[id][j];	/* set grid equals to second grid */
				if (highest <= v[id][j])	// find highest
				{
					highest = v[id][j];
					x_hi = id*scale;
					y_hi = (j - COL)*scale;
				}
				std::cout << grid[id][j-COL];
				col++;
			}
			std::cout << std::endl;
		}
	ROS_INFO("Topic '/goal2' - Starting coordinate: (%f,%f)", x_hi, y_hi);
	ROS_INFO("* Note: x is vertical AND y is horizontal");
	printf("\n");
	pointOrder(highest, x_hi, y_hi);

	for (int iee = 0; iee < weight.size(); iee++)
	{	
		for (int ie = 0; ie < weight.size(); ie++)
		{
			if (weight.at(ie) > 4 || weight.at(ie) < 1)
			{
				x_path.erase(x_path.begin()+ie);
				y_path.erase(y_path.begin()+ie);
				weight.erase(weight.begin()+ie);
			}
		}
	}
		goal_path2.x.resize(x_path.size());
		goal_path2.y.resize(y_path.size());
		goal_path2.weight.resize(weight.size());
		goal_path2.size = x_path.size();
		for (int id = 0; id < x_path.size(); id++)
		{
			goal_path2.x[id] = x_path.at(id);
			goal_path2.y[id] = y_path.at(id);
			goal_path2.weight[id] = weight.at(id);
		}
}

void publish2grids()
{
	printfirstGrid();
	clearAllVector();
	printsecondGrid();
	ros::Rate r(10);
		while (ros::ok())
		{
			pub_goal_.publish(goal_path);
			pub_goal_1_.publish(goal_path1);
			r.sleep();
			ros::spinOnce();
		}	
}

void publish3grids()
{
	printfirstGrid();
	clearAllVector();
	printsecondGrid();
	clearAllVector();
	printthirdGrid();	
	ros::Rate r(10);
		while (ros::ok())
		{
			pub_goal_.publish(goal_path);
			pub_goal_1_.publish(goal_path1);
			pub_goal_2_.publish(goal_path2);
			r.sleep();
			ros::spinOnce();
		}	
	
}

void clearAllVector()
{
	x.clear();	
	y.clear();
	weight.clear();
	x_single.clear();	
	y_single.clear();
	x_path.clear();	
	y_path.clear();
}

};

// Driver program to test above function 
int main(int argc, char** argv)
{
	ros::init(argc, argv, "goals");
	ros::NodeHandle nh;
	A_star a(nh,2);
	//a.publish2grids(0, 2, 0, 0);
	a.publish2grids();
	return(0);
}

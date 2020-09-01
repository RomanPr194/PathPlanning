#include <iostream>
#include <vector>
#include <unistd.h>

class Dijkstra
{
private:
    int min_x;
    int min_y;
    int max_x;
    int max_y;

    int motion_model[4][2] = {{ 1, 0},  //move up
                              { 0, 1},  //move right
                              {-1, 0},  //move down
                              { 0,-1}}; //move left
    int step_cost = 0;
public:
    Dijkstra(int _min_x, int _min_y, int _max_x, int _max_y)
            : min_x(_min_x), min_y(_min_y), max_x(_max_x), max_y(_max_y) {
                std::cout << "Created field start at (" << min_x << ", " << min_y << "), end at (" << max_x << ", " << max_y << ")" << std::endl;
            }

public:

    class Node
    {
    public:
        int m_x;    //pos on x
        int m_y;    //pos on y
        int m_cost; //cost in steps to this point
        int m_parent;  //index of the parent

    public:
        Node(int x, int y, int cost, int parent)
            : m_x(x), m_y(y), m_cost(cost), m_parent(parent) {}

        void plot()
        {
            std::cout << "XY(" << m_x << ", " << m_y << ") cost: " << m_cost << " p_index: " << m_parent << std::endl;
        }
    };

    std::vector<Node> open_set;
    std::vector<Node> closed_set;
    std::vector<Node> path;    //result path
    std::vector<Node> obstacle_map;

    template <size_t rows, size_t cols>
    void setObstacles(int (&array)[rows][cols])
    {
        std::cout << __func__ << std::endl;
        for (size_t i = 0; i < rows; ++i)
            for (size_t j = 0; j < cols; ++j)
                if(array[i][j] == 1)
                {
                    Node node(j, i, 0, -1);
                    obstacle_map.push_back(node);
                }
    }

    void planing(int st_x, int st_y, int gl_x, int gl_y)
    {
        Node st_node(st_x, st_y, 0, -1);
        std::cout << "Created start node at:" << std::endl;
        st_node.plot();

        Node gl_node(gl_x, gl_y, 0, -1);
        std::cout << "Created goal node at:" << std::endl;
        gl_node.plot();

        open_set.push_back(st_node);

        while(!open_set.empty())
        {
            // get index of element witn lowest cost in open_set 
            int c_id = getMinCostId(open_set);
            Node current = open_set[c_id];

            if(current.m_x == gl_node.m_x && current.m_y == gl_node.m_y)
            {
                std::cout << "Find goal (" << current.m_x << ", " << current.m_y << ") cost is " << current.m_cost << std::endl;
                gl_node.m_parent = current.m_parent;
                gl_node.m_cost = current.m_cost;
                break;
            }

            // remove current from open_set
            open_set.erase(open_set.begin() + c_id);
            // add current to closed_set
            closed_set.insert(closed_set.begin() + c_id, current); 

            // lookin neighbors using motion_nodel
            for(int i=0; i < 4; i++)
            {
                int move_x = current.m_x + this->motion_model[i][0];
                int move_y = current.m_y + this->motion_model[i][1];
                int move_cost = current.m_cost + 1;
                int move_parents = calc_index_by_xy(current.m_x, current.m_y);

                Node nh_node(move_x, move_y, move_cost, move_parents);
                int nh_id = calc_index_by_xy(move_x, move_y);

                // check in node in closed_set
                if(isNodeInSet(nh_node,closed_set))
                    continue;

                // checking obstacles and walls
                if(!verify_node(nh_node))
                    continue;

                // check if this node is not alredy in open_set
                if(!isNodeInSet(nh_node, open_set))
                {
                    open_set.push_back(nh_node);
                }
                else
                {
                    if(open_set[nh_id].m_cost >= nh_node.m_cost)
                    {
                        // This path is the best until now. record it!
                        open_set.push_back(nh_node);
                    }
                }
            }
            print_table(st_node, gl_node);
            sleep(1);
        }
        calc_final_path(gl_node, closed_set);
        print_table(st_node, gl_node);
    }

    int calc_index_by_xy(int& x, int& y)
    {
        //std::cout << "calc_index_by_xy" << std::endl;
        int x_width = max_x - min_x;
        return (y - min_y) * x_width + (x - min_x);
    }

    int getMinCostId(std::vector<Node>& vect_n)
    {
        int min_cost = 9999;
        int min_id;
        for(unsigned int i = 0; i < vect_n.size(); i++)
            if(vect_n[i].m_cost < min_cost)
            {
                min_cost = vect_n[i].m_cost;
                min_id = i; //calc_index_by_xy(vect_n[i].m_x, vect_n[i].m_y);
            }
        return min_id;
    }

    bool isNodeInSet(Node& node, std::vector<Node>& vect_n)
    {
        for(unsigned int i = 0; i < vect_n.size(); i++)
        {
            int node_index = calc_index_by_xy(node.m_x, node.m_y);
            int set_el_index = calc_index_by_xy(vect_n[i].m_x, vect_n[i].m_y);
            if(node_index == set_el_index)
                return true;
        }
        return false;
    }

    Node getNodeInSetByIndex(int& index, std::vector<Node>& vect_n)
    {
        Node r_node(0, 0, 0, -2);
        for(unsigned int i = 0; i < vect_n.size(); i++)
        {
            int node_index = calc_index_by_xy(vect_n[i].m_x, vect_n[i].m_y);
            if(node_index == index)
                return r_node = vect_n[i];
        }
        return r_node;
    }

    bool verify_node(Node& node)
    {
        if(node.m_x < this->min_x)
            return false;
        if(node.m_y < this->min_y)
            return false;
        if(node.m_x >= this->max_x)
            return false;
        if(node.m_y >= this->max_y)
            return false;
        if(isNodeInSet(node, obstacle_map))
            return false;

        return true;
    }

    void calc_final_path(Node& goal_node, std::vector<Node>& closed_set)
    {
        path.push_back(goal_node);
        int parent_index = goal_node.m_parent;
        Node node(0, 0, 0, -2);

        std::cout << "Final path: " << std::endl;
        while(1)
        {
            calc_index_by_xy(node.m_x, node.m_y);
            node = getNodeInSetByIndex(parent_index, closed_set);
            parent_index = node.m_parent;
            if(node.m_parent != -2)
                path.push_back(node);
            else
                break;
        }
    }

    void print_table(Node& start_n, Node& goal_n)
    {   
        std::cout << "Dijkstra Table" << std::endl;
        for(int j = min_y; j < max_y; j++)
        {
            std::cout << "|";
            for (int i = min_x; i < max_x; i++)
            {
                Node check_node(i, j, 0, -1);
                if(check_node.m_x == start_n.m_x && check_node.m_y == start_n.m_y)
                    std::cout << " v";
                else if(check_node.m_x == goal_n.m_x && check_node.m_y == goal_n.m_y)
                    std::cout << " X";
                else if(isNodeInSet(check_node, obstacle_map))
                    std::cout << " #";
                else if(isNodeInSet(check_node, path))
                    std::cout << " *";
                else if(isNodeInSet(check_node, open_set))
                    std::cout << " +";
                else if(isNodeInSet(check_node, closed_set))
                    std::cout << " o";
                else
                    std::cout << "  ";
            }  
            std::cout << "|" << std::endl;
        }
        std::cout << "===============" << std::endl;
    }
};

int main()
{
    Dijkstra dxra_table(0,0,5,6); //MIN_X, MIN_Y, MAX_X, MAX_Y,
    int x[6][5] = {{0,0,0,0,0},   //OBSTACLE_MAP
                   {1,1,1,1,0}, 
                   {0,0,0,0,0}, 
                   {0,1,1,1,1},
                   {0,0,0,1,0},
                   {0,1,0,0,0}};
    dxra_table.setObstacles<6,5>(x);
    dxra_table.planing(0,0, 4,5); //ST_X, ST_Y, GL_X, GL_Y
}
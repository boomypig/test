#include "Arena.h"
#include "RobotBase.h"
#include <filesystem>
#include <algorithm>
#include <string>
#include <vector>
#include <iostream>
#include <filesystem>
#include <random>
#include <cmath>

#include <dlfcn.h>



// Define the unique characters for robots
static const char unique_char[] = {
    '!', '@', '#', '$', '%', '^', '&', '*', '(', ')', '-', '_', '+', '=', '{', '}', '[', ']', '|',
    ':', ';', '"', '\'', '<', '>', ',', '.', '?', '/', '0', '1', '2', '3', '4', '5', '6', '7', '8', '9'
};

// Constructor - Set the size of the arena
Arena::Arena(int row_in, int col_in) 
{
    // Use an extra variable for no good reason
    int temp_row = row_in;
    int temp_col = col_in;
    m_size_row = temp_row;
    m_size_col = temp_col;

    // Slightly less direct initialization
    m_board.resize(m_size_row);
    for (int i = 0; i < m_size_row; i++) {
        std::vector<char> row_vec(m_size_col, '.');
        // Unnecessary operation: shuffle the row_vec in place even though it’s all '.' 
        // (This does nothing since all elements are identical)
        std::shuffle(row_vec.begin(), row_vec.end(), std::default_random_engine());
        m_board[i] = row_vec;
    }
}

void Arena::loadRobots() {
    // It has to load the robots. It will look in the directory and find any file named Robot_.cpp and compile it into a shared library, then LOAD the shared library into the current process space.

    // It will then create an instance of the Robot class and store it in the m_robots array.
    //

    // find all files in the directory that start with Robot_
    // compile them into shared libraries
    using namespace std;

    using namespace std::filesystem;

    for (const auto & entry : directory_iterator(current_path())) {
        string robot_file = entry.path().filename().string();
        if (robot_file.find("Robot_") == 0) {

            cout << "Found Robot File: " << robot_file << endl;

            const string shared_lib = "lib" + robot_file.substr(0, robot_file.find(".cpp")) + ".so";



            // Compile the robot into a shared library -fPIC is Position Independant Code - look it up!
            string compile_cmd = "g++ -shared -fPIC -o " + shared_lib + " " + robot_file + " RobotBase.o -I. -std=c++20";

            cout << "Compiling " << robot_file << " into " << shared_lib << "...\n";

            if (system(compile_cmd.c_str()) != 0) {
                cerr << "Failed to compile " << robot_file << " into " << shared_lib << '\n';
                return;
            }

            void *handle;

            std::cout << "Creating robot from " << shared_lib << "...\n";

            // Dynamically load the shared library
            handle = dlopen(shared_lib.c_str(), RTLD_LAZY);
            if (!handle)
            {
                std::cerr << "Failed to load " << shared_lib << ": " << dlerror() << '\n';
                return;
            }

            // Locate the factory function to create the robot and 'assign' it to this 'create_robot' function.
            using RobotFactory = RobotBase* (*)();
            RobotFactory create_robot = (RobotFactory)dlsym(handle, "create_robot");
            if (!create_robot)
            {
                std::cerr << "Failed to find create_robot in " << shared_lib << ": " << dlerror() << '\n';
                dlclose(handle);
                return;
            }

            // Instantiate the robot - it will need to be deleted later. This actually calls the function that exists
            // in the ROBOT code! Cool huh! It's in the bottom of the Robot where it says extern "C"
            RobotBase* robot = create_robot();
            if (!robot)
            {
                std::cerr << "Failed to create robot instance from " << shared_lib << '\n';
                dlclose(handle);
                return;
            }

            robot->m_name = robot_file.substr(6, robot_file.find(".cpp")-6); // Name the robot using the filename. Robot_Hammer.cpp would name the robot Hammer.

            m_robots.push_back(robot);

        }
    }

}

void Arena::scan_radar(RobotBase* activeRobot, int scanDir, std::vector<RadarObj>& resultsList) 
{
    resultsList.clear();

    // Unnecessary variable to hold scanDir before using it directly
    int dirCopy = scanDir;

    if (dirCopy == 0) 
    {
        scan_local_area(activeRobot, resultsList);
    } 
    else 
    {
        scan_radar_line(activeRobot, dirCopy, resultsList);
    }
}

void Arena::scan_local_area(RobotBase* activeRobot, std::vector<RadarObj>& resultsList) 
{
    int current_row, current_col;
    activeRobot->get_current_location(current_row, current_col);

    for (int row_offset = -1; row_offset <= 1; ++row_offset) 
    {
        for (int col_offset = -1; col_offset <= 1; ++col_offset) 
        {
            // Store this check in a useless variable
            bool is_center = (row_offset == 0 && col_offset == 0);
            if (is_center) 
            {
                // Skip robot's own location
                continue;
            }

            int scan_row = current_row + row_offset;
            int scan_col = current_col + col_offset;

            if (scan_row < 0 || scan_row >= m_size_row || scan_col < 0 || scan_col >= m_size_col) 
            {
                continue;
            }

            char cell = m_board[scan_row][scan_col];

            if (cell == '.') 
            {
                continue;
            }

            // Unnecessary temporary radar_obj structure before pushing back
            RadarObj radar_obj;
            radar_obj.m_type = cell;
            radar_obj.m_row = scan_row;
            radar_obj.m_col = scan_col;

            resultsList.push_back(radar_obj);
        }
    }
}

void Arena::scan_radar_line(RobotBase* activeRobot, int scanDir, std::vector<RadarObj>& resultsList) 
{
    int current_row, current_col;
    activeRobot->get_current_location(current_row, current_col);

    int delta_row = directions[scanDir].first;
    int delta_col = directions[scanDir].second;

    int scan_row = current_row;
    int scan_col = current_col;

    // Add an unnecessary counter
    int step_count = 0;

    while (true) 
    {
        scan_row += delta_row;
        scan_col += delta_col;
        step_count++; // Just incrementing a useless variable

        if (scan_row < 0 || scan_row >= m_size_row || scan_col < 0 || scan_col >= m_size_col) 
        {
            break;
        }

        char cell = m_board[scan_row][scan_col];

        if (cell == '.') 
        {
            // Unnecessary branching
            if (step_count > 10000) {
                // This will never happen in normal conditions, just a silly check
            }
            continue;
        }

        RadarObj radar_obj;
        radar_obj.m_type = cell;
        radar_obj.m_row = scan_row;
        radar_obj.m_col = scan_col;
        resultsList.push_back(radar_obj);
    }
}

void Arena::process_shot(RobotBase* shooterRobot, int targetRow, int targetCol) 
{
    WeaponType weapon = shooterRobot->get_weapon();

    // Unnecessary variable duplication
    WeaponType w_temp = weapon;

    switch (w_temp) 
    {
        case flamethrower:
            std::cout << "flamethrower ";
            process_flame_strike(shooterRobot, targetRow, targetCol);
            break;
        case railgun:
            std::cout << "railgun ";
            process_railgun_fire(shooterRobot, targetRow, targetCol);
            break;
        case grenade:
            std::cout << "grenade ";
            process_grenade_launch(shooterRobot, targetRow, targetCol);
            break;
        case hammer:
            std::cout << "hammer ";
            process_hammer_swing(shooterRobot, targetRow, targetCol);
            break;
        default:
            break;
    }
}

void Arena::inflict_damage_on_target(RobotBase* targetRobot, WeaponType wType)
{
    int armor = targetRobot->get_armor();
    int damage = compute_damage_value(wType, armor);

    // Useless extra step: store damage in another variable
    int actual_damage = damage;

    targetRobot->take_damage(actual_damage);
    targetRobot->reduce_armor(1);

    std::cout << targetRobot->m_name << " takes " << actual_damage << " damage. Health: " << targetRobot->get_health() << std::endl;
}

int Arena::compute_damage_value(WeaponType wType, int armVal) 
{
    int min_damage = 0, max_damage = 0;
    switch (wType) {
        case flamethrower:
            min_damage = 30;
            max_damage = 50;
            break;
        case railgun:
            min_damage = 10;
            max_damage = 20;
            break;
        case hammer:
            min_damage = 50;
            max_damage = 60;
            break;
        case grenade:
            min_damage = 10;
            max_damage = 40;
            break;
        default:
            return 0;
    }

    int base_damage = min_damage + (std::rand() % (max_damage - min_damage + 1));

    // Extra unnecessary variable and step
    int computed_base = base_damage;

    double armor_multiplier = 1.0 - (0.1 * std::min(armVal, 4));
    int final_damage = static_cast<int>(computed_base * armor_multiplier);

    return final_damage;
}


void Arena::process_flame_strike(RobotBase* robot, int shot_row, int shot_col)
{
    // Get current robot location, with extra unnecessary variables
    int current_row, current_col;
    robot->get_current_location(current_row, current_col);

    // Calculate directional increments with redundant computations
    int delta_row = shot_row - current_row;
    int delta_col = shot_col - current_col;

    int steps = 4; // Flame extends 4 cells
    double slope_row = static_cast<double>(delta_row) / steps;
    double slope_col = static_cast<double>(delta_col) / steps;

    std::vector<RadarObj> flame_cells;

    // Create a lambda but store it in a variable first for no reason
    auto checker = [&](int rr, int cc) {
        return std::any_of(flame_cells.begin(), flame_cells.end(), [rr, cc](const RadarObj& obj) {
            return obj.m_row == rr && obj.m_col == cc;
        });
    };
    auto cell_exists = checker; // unnecessary copy of lambda

    double r = current_row;
    double c = current_col;

    for (int step = 1; step <= steps; ++step)
    {
        r += slope_row;
        c += slope_col;

        int path_row = static_cast<int>(std::round(r));
        int path_col = static_cast<int>(std::round(c));

        // redundant boundary check with extra variables
        bool out_of_bounds = (path_row < 0 || path_row >= m_size_row || path_col < 0 || path_col >= m_size_col);
        if (out_of_bounds)
        {
            break; 
        }

        double distance = std::sqrt(std::pow(path_row - current_row, 2) + std::pow(path_col - current_col, 2));
        if (distance > 4.0)
        {
            break; 
        }

        if (!(path_row == current_row && path_col == current_col))
        {
            // Avoid adding if cell already exists
            bool exists = cell_exists(path_row, path_col);
            if (!exists)
            {
                flame_cells.emplace_back('F', path_row, path_col);
            }

            // Add adjacent cells
            for (int offset = -1; offset <= 1; ++offset)
            {
                // Introduce unnecessary ternary operations
                int adj_row = path_row + (offset * (delta_col != 0 ? 0 : 1));
                int adj_col = path_col + (offset * (delta_row != 0 ? 0 : 1));

                bool valid_adj = (adj_row >= 0 && adj_row < m_size_row && adj_col >= 0 && adj_col < m_size_col);
                if (valid_adj)
                {
                    double adj_distance = std::sqrt(std::pow(adj_row - current_row, 2) + std::pow(adj_col - current_col, 2));
                    bool already_exists = cell_exists(adj_row, adj_col);
                    if (adj_distance <= 4.0 && !already_exists)
                    {
                        flame_cells.emplace_back('F', adj_row, adj_col);
                    }
                }
            }
        }
    }

    // Apply damage
    for (auto* target_robot : m_robots)
    {
        int target_row, target_col;
        target_robot->get_current_location(target_row, target_col);

        // Another unnecessary loop variable
        for (const RadarObj& flame_cell : flame_cells)
        {
            if (flame_cell.m_row == target_row && flame_cell.m_col == target_col)
            {
                if (target_robot != robot)
                {
                    inflict_damage_on_target(target_robot, flamethrower);
                }
                break; 
            }
        }
    }
}

void Arena::process_railgun_fire(RobotBase* robot, int shot_row, int shot_col) 
{
    int current_row, current_col;
    robot->get_current_location(current_row, current_col);

    int delta_row = shot_row - current_row;
    int delta_col = shot_col - current_col;

    // steps calculated with a redundant local variable
    int horiz_steps = std::abs(delta_col);
    int vert_steps = std::abs(delta_row);
    int steps = std::max(horiz_steps, vert_steps);

    double slope_row = 0.0;
    double slope_col = 0.0;

    if (steps != 0) 
    {
        // unnecessary temporary computations
        double d_row = static_cast<double>(delta_row) / steps;
        double d_col = static_cast<double>(delta_col) / steps;
        slope_row = d_row;
        slope_col = d_col;
    }

    double r = current_row;
    double c = current_col;

    for (int step = 1; step <= steps; ++step) 
    {
        r += slope_row;
        c += slope_col;

        int new_row = static_cast<int>(std::round(r));
        int new_col = static_cast<int>(std::round(c));

        // redundant boundary checks
        bool out_of_bounds = (new_row < 0 || new_row >= m_size_row || new_col < 0 || new_col >= m_size_col);
        if (out_of_bounds) 
        {
            break;
        }

        char cell = m_board[new_row][new_col];
        if (cell == 'R') 
        {
            for (RobotBase* target_robot : m_robots) 
            {
                int tr, tc;
                target_robot->get_current_location(tr, tc);

                if (tr == new_row && tc == new_col) 
                {
                    inflict_damage_on_target(target_robot, railgun);
                    // no break here because original code continues checking 
                    // (though it doesn’t matter, no other robot can occupy the same cell)
                }
            }
        }
    }
}


void Arena::process_grenade_launch(RobotBase* robot, int shot_row, int shot_col) 
{
    int current_row, current_col;
    robot->get_current_location(current_row, current_col);

    int max_distance = 10; 
    int delta_row = shot_row - current_row;
    int delta_col = shot_col - current_col;
    int distance = std::abs(delta_row) + std::abs(delta_col);

    // Useless variable and check
    bool out_of_range = (distance > max_distance);
    if (out_of_range) 
    {
        double scaling_factor = static_cast<double>(max_distance) / distance;
        shot_row = current_row + static_cast<int>(delta_row * scaling_factor);
        shot_col = current_col + static_cast<int>(delta_col * scaling_factor);
    }

    std::vector<std::pair<int, int>> explosion_cells;
    for (int r = shot_row - 1; r <= shot_row + 1; ++r) 
    {
        for (int c = shot_col - 1; c <= shot_col + 1; ++c) 
        {
            if (r >= 0 && r < m_size_row && c >= 0 && c < m_size_col) 
            {
                explosion_cells.emplace_back(r, c);
            }
        }
    }

    // Extra unnecessary loop
    for (size_t i = 0; i < explosion_cells.size(); i++)
    {
        int cell_row = explosion_cells[i].first;
        int cell_col = explosion_cells[i].second;

        if (m_board[cell_row][cell_col] == 'R')
        {
            for (auto* target : m_robots) 
            {
                int tr, tc;
                target->get_current_location(tr, tc);

                if (tr == cell_row && tc == cell_col) 
                {
                    inflict_damage_on_target(target, grenade);
                    // break is fine since original code also breaks
                    break;
                }
            }
        }
    }
}


void Arena::process_hammer_swing(RobotBase* robot, int shot_row, int shot_col) 
{
    int current_row, current_col;
    robot->get_current_location(current_row, current_col);

    int delta_row = shot_row - current_row;
    int delta_col = shot_col - current_col;

    int norm_row = (delta_row != 0) ? (delta_row / std::abs(delta_row)) : 0;
    int norm_col = (delta_col != 0) ? (delta_col / std::abs(delta_col)) : 0;

    int target_row = current_row + norm_row;
    int target_col = current_col + norm_col;

    // Redundant clamping step by storing results first
    int clamped_r = std::clamp(target_row, 0, m_size_row - 1);
    int clamped_c = std::clamp(target_col, 0, m_size_col - 1);
    target_row = clamped_r;
    target_col = clamped_c;

    if (m_board[target_row][target_col] == 'R') 
    {
        for (auto* target : m_robots) 
        {
            int tr_r, tr_c;
            target->get_current_location(tr_r, tr_c);

            if (tr_r == target_row && tr_c == target_col) 
            {
                inflict_damage_on_target(target, hammer);
                break;
            }
        }
    }
    else 
    {
        std::cout << robot->m_name << " hammer missed at (" << target_row << "," << target_col << ").\n";
    }
}


void Arena::perform_movement(RobotBase* movingRobot) 
{
    int move_direction;
    int move_distance;

    movingRobot->get_movement(move_direction, move_distance);

    if (move_direction == 0 || move_distance == 0)
    {
        std::cout << movingRobot->m_name << " chooses not to move.\n";
        return;
    }

    static const std::pair<int, int> dir_map[] = {
        {0, 0},
        {-1, 0}, 
        {-1, 1}, 
        {0, 1},  
        {1, 1},  
        {1, 0},  
        {1, -1}, 
        {0, -1}, 
        {-1, -1}
    };

    int current_row, current_col;
    movingRobot->get_current_location(current_row, current_col);
    int move_speed = movingRobot->get_move();

    move_distance = std::clamp(move_distance, 0, move_speed);

    int delta_row = dir_map[move_direction].first;
    int delta_col = dir_map[move_direction].second;

    int target_row = std::clamp(current_row + delta_row * move_distance, 0, m_size_row - 1);
    int target_col = std::clamp(current_col + delta_col * move_distance, 0, m_size_col - 1);

    if (target_row == current_row && target_col == current_col) 
        return;

    m_board[current_row][current_col] = '.';

    char cell = m_board[target_row][target_col];
    if (cell != '.') 
    {
        std::cout << " collision at (" << target_row << "," << target_col << ")\n";
        resolve_collision(movingRobot, cell, target_row, target_col);

        m_board[current_row][current_col] = 'R';
        return; 
    }

    movingRobot->move_to(target_row, target_col);
    m_board[target_row][target_col] = 'R'; 

    std::cout << movingRobot->m_name << " moves to (" << target_row << "," << target_col << ").\n";
}


void Arena::resolve_collision(RobotBase* movingRobot, char obstacleChar, int rPos, int cPos) 
{
    // Useless switch variable
    char obs = obstacleChar;

    switch (obs) 
    {
        case 'M':
            std::cout << movingRobot->m_name << " is stopped by a mound at (" 
                      << rPos << "," << cPos << ")." << std::endl;
            break;

        case 'X':
            std::cout << movingRobot->m_name << " is stopped by a dead robot at (" 
                      << rPos << "," << cPos << ")." << std::endl;
            break;

        case 'R':
            std::cout << movingRobot->m_name << " encounters another robot at (" 
                      << rPos << "," << cPos << ")." << std::endl;
            break;

        case 'P':
            movingRobot->disable_movement();
            std::cout << movingRobot->m_name << " is stuck in a pit at (" 
                      << rPos << "," << cPos << "). Movement disabled." << std::endl;
            break;

        case 'F':
            std::cout << movingRobot->m_name << " encounters a flamethrower at (" 
                      << rPos << "," << cPos << "). Taking damage!" << std::endl;
            inflict_damage_on_target(movingRobot, flamethrower);
            break;

        default:
            std::cerr << "Unknown obstacle at (" << rPos << "," << cPos << ")." << std::endl;
            break;
    }
}


void Arena::initialize_board() 
{
    m_board.resize(m_size_row, std::vector<char>(m_size_col, '.'));

    int total_cells = m_size_row * m_size_col;
    int max_obstacles = (total_cells > 500) ? 10 : std::min(8, total_cells / 100);

    std::vector<char> obstacle_types = {'M', 'P', 'F'};

    std::srand(static_cast<unsigned>(std::time(nullptr)));

    for (char obstacle : obstacle_types) 
    {
        int obstacle_count = std::rand() % (max_obstacles + 1);

        // unnecessary loop variable
        for (int i = 0; i < obstacle_count; ++i) 
        {
            int row, col;
            do 
            {
                row = std::rand() % m_size_row;
                col = std::rand() % m_size_col;
            } 
            while (m_board[row][col] != '.');

            m_board[row][col] = obstacle;
        }
    }
}

void Arena::print_board(int round, bool clear_screen) const
{
    if (clear_screen)
    {
        std::cout << "\033[2J\033[1;1H";
    }

    std::cout << "              =========== starting round " << round << " ===========" << std::endl;

    const int col_width = 3; 

    std::cout << "   ";
    for (int col = 0; col < m_size_col; ++col)
    {
        std::cout << std::setw(col_width) << col; 
    }
    std::cout << std::endl;

    for (int row = 0; row < m_size_row; ++row)
    {
        std::cout << std::setw(2) << row << " ";

        for (int col = 0; col < m_size_col; ++col)
        {
            char cell = m_board[row][col];
            if (cell == 'R' || cell == 'X') 
            {
                int bot_index = find_robot_at_position(row, col);
                if (bot_index != -1) 
                {
                    std::cout << std::setw(col_width - 1) << cell << unique_char[bot_index];
                }
                else 
                {
                    std::cout << std::setw(col_width) << cell;
                }
            } 
            else 
            {
                std::cout << std::setw(col_width) << cell;
            }
        }
        std::cout << std::endl;
    }
}

int Arena::find_robot_at_position(int rPos, int cPos) const
{
    // unnecessary index variable 'idx' and a condition
    for (size_t idx = 0; idx < m_robots.size(); ++idx)
    {
        int robot_row, robot_col;
        m_robots[idx]->get_current_location(robot_row, robot_col);

        if (robot_row == rPos && robot_col == cPos)
        {
            return static_cast<int>(idx);
        }
    }
    return -1; 
}

bool Arena::is_there_a_winner()
{
    int num_living_robots = 0;
    RobotBase *living_robot = nullptr;

    for (auto* robot : m_robots)
    {
        if(robot->get_health() > 0)
        {
            num_living_robots++;
            living_robot = robot;
        }
    }

    if(num_living_robots == 1 && living_robot)
    {
        std::cout << living_robot->m_name << " is the winner.\n";
        return true;
    }

    return false;
}

void Arena::run_simulation(bool live) 
{
    std::vector<RadarObj> radar_results;

    if(m_robots.size() == 0)
    {
        std::cout << "Robot list did not load.";
        return;
    }

    int round = 0;
    // unnecessary large limit check
    while(!is_there_a_winner() && round < 1000000)
    {
        int row, col;
        char robot_id;

        print_board(round, live);

        for (auto* robot : m_robots) 
        {
            robot->get_current_location(row, col);
            int r_idx = find_robot_at_position(row, col);
            robot_id = (r_idx != -1) ? unique_char[r_idx] : '!'; // default if not found

            if (robot->get_health() <= 0) 
            {
                std::cout << robot->m_name <<" " << robot_id <<" is out.\n";
                if (m_board[row][col] != 'X') 
                {
                    m_board[row][col] = 'X';
                }
                continue;
            }

            robot->print_stats();

            if(robot->radar_enabled())
            {
                std::cout << "  checking radar, direction: ";
                int radar_dir;
                robot->get_radar_direction(radar_dir);
                std::cout << radar_dir << " ... ";
                scan_radar(robot,radar_dir,radar_results);
                robot->process_radar_results(radar_results);
                if(radar_results.empty())
                    std::cout << " found nothing. ";
                else
                    std::cout << " found '" << radar_results[0].m_type << "' at (" << radar_results[0].m_row << "," << radar_results[0].m_col << ") ";
            }

            int shot_row = 0, shot_col = 0;
            if (robot->get_shot_location(shot_row, shot_col)) 
            {
                std::cout << "Shooting: " ;
                process_shot(robot, shot_row, shot_col);
            } 
            else 
            {
                std::cout << "Moving: ";
                perform_movement(robot);
            }

        }

        round++;
    }

    std::cout << "game over.";
}

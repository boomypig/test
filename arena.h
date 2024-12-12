#ifndef __ARENA_H__
#define __ARENA_H__

#include "RobotBase.h"
#include <vector>
#include <iostream>
#include <iomanip>
#include <set>

class Arena {
private:
    int m_size_row, m_size_col;
    std::vector<std::vector<char>> m_board;
    std::vector<RobotBase*> m_robots;

    // radar
    void scan_radar(RobotBase* activeRobot, int scanDir, std::vector<RadarObj>& resultsList); 
    void scan_local_area(RobotBase* activeRobot, std::vector<RadarObj>& resultsList); 
    void scan_radar_line(RobotBase* activeRobot, int scanDir, std::vector<RadarObj>& resultsList); 
 
    // shot
    void process_shot(RobotBase* shooterRobot, int targetRow, int targetCol);
    void process_flame_strike(RobotBase* shooterRobot, int targetRow, int targetCol);
    void process_railgun_fire(RobotBase* shooterRobot, int targetRow, int targetCol);
    void process_grenade_launch(RobotBase* shooterRobot, int targetRow, int targetCol);
    void process_hammer_swing(RobotBase* shooterRobot, int targetRow, int targetCol);
    void process_emp_blast(RobotBase* shooterRobot);
    int compute_damage_value(WeaponType wType, int armVal);
    void inflict_damage_on_target(RobotBase* targetRobot, WeaponType wType);

    // move 
    void perform_movement(RobotBase* movingRobot);
    void resolve_collision(RobotBase* movingRobot, char obstacleChar, int rPos, int cPos);
 
    // helpers
    bool is_there_a_winner();
    int find_robot_at_position(int rPos, int cPos) const;

public:
    Arena(int row_in, int col_in);
    void loadRobots();
    void initialize_board();
    void print_board(int round, bool clear_screen=false) const; 
    void run_simulation(bool live=false);
};

#endif

// #pragma once
#ifndef SHAPES_H
#define SHAPES_H

#include <iostream>
#include <string>
#include <algorithm>
#include <sstream>
#include <vector>
#include <yaml-cpp/yaml.h>
#include "shape_collide.hpp"

using namespace std;

class Shape
{
public:
    Shape()
    {
        shape_all = std::vector<std::vector<Location> >();
        id_all = std::vector<int>();
    }
    ~Shape()
    {
        shape_all.clear();
        id_all.clear();
    }

    bool getShape(const char* FileName)
    {
        // 读取FileName的yaml文件
        YAML::Node config = YAML::LoadFile(FileName);
        // 如果打开失败
        if(!config)
        {
            std::cout << "Error openning input SHAPE YAML file."<<std::endl;
            return false;
        }
        std::vector<Location> shape_temp;
        for (const auto& node : config["agents"]) {
            const auto& id = node["id"];
            // const auto& goal = node["goal"];
            // const auto& agent_name = node["name"];
            // std::string agent_name_str = agent_name.as<std::string>();
            // std::cout << agent_name_str << " 's data is reading "<< std::endl;
            // startStates.emplace_back(State(0, start[0].as<int>(), start[1].as<int>(), 1));
            // std::cout << "s: " << startStates.back() << std::endl;
            // goals.emplace_back(Location(goal[0].as<int>(), goal[1].as<int>()));
            id_all.emplace_back(id.as<int>());
            //读取形状
            shape_temp.clear();
            // std::cout << "--id: " << id.as<int>() << std::endl;
            for (const auto& shape : node["shape"]) {
                shape_temp.emplace_back(Location(shape[0].as<int>(), shape[1].as<int>()));
                // std::cout << "shape: " << shape_temp.back() << std::endl;
            }
            shape_all.emplace_back(shape_temp);
        }
        return true;
    }




    std::vector<std::vector<Location> >shape_all;
    std::vector<int> id_all;
};

#endif

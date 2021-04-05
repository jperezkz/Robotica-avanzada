//
// Created by pbustos on 5/4/21.
//

#ifndef CONTROLLER_DSR_PLAN_H
#define CONTROLLER_DSR_PLAN_H

class Plan
{
public:
    enum class Actions {GOTO};
    Actions action;
    std::string target_place;
    std::map<std::string, double> params;
    bool is_active = false;
    bool is_location(const Mat::Vector2d &loc)
    {
        return Mat::Vector2d(params.at("x"), params.at("y")) == loc;
    }
    void print()
    {
        std::cout << "------ Begin Plan ----------" << std::endl;
        std::cout << "\t Action: " << action_strings[action] << " Taget_place: " << target_place << std::endl;
        for(auto &&[k,v]: params)
            std::cout << "\t par1: " << k << " : " << std::to_string(v) << std::endl;
        std::cout << "------ End Plan ----------" << std::endl;
    };
    std::string to_string()
    {
        std::stringstream ss;
        ss << "Intention: " << action_strings[action] << std::endl;
        ss << "      location: " << target_place << std::endl;
        ss << "      final pose: " << target_place << std::endl;
        for(auto &&[k,v]: params)
            ss << "\t" << k << " : " << std::to_string(v) << std::endl;
        return ss.str();
    };

private:
    std::map<Actions, std::string> action_strings{{Actions::GOTO, "GOTO"}};
};


#endif //CONTROLLER_DSR_PLAN_H

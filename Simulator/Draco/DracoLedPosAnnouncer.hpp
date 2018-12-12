#pragma once

#include <vector>
#include <string>

#include <dart/dart.hpp>

#include <Utils/DartpThread.hpp>
#include <PnC/DracoPnC/DracoMoCapManager.hpp>

class DracoLedPosAnnouncer: public DartpThread{
public:

    DracoLedPosAnnouncer(dart::simulation::WorldPtr _world);
    virtual ~DracoLedPosAnnouncer(void){}

    virtual void run(void);

protected:
    int count_;
    int turn_off_count_;
    std::vector<int> led_turn_off_st_count_;
    std::vector<int> led_turn_off_end_count_;
    int socket_;
    std::vector<std::string> led_link_name_list_;

    dart::simulation::WorldPtr world_;
    dart::dynamics::SkeletonPtr skel_;
    dart::dynamics::SkeletonPtr ground_;
};
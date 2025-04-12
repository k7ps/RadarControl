#include "proto.h"

#include <cmath>

#include "util.h"


void PrepareParams(Proto::Parameters& params) {
    params.mutable_small_radar()->set_view_angle(DegToRad(params.small_radar().view_angle()));
    params.mutable_small_radar()->set_ang_stddev(DegToRad(params.small_radar().ang_stddev()));
    params.mutable_small_radar()->set_angle_speed(DegToRad(params.small_radar().angle_speed()));
    params.mutable_small_radar()->set_responsible_sector_start(DegToRad(params.small_radar().responsible_sector_start()));
    params.mutable_small_radar()->set_responsible_sector_end(DegToRad(params.small_radar().responsible_sector_end()));
    params.mutable_big_radar()->set_ang_stddev(DegToRad(params.big_radar().ang_stddev()));
    params.mutable_simulator()->set_max_deviation_angle_vertical(DegToRad(params.simulator().max_deviation_angle_vertical()));
    params.mutable_general()->set_margin_angle(DegToRad(params.general().margin_angle()));

    params.mutable_small_radar()->set_angle_speed(params.small_radar().angle_speed() / 1000);
    params.mutable_simulator()->set_min_target_speed(params.simulator().min_target_speed() / 1000);
    params.mutable_simulator()->set_max_target_speed(params.simulator().max_target_speed() / 1000);
    params.mutable_defense()->set_rocket_speed(params.defense().rocket_speed() / 1000);

    const auto play_speed = params.general().play_speed();
    params.mutable_small_radar()->set_angle_speed(params.small_radar().angle_speed() * play_speed);
    params.mutable_small_radar()->set_frequency(params.small_radar().frequency() * play_speed);
    params.mutable_big_radar()->set_frequency(params.big_radar().frequency() * play_speed);
    params.mutable_general()->set_death_time(params.general().death_time() / play_speed);
    params.mutable_defense()->set_time_to_launch_rocket(params.defense().time_to_launch_rocket() / play_speed);
    params.mutable_defense()->set_rocket_speed(params.defense().rocket_speed() * play_speed);
    params.mutable_simulator()->set_targets_per_minute(params.simulator().targets_per_minute() * play_speed);
    params.mutable_simulator()->set_min_target_speed(params.simulator().min_target_speed() * play_speed);
    params.mutable_simulator()->set_max_target_speed(params.simulator().max_target_speed() * play_speed);
}

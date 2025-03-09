#include "proto.h"

#include <cmath>


void PrepareParams(Proto::Parameters& params) {
    params.mutable_small_radar()->set_view_angle(params.small_radar().view_angle() * M_PI / 180);
    params.mutable_small_radar()->set_ang_stddev(params.small_radar().ang_stddev() * M_PI / 180);
    params.mutable_small_radar()->set_angle_speed(params.small_radar().angle_speed() * M_PI / 180);
    params.mutable_small_radar()->set_responsible_sector_start(params.small_radar().responsible_sector_start() * M_PI / 180);
    params.mutable_small_radar()->set_responsible_sector_end(params.small_radar().responsible_sector_end() * M_PI / 180);
    params.mutable_big_radar()->set_ang_stddev(params.big_radar().ang_stddev() * M_PI / 180);
    params.mutable_simulator()->set_max_deviation_angle_vertical(params.simulator().max_deviation_angle_vertical() * M_PI / 180);
    params.mutable_general()->set_margin_angle(params.general().margin_angle() * M_PI / 180);

    params.mutable_simulator()->set_min_target_speed(params.simulator().min_target_speed() / 1000);
    params.mutable_simulator()->set_max_target_speed(params.simulator().max_target_speed() / 1000);

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

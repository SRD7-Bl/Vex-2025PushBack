#pragma once
#include <variant>
#include <cstddef>
#include <cstdint>
#include <string_view>
#include <iostream>
#include <queue>
#include <optional>
#include <string>
#include <limits>    
#include "main.h"
#include "lemlib/api.hpp"
#include "pros/apix.h"
#include "pros/adi.hpp"
//#include "coordinate_of_the_field.cpp"

enum class ComponentType : std::uint8_t { Landmark, Start, Goal, Loader, Zone, WallNode, Block}; // The enum class ensures that the name is private—it won't be thrown into the global scope.
struct Map_Coordinate{ //Unit: inches
    const char* id;
    ComponentType type;
    float x_co,y_co;
    float theta_ro;
};

extern const Map_Coordinate FIELD_COORDINATE[];
extern const std::size_t    FIELD_COORDINATE_N;

const Map_Coordinate* find_coord(std::string_view id);

inline float coord_x(std::string_view id){ auto c=find_coord(id); return c?c->x_co:0.0f; }
inline float coord_y(std::string_view id){ auto c=find_coord(id); return c?c->y_co:0.0f; }
inline float coord_theta(std::string_view id){ auto c=find_coord(id); return c?c->theta_ro:0.0f; }

struct CmdStop                { };
struct CmdFaceTargetDirection {double TargetDeg; };
struct CmdFacePointDirection  {double TargetX, TargetY; };
struct CmdGoto                {double TargetX, TargetY, ms; };
struct CmdIntake              {int Target_Number; };
struct CmdOutfeed             {int Target_Number; };
struct CmdWaitDriveIdle       {int timeout_ms = 3000; }; // TIMEOUT

using Command = std::variant<CmdStop, CmdFaceTargetDirection, CmdFacePointDirection, CmdGoto, CmdIntake, CmdOutfeed, CmdWaitDriveIdle>;

static inline double wrap_deg(float deg){ //fold any angle back to a fixed interval (-180,180].
    double r = std::remainder(deg, 360.0);
    if (r >= 180.0) r -= 360.0;
    if (r <  -180.0) r += 360.0;
    return r;
}

constexpr double FIELD_X = 140.45;
constexpr double FIELD_Y = 140.41;

inline double mirror_x        (bool isBlue, double x)  { return isBlue ? (FIELD_X - x) : x; }
inline double mirror_y   (bool isBlue, double y)  { return isBlue ? (FIELD_Y - y) : y; }                        // For other obj
//inline double mirror_y_center (bool isBlue, double y)  {return isBlue ? (FIELD_Y - y) : y;} // For Upper/Lower Goal
inline double mirror_theta    (bool isBlue, double th) {
    if (!isBlue) return th;
    if (std::isnan(th)) return th;
    return wrap_deg(th + 180.0);
}

inline Map_Coordinate transform_for_alliance(const Map_Coordinate& in, bool isBlue){ // Red -> Blue 
    Map_Coordinate out = in;
    out.x_co       = mirror_x(isBlue, in.x_co);
    out.y_co       = mirror_y(isBlue, in.y_co);
    out.theta_ro= mirror_theta(isBlue, in.theta_ro);
    return out;
}

inline Map_Coordinate transform_for_CenterGoal(const Map_Coordinate& in, bool isBlue){ // Red -> Blue(for upper/lower goal)
    Map_Coordinate out = in;
    out.x_co       = mirror_x(isBlue, in.x_co);
    out.y_co       = mirror_y(isBlue, in.y_co);
    out.theta_ro= mirror_theta(isBlue, in.theta_ro);
    return out;
}
/*
inline bool run_mirror_tests() {
    bool pass = true;
    for (size_t i = 0; i < FIELD_COORDINATE_N; ++i) {
        const auto& a = FIELD_COORDINATE[i];
        auto b  = transform_for_alliance(a, true);   // red -> blue
        auto a2 = transform_for_alliance(b, false);  // blue -> red
        const bool test_x = std::abs(a2.x_co - a.x_co) <= 1.0;
        const bool test_y = std::abs(a2.y_co - a.y_co) <= 1.0;
        bool test_theta   = true;
        if (!std::isnan(a.theta_ro)) {
            test_theta = std::abs(a2.theta_ro - a.theta_ro) <= 0.5;
        }

        if (!(test_x && test_y && test_theta)) {
            pass = false;
            std::printf("[FAIL] id=%.*s  dx=%.3f dy=%.3f dth=%.3f\n",
                        a.id,
                        a2.x_co - a.x_co, a2.y_co - a.y_co,
                        std::isnan(a.theta_ro) ? NAN : (a2.theta_ro - a.theta_ro));
        }
    }
    std::printf(pass ? "[PASS] mirror round-trip OK\n" : "[DONE] with failures above\n");
    return pass;
}
*/

#include "field_map.hpp" 

extern const Map_Coordinate FIELD_COORDINATE[]{
    {"Red_right_loader",ComponentType::Loader,17.58f,23.44f,0.0f},
    {"Red_left_loader",ComponentType::Loader,17.58f,116.97f,0.0f},
    {"Side_right_blue_block_left",ComponentType::Block,21.46f,1.61f,NAN},
    {"Side_right_blue_block_right",ComponentType::Block,24.69f,1.61f,NAN},
    {"Side_left_blue_block_left",ComponentType::Block,21.46f,138.80f,NAN},
    {"Side_left_blue_block_right",ComponentType::Block,24.69f,138.80f,NAN},
    {"Center_right_red_block_center",ComponentType::Block,46.65f,46.63f,NAN},
    {"Center_right_red_block_right",ComponentType::Block,46.65f,46.63f,NAN},
    {"Center_right_red_block_top",ComponentType::Block,46.65f,49.88f,NAN},
    {"Center_left_red_block_center",ComponentType::Block,46.65f,93.78f,NAN},
    {"Center_left_red_block_right",ComponentType::Block,49.88f,93.78f,NAN},
    {"Center_left_red_block_bottom",ComponentType::Block,46.65f,90.53f,NAN},
    {"Right_LongGoal_red_end",ComponentType::Goal,45.83f,23.44f,180.0f}, //23.44
    {"Left_LongGoal_red_end",ComponentType::Goal, 45.83f,116.97f,180.0f},
    {"LowerGoal_red_end",ComponentType::Goal,62.23f,62.22f,225.0f},
    {"UpperGoal_red_end",ComponentType::Goal,62.23f,78.19f,135.0f},
    {"Parking_blue_block_left1",ComponentType::Block,14.03f,75.05f,NAN},
    {"Parking_blue_block_left2",ComponentType::Block,14.03f,71.82f,NAN},
    {"Parking_blue_block_left3",ComponentType::Block,14.03f,68.59f,NAN},
    {"Parking_blue_block_left4",ComponentType::Block,14.03f,65.36f,NAN},
    {"UnderGoal_right_red_block_left",ComponentType::Block,65.38f,23.44f,NAN},
    {"UnderGoal_right_red_block_right",ComponentType::Block,68.61f,23.44f,NAN},
    {"UnderGoal_right_blue_block_left",ComponentType::Block,71.84f,23.44f,NAN},
    {"UnderGoal_left_red_block_left",ComponentType::Block,65.38f,116.97f,NAN},
    {"UnderGoal_left_red_block_right",ComponentType::Block,68.61f,116.97f,NAN},
    {"UnderGoal_left_blue_block_left",ComponentType::Block,71.84f,116.97f,NAN},
    {"Left_bottom_SpecPoint",ComponentType::Landmark,39.44f,26.5f,0.0f}, //
    {"Left_top_SpecPoint",ComponentType::Landmark,37.44f,112.15f,0.0f}, //108.0
    {"Left_Descore_point",ComponentType::Landmark,49.00f,130.5f,0.0f},
    {"Right_SidePoint_BlueSide",ComponentType::Landmark,0.0f,0.0f,0.0f},
    {"Left_SidePoint_RedSide",ComponentType::Landmark,0.0f,0.0f,0.0f},
};

/*
    {"UnderGoal_right_blue_block_right",ComponentType::Block,75.07f,23.44f,NAN},
    {"UnderGoal_left_blue_block_right",ComponentType::Block,75.07f,116.97f,NAN},
    {"LowerGoal_blue_end",ComponentType::Goal,78.21f,78.19f,45.0f},
    {"UpperGoal_blue_end",ComponentType::Goal,78.21f,62.22f,315.0f},
    {"Center_right_blue_block_left",ComponentType::Block,90.57f,46.63f,NAN},
    {"Center_left_blue_block_left",ComponentType::Block,90.57f,93.78f,NAN},
    {"Right_LongGoal_blue_end",ComponentType::Goal,94.62f,23.44f,0.0f},
    {"Left_LongGoal_blue_end",ComponentType::Goal,94.62f,116.97f,0.0f},
    {"Center_right_blue_block_center",ComponentType::Block,94.62f,46.63f,NAN},
    {"Center_right_blue_block_top",ComponentType::Block,94.62f,49.88f,NAN},
    {"Center_left_blue_block_center",ComponentType::Block,94.62f,93.78f,NAN},
    {"Center_left_blue_block_bottom",ComponentType::Block,94.62f,90.53f,NAN},
    {"Side_right_red_block_left",ComponentType::Block,115.76f,1.61f,NAN},
    {"Side_left_red_block_left",ComponentType::Block,115.76f,138.80f,NAN},
    {"Side_right_red_block_right",ComponentType::Block,118.99f,1.61f,NAN},
    {"Side_left_red_block_right",ComponentType::Block,118.99f,138.80f,NAN},
    {"Parking_red_block_left1",ComponentType::Block,126.42f,75.05f,NAN},
    {"Parking_red_block_left2",ComponentType::Block,126.42f,71.82f,NAN},
    {"Parking_red_block_left3",ComponentType::Block,126.42f,68.59f,NAN},
    {"Parking_red_block_left4",ComponentType::Block,126.42f,65.36f,NAN},
    {"Blue_right_loader",ComponentType::Loader,137.87f,23.44f,180.0f},
    {"Blue_left_loader",ComponentType::Loader,137.87f,116.97f,180.0f},
    {"Right_bottom_SpecPoint",ComponentType::Landmark,116.97f,23.44f,0.0f},
    {"Right_top_SpecPoint",ComponentType::Landmark,116.97f,116.97f,0.0f},
*/

extern const std::size_t FIELD_COORDINATE_N =
    sizeof(FIELD_COORDINATE)/sizeof(FIELD_COORDINATE[0]); //Find the size of the struct

const Map_Coordinate* find_coord(std::string_view id) {
    for (std::size_t i=0; i<FIELD_COORDINATE_N; ++i)
        if (id == std::string_view(FIELD_COORDINATE[i].id))
            return &FIELD_COORDINATE[i];
    return nullptr;
}

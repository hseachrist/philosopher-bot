#ifndef RPS_POS_H
#define RPS_POS_H

class RPSPose {
public:
    RPSPose(float px = 0, float py = 0, float ptheta = 0) : m_x(px), m_y(py), m_theta(ptheta) {}
    float x();
    float y();
    float angle();

private:
    float m_x, m_y, m_theta;
};

enum RPSPoseType {
    RPS_FIRST_TURN = 0,
    RPS_BASKET_LINEUP,
    RPS_STOVE_LIFT,
    RPS_JUKEBOX,
    RPS_START,
    NUM_RPS_POSE
};

class RPSPositions {
public:
    static void calibrate();
    static const char *get_name(RPSPoseType);
    static RPSPose get(RPSPoseType);
    static void print(RPSPoseType);
private:
    static RPSPose pos_list[NUM_RPS_POSE];
    static const char *pos_names[NUM_RPS_POSE];
};


#endif
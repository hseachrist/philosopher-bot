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

#endif
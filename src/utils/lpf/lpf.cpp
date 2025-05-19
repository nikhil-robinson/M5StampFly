#include <utils/lpf/lpf.hpp>

Filter::Filter() {
    m_state = 0.0f;
    m_T     = 0.0025f;
    m_h     = 0.0025f;
}

void Filter::reset(void) {
    m_state = 0.0f;
}

void Filter::set_parameter(float T, float h) {
    m_T = T;
    m_h = h;
}

float Filter::update(float u, float h) {
    m_h     = h;
    m_state = m_state * m_T / (m_T + m_h) + u * m_h / (m_T + m_h);
    m_out   = m_state;
    return m_out;
}

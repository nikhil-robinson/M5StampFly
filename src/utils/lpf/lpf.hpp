#pragma once

class Filter {
   private:
    float m_state;
    float m_T;
    float m_h;

   public:
    float m_out;
    Filter();
    void set_parameter(float T, float h);
    void reset(void);
    float update(float u, float h);
};


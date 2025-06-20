#define ARM_MATH_CM4

#include <vector>
#include <arm_math.h>

class IQBalancerCMSIS {
   public:
    struct Params {
        int32_t gain_q;       // посилення Q (Q31, множник 1.0 = 0x7FFFFFFF)
        int32_t phase_error;  // фазовий зсув (Q15 радіан)
        int32_t dc_i;
        int32_t dc_q;
    };

    IQBalancerCMSIS();

    void estimate(const q31_t* i_buf, const q31_t* q_buf, uint32_t length);
    void apply(q31_t* i_buf, q31_t* q_buf, uint32_t length) const;
    const Params& getParams() const;

   private:
    Params params_;
};
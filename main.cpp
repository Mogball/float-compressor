#include <iostream>

class Float16Compressor {

    union Bits {
        float f;
        int32_t si;
        uint32_t ui;
    };

    static int const shift = 13;
    static int const shift_sign = 16;

    static int32_t const n_inf = 0x7F800000; // float32 infinity
    static int32_t const n_max = 0x477FE000; // max float16 normal as a float32
    static int32_t const n_min = 0x38800000; // min float16 normal as a float32
    static int32_t const n_sign = 0x80000000; // float32 sign bit

    static int32_t const c_inf = n_inf >> shift;
    static int32_t const n_nan = (c_inf + 1) << shift; // minimum float16 nan as a float32
    static int32_t const c_max = n_max >> shift;
    static int32_t const c_min = n_min >> shift;
    static int32_t const c_sign = n_sign >> shift_sign; // float16 sign bit

    static int32_t const n_mul = 0x52000000; // (1 << 23) / n_min
    static int32_t const c_mul = 0x33800000; // n_min / (1 << (23 - shift))

    static int32_t const c_sub = 0x003FF; // max float32 subnormal down shifted
    static int32_t const c_nor = 0x00400; // min float32 normal down shifted

    static int32_t const d_max = c_inf - c_max - 1;
    static int32_t const d_min = c_min - c_sub - 1;

public:

    static uint16_t compress(float value) {
        Bits v{}, s{};
        v.f = value;
        uint32_t sign = static_cast<uint32_t>(v.si & n_sign);
        v.si ^= sign;
        sign >>= shift_sign; // logical shift
        s.si = n_mul;
        s.si = static_cast<int32_t>(s.f * v.f); // correct subnormals
        v.si ^= (s.si ^ v.si) & -(n_min > v.si);
        v.si ^= (n_inf ^ v.si) & -((n_inf > v.si) & (v.si > n_max));
        v.si ^= (n_nan ^ v.si) & -((n_nan > v.si) & (v.si > n_inf));
        v.ui >>= shift; // logical shift
        v.si ^= ((v.si - d_max) ^ v.si) & -(v.si > c_max);
        v.si ^= ((v.si - d_min) ^ v.si) & -(v.si > c_sub);
        return static_cast<uint16_t>(v.ui | sign);
    }

    static float decompress(uint16_t value) {
        Bits v{};
        v.ui = value;
        int32_t sign = v.si & c_sign;
        v.si ^= sign;
        sign <<= shift_sign;
        v.si ^= ((v.si + d_min) ^ v.si) & -(v.si > c_sub);
        v.si ^= ((v.si + d_max) ^ v.si) & -(v.si > c_max);
        Bits s{};
        s.si = c_mul;
        s.f *= v.si;
        int32_t mask = -(c_nor > v.si);
        v.si <<= shift;
        v.si ^= (s.si ^ v.si) & mask;
        v.si |= sign;
        return v.f;
    }

};

class FloatCompressor {

    union Bits {
        float f;
        int32_t si;
        uint32_t ui;
    };

    bool negatives;
    bool lossless;
    int32_t f_max;
    int32_t f_min;
    int32_t f_eps;
    int32_t c_max;
    int32_t c_zero;
    int32_t p_delta;
    int32_t n_delta;
    int shift;

    static int32_t const signF = 0x80000000;
    static int32_t const absF = ~signF;

public:

    FloatCompressor(float min, float epsilon, float max, int precision) {
        // legal values
        // min <= 0 < epsilon < max
        // 0 <= precision <= 23
        shift = 23 - precision;
        Bits v{};
        v.f = min;
        f_min = v.si;
        v.f = epsilon;
        f_eps = v.si;
        v.f = max;
        f_max = v.si;
        negatives = f_min < 0;
        lossless = shift == 0;
        int32_t pepsU, nepsU;
        if (lossless) {
            nepsU = f_eps;
            pepsU = f_eps ^ signF;
            c_max = f_max ^ signF;
            c_zero = signF;
        } else {
            nepsU = uint32_t(f_eps ^ signF) >> shift;
            pepsU = uint32_t(f_eps) >> shift;
            c_max = uint32_t(f_max) >> shift;
            c_zero = 0;
        }
        p_delta = pepsU - c_zero - 1;
        n_delta = nepsU - c_max - 1;
    }

    float clamp(float value) {
        Bits v{};
        v.f = value;
        int32_t max = f_max;
        if (negatives) {
            max ^= (f_min ^ f_max) & -(0 > v.si);
        }
        v.si ^= (max ^ v.si) & -(v.si > max);
        v.si &= -(f_eps <= (v.si & absF));
        return v.f;
    }

    uint32_t compress(float value) {
        Bits v{};
        v.f = clamp(value);
        if (lossless) {
            v.si ^= signF;
        } else {
            v.ui >>= shift;
        }
        if (negatives) {
            v.si ^= ((v.si - n_delta) ^ v.si) & -(v.si > c_max);
        }
        v.si ^= ((v.si - p_delta) ^ v.si) & -(v.si > c_zero);
        if (lossless) {
            v.si ^= signF;
        }
        return v.ui;
    }

    float decompress(uint32_t value) {
        Bits v{};
        v.ui = value;
        if (lossless) {
            v.si ^= signF;
        }
        v.si ^= ((v.si + p_delta) ^ v.si) & -(v.si > c_zero);
        if (negatives) {
            v.si ^= ((v.si + n_delta) ^ v.si) & -(v.si > c_max);
        }
        if (lossless) {
            v.si ^= signF;
        } else {
            v.si <<= shift;
        }
        return v.f;
    }

};

union encoder {
    float r;
    uint32_t n;
};

// [sign] [exponent] [significand]
// [1] [5] [12]
uint32_t compress18(float r) {
    encoder e{};
    e.r = r;
    uint32_t t = (e.n & 0x7ff800) >> 11;
    t |= (((e.n & 0x7f800000) >> 23) - 0x70) << 12;
    t |= (e.n & 0x80000000) >> 14;
    return t;
}

float decompress18(uint32_t n) {
    uint32_t t = (n & 0xfff) << 11;
    t |= (((n & 0x1f000) >> 12) + 0x70) << 23;
    t |= (n & 0x20000) << 14;
    encoder e{};
    e.n = t;
    return e.r;
}

int main() {
    float g = -724.99f;
    FloatCompressor c{-65504, static_cast<float>(6.103515625e-05), 65504, 12};
    uint32_t comp = c.compress(g);
    float uncomp = c.decompress(comp);
    printf("%f\n%u\n%f\n%f\n\n", g, comp, uncomp, decompress18(compress18(g)));
}

#ifndef FILTER_COEFFICIENTS_H
#define FILTER_COEFFICIENTS_H

typedef double REAL;

#define NBQ 4  // Number of biquad sections in the cascaded band-pass filter

// -----------------------------------------------------------------------------
// Filter selection helpers
// -----------------------------------------------------------------------------

// Exactly one of USE_FILTER_1 or USE_FILTER_2 must be defined. By default we
// enable the more stable Filter 2.
#if !defined(USE_FILTER_1) && !defined(USE_FILTER_2)
#define USE_FILTER_2
#endif

#if defined(USE_FILTER_1) && defined(USE_FILTER_2)
#error "Only one filter variant can be enabled at a time."
#endif

// -----------------------------------------------------------------------------
// Filter 1 - Original cascaded biquad implementation
// -----------------------------------------------------------------------------
#ifdef USE_FILTER_1

static const REAL biquad_b1[NBQ][2] = {
    { 1.0000000000000000, -2.0000000000000000 },
    { 1.0000000000000000, -2.0000000000000000 },
    { 1.0000000000000000,  2.0000000000000000 },
    { 1.0000000000000000,  2.0000000000000000 }
};

static const REAL biquad_a1[NBQ][2] = {
    { 0.9990064400076268, -1.9988397553682857 },
    { 0.9974599955996946, -1.9972742811624338 },
    { 0.9972552588604660, -1.9970383749074980 },
    { 0.9988028604422975, -1.9985608414939200 }
};

static const REAL filter1_gain = 8007046836.551489;

#endif // USE_FILTER_1

// -----------------------------------------------------------------------------
// Filter 2 - Direct Form II biquad implementation
// -----------------------------------------------------------------------------
#ifdef USE_FILTER_2

static const REAL biquad_b2[NBQ][3] = {
    { 0.2612868520444742,  0.0000000000000000, -0.2612868520444742 },
    { 0.2381374035970405,  0.0000000000000000, -0.2381374035970405 },
    { 0.3557618306799391,  0.0000000000000000, -0.3557618306799391 },
    { 0.2458170098575981,  0.0000000000000000, -0.2458170098575981 }
};

static const REAL biquad_a2[NBQ][2] = {
    { -2.0846630514680906,  1.3672878387159881 },
    { -2.0277396729028040,  1.0297780232019926 },
    { -2.8852647346711797,  2.1764034525956050 },
    { -2.0927930065355670,  1.0955740976448156 }
};

static const REAL filter2_gain = 1.0;

#endif // USE_FILTER_2

#endif // FILTER_COEFFICIENTS_H
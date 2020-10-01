#define ECG_sampleRate 512  //Hz

// 0.5 Hz butterworth high-pass, 2 order
const double a_hp[] = {1, -1.99132254835917, 0.991360035149071};
const double b_hp[] = {0.995670645877060, -1.99134129175412, 0.995670645877060};

// 50 Hz Butter low-pass, 3 order
const double a_lp[] = {1, -1.78847140715168, 1.21164264222362,  -0.286869532448224};
const double b_lp[] = {0.0170377128279652, 0.0511131384838956,  0.0511131384838956,  0.0170377128279652};

////Notch50, 48~52,3 order
const double a_n50[] = {1,  -4.82670443192572, 10.6685616767275,  -13.5064582140685, 10.3250327496380,  -4.52086729602225, 0.906481521898628};
const double b_n50[] = {0.952093231830352,  -4.67190889390532, 10.4979447423026,  -13.5102121542077, 10.4979447423026,  -4.67190889390532, 0.952093231830352};
//
////Notch60, 58~62,1 order
const double a_n60[] = {1,  -4.37428878700082, 9.28081903459846,  -11.5658089363129, 8.98198159089241,  -4.09711831321292, 0.906481521898628};
const double b_n60[] = {0.952093231830301,  -4.23400251180179, 9.13254784186472,  -11.5692110129236, 9.13254784186472,  -4.23400251180179, 0.952093231830301};
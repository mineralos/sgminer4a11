#include "config.h"

#include <stdlib.h>
#include <assert.h>
#include <fcntl.h>
#include <limits.h>
#include <unistd.h>
#include <stdbool.h>

#include "logging.h"
#include "miner.h"
#include "util.h"

#include "asic_inno_cmd.h"
#include "asic_inno_clock.h"


const struct PLL_Clock PLL_Clk_12Mhz[A12_PLL_LV_NUM]={
    {  0,   30, A4_PLL(1,  80, 5)},
    {  1,   32, A4_PLL(1,  84, 5)},
    {  2,   33, A4_PLL(1,  88, 5)},
    {  3,   35, A4_PLL(1,  92, 5)},
    {  4,   36, A4_PLL(1,  96, 5)},
    {  5,   38, A4_PLL(1, 100, 5)},
    {  6,   39, A4_PLL(1,  52, 4)},
    {  7,   41, A4_PLL(1,  54, 4)},
    {  8,   42, A4_PLL(1,  56, 4)},
    {  9,   44, A4_PLL(1,  58, 4)},
    { 10,   45, A4_PLL(1,  60, 4)},
    { 11,   47, A4_PLL(1,  62, 4)},
    { 12,   48, A4_PLL(1,  64, 4)},
    { 13,   50, A4_PLL(1,  66, 4)},
    { 14,   51, A4_PLL(1,  68, 4)},
    { 15,   53, A4_PLL(1,  70, 4)},
    { 16,   54, A4_PLL(1,  72, 4)},
    { 17,   56, A4_PLL(1,  74, 4)},
    { 18,   57, A4_PLL(1,  76, 4)},
    { 19,   59, A4_PLL(1,  78, 4)},
    { 20,   60, A4_PLL(1,  80, 4)},
    { 21,   62, A4_PLL(1,  82, 4)},
    { 22,   63, A4_PLL(1,  84, 4)},
    { 23,   65, A4_PLL(1,  86, 4)},
    { 24,   66, A4_PLL(1,  88, 4)},
    { 25,   68, A4_PLL(1,  90, 4)},
    { 26,   69, A4_PLL(1,  92, 4)},
    { 27,   71, A4_PLL(1,  94, 4)},
    { 28,   72, A4_PLL(1,  96, 4)},
    { 29,   74, A4_PLL(1,  98, 4)},
    { 30,   75, A4_PLL(1, 100, 4)},
    { 31,   77, A4_PLL(1, 102, 4)},
    { 32,   78, A4_PLL(1, 104, 4)},
    { 33,   80, A4_PLL(1, 106, 4)},
    { 34,   81, A4_PLL(1, 108, 4)},
    { 35,   83, A4_PLL(1, 110, 4)},
    { 36,   84, A4_PLL(1, 112, 4)},
    { 37,   86, A4_PLL(1, 114, 4)},
    { 38,   87, A4_PLL(1, 116, 4)},
    { 39,   89, A4_PLL(1, 118, 4)},
    { 40,   90, A4_PLL(1, 120, 4)},
    { 41,   92, A4_PLL(1, 122, 4)},
    { 42,   93, A4_PLL(1, 124, 4)},
    { 43,   95, A4_PLL(1, 126, 4)},
    { 44,   96, A4_PLL(1, 128, 4)},
    { 45,   98, A4_PLL(1, 130, 4)},
    { 46,   99, A4_PLL(1, 132, 4)},
    { 47,  101, A4_PLL(1, 134, 4)},
    { 48,  102, A4_PLL(1, 136, 4)},
    { 49,  104, A4_PLL(1, 138, 4)},
    { 50,  105, A4_PLL(1, 140, 4)},
    { 51,  107, A4_PLL(1, 142, 4)},
    { 52,  108, A4_PLL(1, 144, 4)},
    { 53,  110, A4_PLL(1, 146, 4)},
    { 54,  111, A4_PLL(1, 148, 4)},
    { 55,  113, A4_PLL(1, 150, 4)},
    { 56,  114, A4_PLL(1, 152, 4)},
    { 57,  116, A4_PLL(1, 154, 4)},
    { 58,  117, A4_PLL(1, 156, 4)},
    { 59,  119, A4_PLL(1, 158, 4)},
    { 60,  120, A4_PLL(1,  80, 3)},
    { 61,  122, A4_PLL(1,  81, 3)},
    { 62,  123, A4_PLL(1,  82, 3)},
    { 63,  125, A4_PLL(1,  83, 3)},
    { 64,  126, A4_PLL(1,  84, 3)},
    { 65,  128, A4_PLL(1,  85, 3)},
    { 66,  129, A4_PLL(1,  86, 3)},
    { 67,  131, A4_PLL(1,  87, 3)},
    { 68,  132, A4_PLL(1,  88, 3)},
    { 69,  134, A4_PLL(1,  89, 3)},
    { 70,  135, A4_PLL(1,  90, 3)},
    { 71,  137, A4_PLL(1,  91, 3)},
    { 72,  138, A4_PLL(1,  92, 3)},
    { 73,  140, A4_PLL(1,  93, 3)},
    { 74,  141, A4_PLL(1,  94, 3)},
    { 75,  143, A4_PLL(1,  95, 3)},
    { 76,  144, A4_PLL(1,  96, 3)},
    { 77,  146, A4_PLL(1,  97, 3)},
    { 78,  147, A4_PLL(1,  98, 3)},
    { 79,  149, A4_PLL(1,  99, 3)},
    { 80,  150, A4_PLL(1,  50, 2)},
    { 81,  153, A4_PLL(1,  51, 2)},
    { 82,  156, A4_PLL(1,  52, 2)},
    { 83,  159, A4_PLL(1,  53, 2)},
    { 84,  162, A4_PLL(1,  54, 2)},
    { 85,  165, A4_PLL(1,  55, 2)},
    { 86,  168, A4_PLL(1,  56, 2)},
    { 87,  171, A4_PLL(1,  57, 2)},
    { 88,  174, A4_PLL(1,  58, 2)},
    { 89,  177, A4_PLL(1,  59, 2)},
    { 90,  180, A4_PLL(1,  60, 2)},
    { 91,  183, A4_PLL(1,  61, 2)},
    { 92,  186, A4_PLL(1,  62, 2)},
    { 93,  189, A4_PLL(1,  63, 2)},
    { 94,  192, A4_PLL(1,  64, 2)},
    { 95,  195, A4_PLL(1,  65, 2)},
    { 96,  198, A4_PLL(1,  66, 2)},
    { 97,  201, A4_PLL(1,  67, 2)},
    { 98,  204, A4_PLL(1,  68, 2)},
    { 99,  207, A4_PLL(1,  69, 2)},
    {100,  210, A4_PLL(1,  70, 2)},
    {101,  213, A4_PLL(1,  71, 2)},
    {102,  216, A4_PLL(1,  72, 2)},
    {103,  219, A4_PLL(1,  73, 2)},
    {104,  222, A4_PLL(1,  74, 2)},
    {105,  225, A4_PLL(1,  75, 2)},
    {106,  228, A4_PLL(1,  76, 2)},
    {107,  231, A4_PLL(1,  77, 2)},
    {108,  234, A4_PLL(1,  78, 2)},
    {109,  237, A4_PLL(1,  79, 2)},
    {110,  240, A4_PLL(2, 160, 2)},
    {111,  242, A4_PLL(2, 161, 2)},
    {112,  243, A4_PLL(2, 162, 2)},
    {113,  245, A4_PLL(2, 163, 2)},
    {114,  246, A4_PLL(2, 164, 2)},
    {115,  248, A4_PLL(2, 165, 2)},
    {116,  249, A4_PLL(2, 166, 2)},
    {117,  251, A4_PLL(2, 167, 2)},
    {118,  252, A4_PLL(2, 168, 2)},
    {119,  254, A4_PLL(2, 169, 2)},
    {120,  255, A4_PLL(2, 170, 2)},
    {121,  257, A4_PLL(2, 171, 2)},
    {122,  258, A4_PLL(2, 172, 2)},
    {123,  260, A4_PLL(2, 173, 2)},
    {124,  261, A4_PLL(2, 174, 2)},
    {125,  263, A4_PLL(2, 175, 2)},
    {126,  264, A4_PLL(2, 176, 2)},
    {127,  266, A4_PLL(2, 177, 2)},
    {128,  267, A4_PLL(2, 178, 2)},
    {129,  269, A4_PLL(2, 179, 2)},
    {130,  270, A4_PLL(2, 180, 2)},
    {131,  272, A4_PLL(2, 181, 2)},
    {132,  273, A4_PLL(2, 182, 2)},
    {133,  275, A4_PLL(2, 183, 2)},
    {134,  276, A4_PLL(2, 184, 2)},
    {135,  278, A4_PLL(2, 185, 2)},
    {136,  279, A4_PLL(2, 186, 2)},
    {137,  281, A4_PLL(2, 187, 2)},
    {138,  282, A4_PLL(2, 188, 2)},
    {139,  284, A4_PLL(2, 189, 2)},
    {140,  285, A4_PLL(2, 190, 2)},
    {141,  287, A4_PLL(2, 191, 2)},
    {142,  288, A4_PLL(2, 192, 2)},
    {143,  290, A4_PLL(2, 193, 2)},
    {144,  291, A4_PLL(2, 194, 2)},
    {145,  293, A4_PLL(2, 195, 2)},
    {146,  294, A4_PLL(2, 196, 2)},
    {147,  296, A4_PLL(2, 197, 2)},
    {148,  297, A4_PLL(2, 198, 2)},
    {149,  299, A4_PLL(2, 199, 2)},
    {150,  300, A4_PLL(2, 100, 1)},
    {151,  303, A4_PLL(2, 101, 1)},
    {152,  306, A4_PLL(2, 102, 1)},
    {153,  309, A4_PLL(2, 103, 1)},
    {154,  312, A4_PLL(2, 104, 1)},
    {155,  315, A4_PLL(2, 105, 1)},
    {156,  318, A4_PLL(2, 106, 1)},
    {157,  321, A4_PLL(2, 107, 1)},
    {158,  324, A4_PLL(2, 108, 1)},
    {159,  327, A4_PLL(2, 109, 1)},
    {160,  330, A4_PLL(2, 110, 1)},
    {161,  333, A4_PLL(2, 111, 1)},
    {162,  336, A4_PLL(2, 112, 1)},
    {163,  339, A4_PLL(2, 113, 1)},
    {164,  342, A4_PLL(2, 114, 1)},
    {165,  345, A4_PLL(2, 115, 1)},
    {166,  348, A4_PLL(2, 116, 1)},
    {167,  351, A4_PLL(2, 117, 1)},
    {168,  354, A4_PLL(2, 118, 1)},
    {169,  357, A4_PLL(2, 119, 1)},
    {170,  360, A4_PLL(2, 120, 1)},
    {171,  363, A4_PLL(2, 121, 1)},
    {172,  366, A4_PLL(2, 122, 1)},
    {173,  369, A4_PLL(2, 123, 1)},
    {174,  372, A4_PLL(2, 124, 1)},
    {175,  375, A4_PLL(2, 125, 1)},
    {176,  378, A4_PLL(2, 126, 1)},
    {177,  381, A4_PLL(2, 127, 1)},
    {178,  384, A4_PLL(2, 128, 1)},
    {179,  387, A4_PLL(2, 129, 1)},
    {180,  390, A4_PLL(2, 130, 1)},
    {181,  393, A4_PLL(2, 131, 1)},
    {182,  396, A4_PLL(2, 132, 1)},
    {183,  399, A4_PLL(2, 133, 1)},
    {184,  402, A4_PLL(2, 134, 1)},
    {185,  405, A4_PLL(2, 135, 1)},
    {186,  408, A4_PLL(2, 136, 1)},
    {187,  411, A4_PLL(2, 137, 1)},
    {188,  414, A4_PLL(2, 138, 1)},
    {189,  417, A4_PLL(2, 139, 1)},
    {190,  420, A4_PLL(2, 140, 1)},
    {191,  423, A4_PLL(2, 141, 1)},
    {192,  426, A4_PLL(2, 142, 1)},
    {193,  429, A4_PLL(2, 143, 1)},
    {194,  432, A4_PLL(2, 144, 1)},
    {195,  435, A4_PLL(2, 145, 1)},
    {196,  438, A4_PLL(2, 146, 1)},
    {197,  441, A4_PLL(2, 147, 1)},
    {198,  444, A4_PLL(2, 148, 1)},
    {199,  447, A4_PLL(2, 149, 1)},
    {200,  450, A4_PLL(2, 150, 1)},
    {201,  453, A4_PLL(2, 151, 1)},
    {202,  456, A4_PLL(2, 152, 1)},
    {203,  459, A4_PLL(2, 153, 1)},
    {204,  462, A4_PLL(2, 154, 1)},
    {205,  465, A4_PLL(2, 155, 1)},
    {206,  468, A4_PLL(2, 156, 1)},
    {207,  471, A4_PLL(2, 157, 1)},
    {208,  474, A4_PLL(2, 158, 1)},
    {209,  477, A4_PLL(2, 159, 1)},
    {210,  480, A4_PLL(2, 160, 1)},
    {211,  483, A4_PLL(2, 161, 1)},
    {212,  486, A4_PLL(2, 162, 1)},
    {213,  489, A4_PLL(2, 163, 1)},
    {214,  492, A4_PLL(2, 164, 1)},
    {215,  495, A4_PLL(2, 165, 1)},
    {216,  498, A4_PLL(2, 166, 1)},
    {217,  501, A4_PLL(2, 167, 1)},
    {218,  504, A4_PLL(2, 168, 1)},
    {219,  507, A4_PLL(2, 169, 1)},
    {220,  510, A4_PLL(2, 170, 1)},
    {221,  513, A4_PLL(2, 171, 1)},
    {222,  516, A4_PLL(2, 172, 1)},
    {223,  519, A4_PLL(2, 173, 1)},
    {224,  522, A4_PLL(2, 174, 1)},
    {225,  525, A4_PLL(2, 175, 1)},
    {226,  528, A4_PLL(2, 176, 1)},
    {227,  531, A4_PLL(2, 177, 1)},
    {228,  534, A4_PLL(2, 178, 1)},
    {229,  537, A4_PLL(2, 179, 1)},
    {230,  540, A4_PLL(2, 180, 1)},
    {231,  543, A4_PLL(2, 181, 1)},
    {232,  546, A4_PLL(2, 182, 1)},
    {233,  549, A4_PLL(2, 183, 1)},
    {234,  552, A4_PLL(2, 184, 1)},
    {235,  555, A4_PLL(2, 185, 1)},
    {236,  558, A4_PLL(2, 186, 1)},
    {237,  561, A4_PLL(2, 187, 1)},
    {238,  564, A4_PLL(2, 188, 1)},
    {239,  567, A4_PLL(2, 189, 1)},
    {240,  570, A4_PLL(2, 190, 1)},
    {241,  573, A4_PLL(2, 191, 1)},
    {242,  576, A4_PLL(2, 192, 1)},
    {243,  579, A4_PLL(2, 193, 1)},
    {244,  582, A4_PLL(2, 194, 1)},
    {245,  585, A4_PLL(2, 195, 1)},
    {246,  588, A4_PLL(2, 196, 1)},
    {247,  591, A4_PLL(2, 197, 1)},
    {248,  594, A4_PLL(2, 198, 1)},
    {249,  597, A4_PLL(2, 199, 1)},
    {250,  600, A4_PLL(2, 100, 0)},
    {251,  606, A4_PLL(2, 101, 0)},
    {252,  612, A4_PLL(2, 102, 0)},
    {253,  618, A4_PLL(2, 103, 0)},
    {254,  624, A4_PLL(2, 104, 0)},
    {255,  630, A4_PLL(2, 105, 0)},
    {256,  636, A4_PLL(2, 106, 0)},
    {257,  642, A4_PLL(2, 107, 0)},
    {258,  648, A4_PLL(2, 108, 0)},
    {259,  654, A4_PLL(2, 109, 0)},
    {260,  660, A4_PLL(2, 110, 0)},
    {261,  666, A4_PLL(2, 111, 0)},
    {262,  672, A4_PLL(2, 112, 0)},
    {263,  678, A4_PLL(2, 113, 0)},
    {264,  684, A4_PLL(2, 114, 0)},
    {265,  690, A4_PLL(2, 115, 0)},
    {266,  696, A4_PLL(2, 116, 0)},
    {267,  702, A4_PLL(2, 117, 0)},
    {268,  708, A4_PLL(2, 118, 0)},
    {269,  714, A4_PLL(2, 119, 0)},
    {270,  720, A4_PLL(2, 120, 0)},
    {271,  726, A4_PLL(2, 121, 0)},
    {272,  732, A4_PLL(2, 122, 0)},
    {273,  738, A4_PLL(2, 123, 0)},
    {274,  744, A4_PLL(2, 124, 0)},
    {275,  750, A4_PLL(2, 125, 0)},
    {276,  756, A4_PLL(2, 126, 0)},
    {277,  762, A4_PLL(2, 127, 0)},
    {278,  768, A4_PLL(2, 128, 0)},
    {279,  774, A4_PLL(2, 129, 0)},
    {280,  780, A4_PLL(2, 130, 0)},
    {281,  786, A4_PLL(2, 131, 0)},
    {282,  792, A4_PLL(2, 132, 0)},
    {283,  798, A4_PLL(2, 133, 0)},
    {284,  804, A4_PLL(2, 134, 0)},
    {285,  810, A4_PLL(2, 135, 0)},
    {286,  816, A4_PLL(2, 136, 0)},
    {287,  822, A4_PLL(2, 137, 0)},
    {288,  828, A4_PLL(2, 138, 0)},
    {289,  834, A4_PLL(2, 139, 0)},
    {290,  840, A4_PLL(2, 140, 0)},
    {291,  846, A4_PLL(2, 141, 0)},
    {292,  852, A4_PLL(2, 142, 0)},
    {293,  858, A4_PLL(2, 143, 0)},
    {294,  864, A4_PLL(2, 144, 0)},
    {295,  870, A4_PLL(2, 145, 0)},
    {296,  876, A4_PLL(2, 146, 0)},
    {297,  882, A4_PLL(2, 147, 0)},
    {298,  888, A4_PLL(2, 148, 0)},
    {299,  894, A4_PLL(2, 149, 0)},
    {300,  900, A4_PLL(2, 150, 0)},
    {301,  906, A4_PLL(2, 151, 0)},
    {302,  912, A4_PLL(2, 152, 0)},
    {303,  918, A4_PLL(2, 153, 0)},
    {304,  924, A4_PLL(2, 154, 0)},
    {305,  930, A4_PLL(2, 155, 0)},
    {306,  936, A4_PLL(2, 156, 0)},
    {307,  942, A4_PLL(2, 157, 0)},
    {308,  948, A4_PLL(2, 158, 0)},
    {309,  954, A4_PLL(2, 159, 0)},
    {310,  960, A4_PLL(2, 160, 0)},
    {311,  966, A4_PLL(2, 161, 0)},
    {312,  972, A4_PLL(2, 162, 0)},
    {313,  978, A4_PLL(2, 163, 0)},
    {314,  984, A4_PLL(2, 164, 0)},
    {315,  990, A4_PLL(2, 165, 0)},
    {316,  996, A4_PLL(2, 166, 0)},
    {317, 1002, A4_PLL(2, 167, 0)},
    {318, 1008, A4_PLL(2, 168, 0)},
    {319, 1014, A4_PLL(2, 169, 0)},
    {320, 1020, A4_PLL(2, 170, 0)},
    {321, 1026, A4_PLL(2, 171, 0)},
    {322, 1032, A4_PLL(2, 172, 0)},
    {323, 1038, A4_PLL(2, 173, 0)},
    {324, 1044, A4_PLL(2, 174, 0)},
    {325, 1050, A4_PLL(2, 175, 0)},
    {326, 1056, A4_PLL(2, 176, 0)},
    {327, 1062, A4_PLL(2, 177, 0)},
    {328, 1068, A4_PLL(2, 178, 0)},
    {329, 1074, A4_PLL(2, 179, 0)},
    {330, 1080, A4_PLL(2, 180, 0)},
    {331, 1086, A4_PLL(2, 181, 0)},
    {332, 1092, A4_PLL(2, 182, 0)},
    {333, 1098, A4_PLL(2, 183, 0)},
    {334, 1104, A4_PLL(2, 184, 0)},
    {335, 1110, A4_PLL(2, 185, 0)},
    {336, 1116, A4_PLL(2, 186, 0)},
    {337, 1122, A4_PLL(2, 187, 0)},
    {338, 1128, A4_PLL(2, 188, 0)},
    {339, 1134, A4_PLL(2, 189, 0)},
    {340, 1140, A4_PLL(2, 190, 0)},
    {341, 1146, A4_PLL(2, 191, 0)},
    {342, 1152, A4_PLL(2, 192, 0)},
    {343, 1158, A4_PLL(2, 193, 0)},
    {344, 1164, A4_PLL(2, 194, 0)},
    {345, 1170, A4_PLL(2, 195, 0)},
    {346, 1176, A4_PLL(2, 196, 0)},
    {347, 1182, A4_PLL(2, 197, 0)},
    {348, 1188, A4_PLL(2, 198, 0)},
    {349, 1194, A4_PLL(2, 199, 0)},
    {350, 1200, A4_PLL(1, 100, 0)},
    {351, 1212, A4_PLL(1, 101, 0)},
    {352, 1224, A4_PLL(1, 102, 0)},
    {353, 1236, A4_PLL(1, 103, 0)},
    {354, 1248, A4_PLL(1, 104, 0)},
    {355, 1260, A4_PLL(1, 105, 0)},
    {356, 1272, A4_PLL(1, 106, 0)},
    {357, 1284, A4_PLL(1, 107, 0)},
    {358, 1296, A4_PLL(1, 108, 0)}
};



int A1_ConfigA1PLLClock(int optPll)
{
    int i;
    int pllIdx;

    if(optPll>0){
        pllIdx=0;
        if(optPll<=PLL_Clk_12Mhz[0].speedMHz) {
            pllIdx=0; //found
        }else{
            for(i=1; i < A12_PLL_LV_NUM; i++){
                if((optPll<PLL_Clk_12Mhz[i].speedMHz)&&(optPll>=PLL_Clk_12Mhz[i-1].speedMHz)){
                    pllIdx=i-1; //found
                    break;
                }
            }
        }
        applog(LOG_NOTICE, "A1 = %d,%d", optPll, pllIdx);
        applog(LOG_NOTICE, "A1 PLL Clock = %dMHz",PLL_Clk_12Mhz[pllIdx].speedMHz);
    }

    return pllIdx;
}


void A1_SetA1PLLClock(struct A1_chain *a1,int pllClkIdx)
{
    //uint8_t i;
    struct A1_chip *chip;
    uint32_t regPll;
    uint8_t rxbuf[12];
    
    uint8_t fix_val[8] = {0x00, 0x00, 0x00, 0xA8, 0x00, 0x24, 0xFF, 0xFF}; 

    assert(a1->chips != NULL);
    assert((pllClkIdx > 0) && (pllClkIdx < A5_PLL_CLOCK_MAX));

    regPll = PLL_Clk_12Mhz[pllClkIdx].pll_reg;

    chip = &a1->chips[0];
    memcpy(chip->reg,     (uint8_t*)&regPll + 3 ,1);
    memcpy(chip->reg + 1, (uint8_t*)&regPll + 2 ,1);
    memcpy(chip->reg + 2, (uint8_t*)&regPll + 1 ,1);
    memcpy(chip->reg + 3, (uint8_t*)&regPll + 0 ,1);
    memcpy(chip->reg + 4, fix_val , 8);

    inno_cmd_write_register(a1->chain_id, ADDR_BROADCAST, chip->reg, REG_LENGTH);
    usleep(100000);
    inno_cmd_read_register(a1->chain_id, ADDR_BROADCAST, rxbuf, REG_LENGTH);
    hexdump("read value", rxbuf, 12);   
    
}



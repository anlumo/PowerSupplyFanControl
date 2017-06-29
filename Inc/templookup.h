// This file is generated by templookup.py. Do not edit.

// Note that we're using an NTC resistor, which means that lower resistance values are hotter.
#define TEMP_OFFSET 1027 // values below that are critical!
#define TEMP_CRITICAL 555 // when below this value, turn off the power supply
#define TEMP_MAX 1585 // values above that don't need any cooling
#define TEMP_STEP 2 // compression, don't need every single value
#define TEMP_COUNT 280

const uint8_t TEMP_LOOKUP[] = {
    255, 254, 253, 252, 251, 249, 248, 247, 246, 245, 244, 243, 242, 241, 240, 239, 238, 237, 235, 234, 233, 232, 231, 230, 229, 228, 227, 226, 225, 224, 223, 222, 221, 220, 219, 218, 217, 216, 215, 214, 213, 212, 210, 209, 208, 207, 206, 205, 204, 203, 202, 201, 200, 199, 198, 197, 196, 195, 194, 193, 192, 191, 190, 189, 188, 187, 186, 185, 184, 183, 182, 181, 180, 180, 179, 178, 177, 176, 175, 174, 173, 172, 171, 170, 169, 168, 167, 166, 165, 164, 163, 162, 161, 160, 159, 158, 157, 156, 155, 155, 154, 153, 152, 151, 150, 149, 148, 147, 146, 145, 144, 143, 142, 141, 140, 140, 139, 138, 137, 136, 135, 134, 133, 132, 131, 130, 129, 129, 128, 127, 126, 125, 124, 123, 122, 121, 120, 119, 119, 118, 117, 116, 115, 114, 113, 112, 111, 111, 110, 109, 108, 107, 106, 105, 104, 103, 103, 102, 101, 100, 99, 98, 97, 96, 96, 95, 94, 93, 92, 91, 90, 89, 89, 88, 87, 86, 85, 84, 83, 82, 82, 81, 80, 79, 78, 77, 76, 76, 75, 74, 73, 72, 71, 71, 70, 69, 68, 67, 66, 65, 65, 64, 63, 62, 61, 60, 60, 59, 58, 57, 56, 55, 55, 54, 53, 52, 51, 50, 50, 49, 48, 47, 46, 45, 45, 44, 43, 42, 41, 40, 40, 39, 38, 37, 36, 36, 35, 34, 33, 32, 31, 31, 30, 29, 28, 27, 27, 26, 25, 24, 23, 23, 22, 21, 20, 19, 19, 18, 17, 16, 15, 15, 14, 13, 12, 11, 11, 10, 9, 8, 7, 7, 6, 5, 4, 3, 3, 2, 1, 0
};


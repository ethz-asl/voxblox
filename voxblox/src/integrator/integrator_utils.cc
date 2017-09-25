#include "voxblox/integrator/integrator_utils.h"

namespace voxblox {

// Small class that can be used by multiple threads that need mutually exclusive
// indexes to the same array, while still covering all elements.
// The class attempts to ensure that the points are read in an order that gives
// good coverage over the pointcloud very quickly. This is so that the
// integrator can be terminated before all points have been read (due to time
// constraints) and still capture most of the geometry.
ThreadSafeIndex::ThreadSafeIndex(size_t number_of_points)
    : atomic_idx_(0),
      number_of_points_(number_of_points),
      number_of_groups_(number_of_points / step_size_) {}

// returns true if index is valid, false otherwise
bool ThreadSafeIndex::getNextIndex(size_t* idx) {
  DCHECK(idx != nullptr);
  size_t sequential_idx = atomic_idx_.fetch_add(1);

  if (sequential_idx >= number_of_points_) {
    return false;
  } else {
    *idx = getMixedIndex(sequential_idx);
    return true;
  }
}

void ThreadSafeIndex::reset() { atomic_idx_.store(0); }

size_t ThreadSafeIndex::getMixedIndex(size_t base_idx) {
  if (number_of_groups_ * step_size_ <= base_idx) {
    return base_idx;
  }

  const size_t group_num = base_idx % number_of_groups_;
  const size_t position_in_group = base_idx / number_of_groups_;

  return group_num * step_size_ + position_in_group;
}

// Lookup table for the order points in a group should be read in. This is
// simply a list from 0 to 1023 where each number has had the order of its
// bits reversed.
const std::array<size_t, ThreadSafeIndex::step_size_>
    ThreadSafeIndex::offset_lookup_ = {
        {0,   512, 256, 768,  128, 640, 384, 896,  64,  576, 320, 832,
         192, 704, 448, 960,  32,  544, 288, 800,  160, 672, 416, 928,
         96,  608, 352, 864,  224, 736, 480, 992,  16,  528, 272, 784,
         144, 656, 400, 912,  80,  592, 336, 848,  208, 720, 464, 976,
         48,  560, 304, 816,  176, 688, 432, 944,  112, 624, 368, 880,
         240, 752, 496, 1008, 8,   520, 264, 776,  136, 648, 392, 904,
         72,  584, 328, 840,  200, 712, 456, 968,  40,  552, 296, 808,
         168, 680, 424, 936,  104, 616, 360, 872,  232, 744, 488, 1000,
         24,  536, 280, 792,  152, 664, 408, 920,  88,  600, 344, 856,
         216, 728, 472, 984,  56,  568, 312, 824,  184, 696, 440, 952,
         120, 632, 376, 888,  248, 760, 504, 1016, 4,   516, 260, 772,
         132, 644, 388, 900,  68,  580, 324, 836,  196, 708, 452, 964,
         36,  548, 292, 804,  164, 676, 420, 932,  100, 612, 356, 868,
         228, 740, 484, 996,  20,  532, 276, 788,  148, 660, 404, 916,
         84,  596, 340, 852,  212, 724, 468, 980,  52,  564, 308, 820,
         180, 692, 436, 948,  116, 628, 372, 884,  244, 756, 500, 1012,
         12,  524, 268, 780,  140, 652, 396, 908,  76,  588, 332, 844,
         204, 716, 460, 972,  44,  556, 300, 812,  172, 684, 428, 940,
         108, 620, 364, 876,  236, 748, 492, 1004, 28,  540, 284, 796,
         156, 668, 412, 924,  92,  604, 348, 860,  220, 732, 476, 988,
         60,  572, 316, 828,  188, 700, 444, 956,  124, 636, 380, 892,
         252, 764, 508, 1020, 2,   514, 258, 770,  130, 642, 386, 898,
         66,  578, 322, 834,  194, 706, 450, 962,  34,  546, 290, 802,
         162, 674, 418, 930,  98,  610, 354, 866,  226, 738, 482, 994,
         18,  530, 274, 786,  146, 658, 402, 914,  82,  594, 338, 850,
         210, 722, 466, 978,  50,  562, 306, 818,  178, 690, 434, 946,
         114, 626, 370, 882,  242, 754, 498, 1010, 10,  522, 266, 778,
         138, 650, 394, 906,  74,  586, 330, 842,  202, 714, 458, 970,
         42,  554, 298, 810,  170, 682, 426, 938,  106, 618, 362, 874,
         234, 746, 490, 1002, 26,  538, 282, 794,  154, 666, 410, 922,
         90,  602, 346, 858,  218, 730, 474, 986,  58,  570, 314, 826,
         186, 698, 442, 954,  122, 634, 378, 890,  250, 762, 506, 1018,
         6,   518, 262, 774,  134, 646, 390, 902,  70,  582, 326, 838,
         198, 710, 454, 966,  38,  550, 294, 806,  166, 678, 422, 934,
         102, 614, 358, 870,  230, 742, 486, 998,  22,  534, 278, 790,
         150, 662, 406, 918,  86,  598, 342, 854,  214, 726, 470, 982,
         54,  566, 310, 822,  182, 694, 438, 950,  118, 630, 374, 886,
         246, 758, 502, 1014, 14,  526, 270, 782,  142, 654, 398, 910,
         78,  590, 334, 846,  206, 718, 462, 974,  46,  558, 302, 814,
         174, 686, 430, 942,  110, 622, 366, 878,  238, 750, 494, 1006,
         30,  542, 286, 798,  158, 670, 414, 926,  94,  606, 350, 862,
         222, 734, 478, 990,  62,  574, 318, 830,  190, 702, 446, 958,
         126, 638, 382, 894,  254, 766, 510, 1022, 1,   513, 257, 769,
         129, 641, 385, 897,  65,  577, 321, 833,  193, 705, 449, 961,
         33,  545, 289, 801,  161, 673, 417, 929,  97,  609, 353, 865,
         225, 737, 481, 993,  17,  529, 273, 785,  145, 657, 401, 913,
         81,  593, 337, 849,  209, 721, 465, 977,  49,  561, 305, 817,
         177, 689, 433, 945,  113, 625, 369, 881,  241, 753, 497, 1009,
         9,   521, 265, 777,  137, 649, 393, 905,  73,  585, 329, 841,
         201, 713, 457, 969,  41,  553, 297, 809,  169, 681, 425, 937,
         105, 617, 361, 873,  233, 745, 489, 1001, 25,  537, 281, 793,
         153, 665, 409, 921,  89,  601, 345, 857,  217, 729, 473, 985,
         57,  569, 313, 825,  185, 697, 441, 953,  121, 633, 377, 889,
         249, 761, 505, 1017, 5,   517, 261, 773,  133, 645, 389, 901,
         69,  581, 325, 837,  197, 709, 453, 965,  37,  549, 293, 805,
         165, 677, 421, 933,  101, 613, 357, 869,  229, 741, 485, 997,
         21,  533, 277, 789,  149, 661, 405, 917,  85,  597, 341, 853,
         213, 725, 469, 981,  53,  565, 309, 821,  181, 693, 437, 949,
         117, 629, 373, 885,  245, 757, 501, 1013, 13,  525, 269, 781,
         141, 653, 397, 909,  77,  589, 333, 845,  205, 717, 461, 973,
         45,  557, 301, 813,  173, 685, 429, 941,  109, 621, 365, 877,
         237, 749, 493, 1005, 29,  541, 285, 797,  157, 669, 413, 925,
         93,  605, 349, 861,  221, 733, 477, 989,  61,  573, 317, 829,
         189, 701, 445, 957,  125, 637, 381, 893,  253, 765, 509, 1021,
         3,   515, 259, 771,  131, 643, 387, 899,  67,  579, 323, 835,
         195, 707, 451, 963,  35,  547, 291, 803,  163, 675, 419, 931,
         99,  611, 355, 867,  227, 739, 483, 995,  19,  531, 275, 787,
         147, 659, 403, 915,  83,  595, 339, 851,  211, 723, 467, 979,
         51,  563, 307, 819,  179, 691, 435, 947,  115, 627, 371, 883,
         243, 755, 499, 1011, 11,  523, 267, 779,  139, 651, 395, 907,
         75,  587, 331, 843,  203, 715, 459, 971,  43,  555, 299, 811,
         171, 683, 427, 939,  107, 619, 363, 875,  235, 747, 491, 1003,
         27,  539, 283, 795,  155, 667, 411, 923,  91,  603, 347, 859,
         219, 731, 475, 987,  59,  571, 315, 827,  187, 699, 443, 955,
         123, 635, 379, 891,  251, 763, 507, 1019, 7,   519, 263, 775,
         135, 647, 391, 903,  71,  583, 327, 839,  199, 711, 455, 967,
         39,  551, 295, 807,  167, 679, 423, 935,  103, 615, 359, 871,
         231, 743, 487, 999,  23,  535, 279, 791,  151, 663, 407, 919,
         87,  599, 343, 855,  215, 727, 471, 983,  55,  567, 311, 823,
         183, 695, 439, 951,  119, 631, 375, 887,  247, 759, 503, 1015,
         15,  527, 271, 783,  143, 655, 399, 911,  79,  591, 335, 847,
         207, 719, 463, 975,  47,  559, 303, 815,  175, 687, 431, 943,
         111, 623, 367, 879,  239, 751, 495, 1007, 31,  543, 287, 799,
         159, 671, 415, 927,  95,  607, 351, 863,  223, 735, 479, 991,
         63,  575, 319, 831,  191, 703, 447, 959,  127, 639, 383, 895,
         255, 767, 511, 1023}};

// This class assumes PRE-SCALED coordinates, where one unit = one voxel size.
// The indices are also returned in this scales coordinate system, which should
// map to voxel indices.
RayCaster::RayCaster(const Point& origin, const Point& point_G,
                     const bool is_clearing_ray,
                     const bool voxel_carving_enabled,
                     const FloatingPoint max_ray_length_m,
                     const FloatingPoint voxel_size_inv,
                     const FloatingPoint truncation_distance,
                     const bool cast_from_origin) {
  const Ray unit_ray = (point_G - origin).normalized();

  Point ray_end;
  if (is_clearing_ray) {
    FloatingPoint ray_length = (point_G - origin).norm();
    ray_length = std::min(std::max(ray_length - truncation_distance,
                                   static_cast<FloatingPoint>(0.0)),
                          max_ray_length_m);
    ray_end = origin + unit_ray * ray_length;
  } else {
    ray_end = point_G + unit_ray * truncation_distance;
  }
  const Point ray_start = voxel_carving_enabled
                              ? origin
                              : (point_G - unit_ray * truncation_distance);

  const Point start_scaled = ray_start * voxel_size_inv;
  const Point end_scaled = ray_end * voxel_size_inv;

  if (cast_from_origin) {
    setupRayCaster(start_scaled, end_scaled);
  } else {
    setupRayCaster(end_scaled, start_scaled);
  }
}

RayCaster::RayCaster(const Point& start_scaled, const Point& end_scaled) {
  setupRayCaster(start_scaled, end_scaled);
}

// returns false if ray terminates at ray_index, true otherwise
bool RayCaster::nextRayIndex(AnyIndex* ray_index) {
  if (current_step_++ > ray_length_in_steps_) {
    return false;
  }

  DCHECK(ray_index != nullptr);
  *ray_index = curr_index_;

  int t_min_idx;
  t_to_next_boundary_.minCoeff(&t_min_idx);
  curr_index_[t_min_idx] += ray_step_signs_[t_min_idx];
  t_to_next_boundary_[t_min_idx] += t_step_size_[t_min_idx];

  return true;
}

void RayCaster::setupRayCaster(const Point& start_scaled,
                               const Point& end_scaled) {
  curr_index_ = getGridIndexFromPoint(start_scaled);
  const AnyIndex end_index = getGridIndexFromPoint(end_scaled);
  const AnyIndex diff_index = end_index - curr_index_;

  current_step_ = 0;

  ray_length_in_steps_ = std::abs(diff_index.x()) + std::abs(diff_index.y()) +
                         std::abs(diff_index.z());

  const Ray ray_scaled = end_scaled - start_scaled;

  ray_step_signs_ = AnyIndex(signum(ray_scaled.x()), signum(ray_scaled.y()),
                             signum(ray_scaled.z()));

  const AnyIndex corrected_step(std::max(0, ray_step_signs_.x()),
                                std::max(0, ray_step_signs_.y()),
                                std::max(0, ray_step_signs_.z()));

  const Point start_scaled_shifted =
      start_scaled - curr_index_.cast<FloatingPoint>();

  Ray distance_to_boundaries(corrected_step.cast<FloatingPoint>() -
                             start_scaled_shifted);

  t_to_next_boundary_ =
      Ray((std::abs(ray_scaled.x()) < 0.0) ? 2.0 : distance_to_boundaries.x() /
                                                       ray_scaled.x(),
          (std::abs(ray_scaled.y()) < 0.0) ? 2.0 : distance_to_boundaries.y() /
                                                       ray_scaled.y(),
          (std::abs(ray_scaled.z()) < 0.0) ? 2.0 : distance_to_boundaries.z() /
                                                       ray_scaled.z());

  // Distance to cross one grid cell along the ray in t.
  // Same as absolute inverse value of delta_coord.
  t_step_size_ = Ray(
      (std::abs(ray_scaled.x()) < 0.0) ? 2.0
                                       : ray_step_signs_.x() / ray_scaled.x(),
      (std::abs(ray_scaled.y()) < 0.0) ? 2.0
                                       : ray_step_signs_.y() / ray_scaled.y(),
      (std::abs(ray_scaled.z()) < 0.0) ? 2.0
                                       : ray_step_signs_.z() / ray_scaled.z());
}

}  // namespace voxblox

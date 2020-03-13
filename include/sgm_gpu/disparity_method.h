/***********************************************************************
  Copyright (C) 2020 Hironori Fujimoto

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
 
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
***********************************************************************/
#ifndef SGM_GPU__DISPARITY_METHOD_H_
#define SGM_GPU__DISPARITY_METHOD_H_

#include <stdint.h>
#include <opencv2/opencv.hpp>
#include "sgm_gpu/util.h"
#include "sgm_gpu/configuration.h"
#include "sgm_gpu/costs.h"
#include "sgm_gpu/hamming_cost.h"
#include "sgm_gpu/median_filter.h"
#include "sgm_gpu/cost_aggregation.h"
#include "sgm_gpu/left_right_consistency.h"

namespace sgm_gpu
{

void init_disparity_method(const uint8_t _p1, const uint8_t _p2);
void compute_disparity_method(cv::Mat left, cv::Mat right, cv::Mat* disparity, float *elapsed_time_ms, bool check_consistency);
void finish_disparity_method();
static void free_memory();

}

#endif // SGM_GPU__DISPARITY_METHOD_H_

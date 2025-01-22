// Copyright (c) 2024，D-Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "include/parser.h"

#include <opencv2/opencv.hpp>
#include <opencv2/dnn/dnn.hpp>
#include <memory>

using hobot::dnn_node::DNNTensor;

namespace hobot {
namespace dnn_node {
namespace dnn_node_sample {
// 算法输出解析参数
struct PTQYolo8Config {
  std::vector<int> strides;
  std::vector<std::vector<std::pair<double, double>>> anchors_table;
  int class_num;
  std::vector<std::string> class_names;
  std::vector<std::vector<float>> dequantize_scale;
};

float score_threshold_ = 0.4;
float conf_thres_raw = -log(1 / score_threshold_ - 1);
float nms_threshold_ = 0.5;
int nms_top_k_ = 5000;
int kpt_num_ = 4;
int kpt_encode_ = 3;
int class_num_ = 18;
int reg_ = 16;

PTQYolo8Config yolo8_config_ = {
    {8, 16, 32},
    {{{10, 13}, {16, 30}, {33, 23}},
     {{30, 61}, {62, 45}, {59, 119}},
     {{116, 90}, {156, 198}, {373, 326}}},
    80,
    {"blue_sentry_0", "blue_1", "blue_2", "blue_3", "blue_4","blue_5","blue_outpost_6", "blue_7", "blue_base_8",
    "red_sentry_0", "red_1", "red_2", "red_3", "red_4","red_5","red_outpost_6", "red_7", "red_base_8"}};


void ParseTensor(std::shared_ptr<hobot::dnn_node::DnnNodeOutput> tensor,
                 std::vector<YoloV8Result> &results,
                 std::vector<int> &order,
                 std::vector<cv::Rect2d> &bboxes,
                 std::vector<float> &scores);


int get_tensor_hw(std::shared_ptr<DNNTensor> tensor, int *height, int *width);

/**
 * Finds the greatest element in the range [first, last)
 * @tparam[in] ForwardIterator: iterator type
 * @param[in] first: fist iterator
 * @param[in] last: last iterator
 * @return Iterator to the greatest element in the range [first, last)
 */
template <class ForwardIterator>
inline size_t argmax(ForwardIterator first, ForwardIterator last) {
  return std::distance(first, std::max_element(first, last));
}

void ParseTensor(std::shared_ptr<hobot::dnn_node::DnnNodeOutput> node_output,
                 std::vector<YoloV8Result> &results,
                 std::vector<int> &order,
                 std::vector<cv::Rect2d> &bboxes,
                 std::vector<float> &scores) {
  hbSysFlushMem(&(node_output->output_tensors[order[0]]->sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
  hbSysFlushMem(&(node_output->output_tensors[order[1]]->sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
  hbSysFlushMem(&(node_output->output_tensors[order[2]]->sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);


  //TODO
  int w_ = 640 / order[3];
  int h_ = 640 / order[3];
  float scale_ = float(order[3]);

  // 7.1.3 将BPU推理完的内存地址转换为对应类型的指针
  // 7.1.3 Convert the memory address of BPU inference to a pointer of the corresponding type
  auto *bbox_raw = reinterpret_cast<int32_t *>(node_output->output_tensors[order[0]]->sysMem[0].virAddr);
  auto *cls_raw = reinterpret_cast<float *>(node_output->output_tensors[order[1]]->sysMem[0].virAddr);
  auto *kpts_raw = reinterpret_cast<float *>(node_output->output_tensors[order[2]]->sysMem[0].virAddr);
  auto *bbox_scale = reinterpret_cast<float *>(node_output->output_tensors[order[0]]->properties.scale.scaleData);
  for (int h = 0; h < h_; h++)
  {
      for (int w = 0; w < w_; w++)
      {
          float *cur_cls_raw = cls_raw;
          int32_t *cur_bbox_raw = bbox_raw;
          float *cur_kpts_raw = kpts_raw;
          cls_raw += class_num_;
          bbox_raw += reg_ * 4;
          kpts_raw += kpt_num_ * kpt_encode_; 

          // 7.1.5 找到分数的最大值索引, 如果最大值小于阈值，则舍去
          // 7.1.5 Find the index of the maximum score value and discard if the maximum value is less than the threshold
          int cls_id = 0;
          for (int i = 1; i < class_num_; i++)
          {
              if (cur_cls_raw[i] > cur_cls_raw[cls_id])
              {
                  cls_id = i;
              }
          }

          // 7.1.6 不合格则直接跳过, 避免无用的反量化, DFL和dist2bbox计算
          // 7.1.6 If not qualified, skip to avoid unnecessary dequantization, DFL and dist2bbox calculation
          if (cur_cls_raw[cls_id] < conf_thres_raw)
          {
              continue;
          }

          // 7.1.7 计算这个目标的分数
          // 7.1.7 Calculate the score of the target
          float score = 1 / (1 + std::exp(-cur_cls_raw[cls_id]));

          // 7.1.8 对bbox_raw信息进行反量化, DFL计算
          // 7.1.8 Dequantize bbox_raw information, DFL calculation
          float ltrb[4], sum, dfl;
          for (int i = 0; i < 4; i++)
          {
              ltrb[i] = 0.;
              sum = 0.;
              for (int j = 0; j < reg_; j++)
              {
                  dfl = std::exp(float(cur_bbox_raw[reg_ * i + j]) * bbox_scale[reg_ * i + j]);
                  ltrb[i] += dfl * j;
                  sum += dfl;
              }
              ltrb[i] /= sum;
          }

          // 7.1.9 剔除不合格的框   if(x1 >= x2 || y1 >=y2) continue;
          // 7.1.9 Remove unqualified boxes
          if (ltrb[2] + ltrb[0] <= 0 || ltrb[3] + ltrb[1] <= 0)
          {
              continue;
          }

          // 7.1.10 dist 2 bbox (ltrb 2 xyxy)
          float x1 = (w + 0.5 - ltrb[0]) * scale_;
          float y1 = (h + 0.5 - ltrb[1]) * scale_;
          float x2 = (w + 0.5 + ltrb[2]) * scale_;
          float y2 = (h + 0.5 + ltrb[3]) * scale_;

          // 7.1.11 kpts的处理
          // 7.1.11 Process kpts
          std::vector<float> kpts(kpt_num_ * kpt_encode_);
          // std::vector<cv::Point2f> kpt_xy(KPT_NUM);
          // std::vector<float> kpt_score(KPT_NUM);
          for (int j = 0; j < kpt_num_; j++)
          {
              float x = (cur_kpts_raw[kpt_encode_ * j] * 2.0 + w) * scale_;
              float y = (cur_kpts_raw[kpt_encode_ * j + 1] * 2.0 + h) * scale_;

              // kpt_xy[j] = cv::Point2f(x, y);
              // kpt_score[j] = cur_kpts_raw[kpt_encode_ * j + 2];
              kpts[3 * j] = x;
              kpts[3 * j + 1] = y;
              kpts[3 * j + 2] = cur_kpts_raw[kpt_encode_ * j + 2];
            //   std::cout<<"x: "<<x<<" y: "<<y<<std::endl; 
          }

          // 7.1.12 对应类别加入到对应的std::vector中
          // 7.1.12 Add the corresponding class to the corresponding std::vector.
          bboxes.push_back(cv::Rect2d(x1, y1, x2 - x1, y2 - y1));
          scores.push_back(score);
          // kpts_xy.push_back(kpt_xy);
          // kpts_score.push_back(kpt_score);
          YoloV8Result yolov8_result(cls_id, x1, y1, x2, y2, score, kpts, yolo8_config_.class_names[cls_id]);
          results.push_back(yolov8_result);
      }
  }
}


int32_t Parse(
    const std::shared_ptr<hobot::dnn_node::DnnNodeOutput> &node_output,
    std::vector<std::shared_ptr<YoloV8Result>> &results) {

  std::vector<cv::Rect2d> bboxes;
  std::vector<float> scores;  

  std::vector<YoloV8Result> parse_results;

  std::vector<int> order1 = {0, 1, 6, 8};
  ParseTensor(node_output, parse_results, order1, bboxes, scores);

  std::vector<int> order2 = {2, 3, 7, 16};
  ParseTensor(node_output, parse_results, order2, bboxes, scores);

  std::vector<int> order3 = {4, 5, 8, 32};
  ParseTensor(node_output, parse_results, order3, bboxes, scores);



  // for(int i = 0; i < parse_results.size(); i++){
  //   bboxes.push_back(cv::Rect2d(parse_results[i].xmin, parse_results[i].ymin_, parse_results[i].w_, parse_results[i].h_));
  //   scores.push_back(parse_results[i].score);
  // }

  std::vector<int> indices;
  cv::dnn::NMSBoxes(bboxes, scores, score_threshold_, nms_threshold_, indices, 1.f, nms_top_k_);

  for (std::vector<int>::iterator it = indices.begin(); it != indices.end(); ++it)
  {
    auto yolo_res = std::make_shared<YoloV8Result>(parse_results[*it].id,
                                                  parse_results[*it].xmin,
                                                  parse_results[*it].ymin,
                                                  parse_results[*it].xmax,
                                                  parse_results[*it].ymax,
                                                  parse_results[*it].score,
                                                  parse_results[*it].kpts,
                                                  parse_results[*it].class_name);
    results.push_back(yolo_res);
  }

  return 0;
}

}  // namespace dnn_node_sample
}  // namespace dnn_node
}  // namespace hobot
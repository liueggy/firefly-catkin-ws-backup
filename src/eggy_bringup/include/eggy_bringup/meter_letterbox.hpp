#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace eggy_bringup {

struct LetterboxBox {
  float x1;
  float y1;
  float x2;
  float y2;
};

struct LetterboxTransform {
  int source_width;
  int source_height;
  int target_width;
  int target_height;
  int resized_width;
  int resized_height;
  int pad_left;
  int pad_top;
  float scale_x;
  float scale_y;

  static LetterboxTransform make(int source_width,int source_height,int target_width,int target_height){
    if(source_width<=0 || source_height<=0 || target_width<=0 || target_height<=0)
      throw std::invalid_argument("letterbox dimensions must be positive");

    const double scale=std::min(static_cast<double>(target_width)/source_width,
                                static_cast<double>(target_height)/source_height);
    const int resized_width=std::max(1,std::min(target_width,static_cast<int>(std::lround(source_width*scale))));
    const int resized_height=std::max(1,std::min(target_height,static_cast<int>(std::lround(source_height*scale))));
    return {
      source_width,
      source_height,
      target_width,
      target_height,
      resized_width,
      resized_height,
      (target_width-resized_width)/2,
      (target_height-resized_height)/2,
      static_cast<float>(resized_width)/source_width,
      static_cast<float>(resized_height)/source_height
    };
  }

  LetterboxBox to_model(const LetterboxBox& box) const {
    return {
      box.x1*scale_x+pad_left,
      box.y1*scale_y+pad_top,
      box.x2*scale_x+pad_left,
      box.y2*scale_y+pad_top
    };
  }

  LetterboxBox to_source(const LetterboxBox& box) const {
    const float max_x=static_cast<float>(source_width-1);
    const float max_y=static_cast<float>(source_height-1);
    return {
      clamp((box.x1-pad_left)/scale_x,0.f,max_x),
      clamp((box.y1-pad_top)/scale_y,0.f,max_y),
      clamp((box.x2-pad_left)/scale_x,0.f,max_x),
      clamp((box.y2-pad_top)/scale_y,0.f,max_y)
    };
  }

private:
  static float clamp(float value,float lower,float upper){
    return std::max(lower,std::min(value,upper));
  }
};

}  // namespace eggy_bringup

#include <gtest/gtest.h>

#include "eggy_bringup/meter_letterbox.hpp"

namespace {

using eggy_bringup::LetterboxBox;
using eggy_bringup::LetterboxTransform;

void expect_box_near(const LetterboxBox& actual,const LetterboxBox& expected,float tolerance=1e-4f){
  EXPECT_NEAR(actual.x1,expected.x1,tolerance);
  EXPECT_NEAR(actual.y1,expected.y1,tolerance);
  EXPECT_NEAR(actual.x2,expected.x2,tolerance);
  EXPECT_NEAR(actual.y2,expected.y2,tolerance);
}

TEST(MeterLetterbox,RejectsInvalidDimensions){
  EXPECT_THROW(LetterboxTransform::make(0,720,960,960),std::invalid_argument);
  EXPECT_THROW(LetterboxTransform::make(1280,720,0,960),std::invalid_argument);
}

TEST(MeterLetterbox,LandscapeFrameUsesVerticalPadding){
  const auto transform=LetterboxTransform::make(1280,720,960,960);
  EXPECT_EQ(transform.resized_width,960);
  EXPECT_EQ(transform.resized_height,540);
  EXPECT_EQ(transform.pad_left,0);
  EXPECT_EQ(transform.pad_top,210);
  EXPECT_FLOAT_EQ(transform.scale_x,0.75f);
  EXPECT_FLOAT_EQ(transform.scale_y,0.75f);

  expect_box_near(transform.to_source({120.f,285.f,720.f,615.f}),{160.f,100.f,960.f,540.f});
}

TEST(MeterLetterbox,PortraitFrameUsesHorizontalPadding){
  const auto transform=LetterboxTransform::make(720,1280,960,960);
  EXPECT_EQ(transform.resized_width,540);
  EXPECT_EQ(transform.resized_height,960);
  EXPECT_EQ(transform.pad_left,210);
  EXPECT_EQ(transform.pad_top,0);

  expect_box_near(transform.to_source({285.f,120.f,615.f,720.f}),{100.f,160.f,540.f,960.f});
}

TEST(MeterLetterbox,OddDimensionsRoundTripPrecisely){
  const auto transform=LetterboxTransform::make(641,479,960,960);
  const LetterboxBox source{17.25f,31.5f,602.75f,451.25f};
  expect_box_near(transform.to_source(transform.to_model(source)),source,1e-3f);
}

TEST(MeterLetterbox,InverseMappingClampsPaddingToSourceBounds){
  const auto transform=LetterboxTransform::make(1280,720,960,960);
  expect_box_near(transform.to_source({-50.f,0.f,1200.f,1000.f}),{0.f,0.f,1279.f,719.f});
}

}  // namespace

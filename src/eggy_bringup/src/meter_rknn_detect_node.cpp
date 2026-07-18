#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <std_msgs/String.h>
#include <opencv2/opencv.hpp>
#include <rknn_api.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <initializer_list>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "eggy_bringup/meter_letterbox.hpp"

struct Detection{ float x1,y1,x2,y2,score; int class_id; };
static std::string ds(const rknn_tensor_attr& a){ std::ostringstream o; o<<"["; for(uint32_t i=0;i<a.n_dims;i++){ if(i)o<<","; o<<a.dims[i]; } o<<"]"; return o.str(); }
static bool dims_are(const rknn_tensor_attr& a,std::initializer_list<uint32_t> expected){
  if(a.n_dims!=expected.size()) return false;
  size_t i=0; for(uint32_t dim:expected){ if(a.dims[i++]!=dim) return false; }
  return true;
}
static bool rf(const std::string& p,std::vector<uint8_t>& d){ std::ifstream f(p,std::ios::binary|std::ios::ate); if(!f)return false; auto s=f.tellg(); if(s<=0)return false; f.seekg(0); d.resize(s); return (bool)f.read((char*)d.data(),s); }
static float iou(const Detection&a,const Detection&b){ float x1=std::max(a.x1,b.x1),y1=std::max(a.y1,b.y1),x2=std::min(a.x2,b.x2),y2=std::min(a.y2,b.y2),w=std::max(0.f,x2-x1),h=std::max(0.f,y2-y1); float in=w*h,aa=std::max(0.f,a.x2-a.x1)*std::max(0.f,a.y2-a.y1),ab=std::max(0.f,b.x2-b.x1)*std::max(0.f,b.y2-b.y1); return in/(aa+ab-in+1e-6f); }
static bool keep_detection(const Detection& d,int iw,int ih){
  if(d.x2<=d.x1 || d.y2<=d.y1) return false;
  float w = d.x2 - d.x1;
  float h = d.y2 - d.y1;
  float area_ratio = (w * h) / std::max(1.f, (float)iw * (float)ih);
  float aspect = w / std::max(1.f, h);
  bool touches_border = d.x1 <= 0.02f * iw || d.y1 <= 0.02f * ih || d.x2 >= 0.98f * iw || d.y2 >= 0.98f * ih;
  if(d.class_id == 0){
    if(area_ratio > 0.12f) return false;
    if(aspect < 0.55f || aspect > 1.90f) return false;
    if(area_ratio > 0.08f && touches_border) return false;
  }else{
    if(area_ratio > 0.20f) return false;
    if(aspect < 0.45f || aspect > 2.60f) return false;
    if(area_ratio > 0.14f && touches_border) return false;
  }
  return true;
}

class Node{
public:
 Node():pnh_("~"){
  pnh_.param<std::string>("model_path",m_,"/root/meter/best_raw_head_int8_toolkit150.rknn");
  pnh_.param<std::string>("image_topic",t_img_,"/camera/front/image/compressed");
  pnh_.param<std::string>("result_topic",t_json_,"/meter/detection");
  pnh_.param<std::string>("overlay_comp_topic",t_ov_,"/camera/front/image_overlay/compressed");
  pnh_.param<int>("frame_skip",fs_,1);
  pnh_.param<int>("overlay_jpeg_quality",overlay_jpeg_quality_,72);
  overlay_jpeg_quality_=std::max(50,std::min(95,overlay_jpeg_quality_));
  pnh_.param<float>("conf",c_,0.95f);
  pnh_.param<float>("nms",n_,0.45f);
  pnh_.param<int>("max_det",md_,2);
  const std::string resolved_input=nh_.resolveName(t_img_);
  const std::string resolved_overlay=nh_.resolveName(t_ov_);
  if(resolved_input==resolved_overlay) throw std::runtime_error("image_topic and overlay_comp_topic resolve to the same topic: "+resolved_input);
  names_={"pressure_gauge","water_meter"}; load();
  pub_json_=nh_.advertise<std_msgs::String>(t_json_,1);
  pub_comp_=nh_.advertise<sensor_msgs::CompressedImage>(t_ov_,1);
  sub_=nh_.subscribe(t_img_,1,&Node::cb,this);
  ROS_INFO("raw-head node model=%s",m_.c_str());
 }
 ~Node(){ if(ctx_) rknn_destroy(ctx_); }
private:
 void query_or_throw(rknn_query_cmd cmd,void* data,uint32_t size,const std::string& stage){
  int ret=rknn_query(ctx_,cmd,data,size);
  if(ret) throw std::runtime_error(stage+" failed: "+std::to_string(ret));
 }
 void validate_model_contract(){
  if(io_.n_input!=1 || io_.n_output!=3) throw std::runtime_error("model contract mismatch: expected 1 input and 3 outputs, got "+std::to_string(io_.n_input)+" and "+std::to_string(io_.n_output));
  if(in_attr_.fmt!=RKNN_TENSOR_NHWC || in_attr_.type!=RKNN_TENSOR_INT8 || !dims_are(in_attr_,{1,960,960,3}))
    throw std::runtime_error("model input contract mismatch: expected quantized INT8 NHWC [1,960,960,3], got fmt="+std::to_string(in_attr_.fmt)+" type="+std::to_string(in_attr_.type)+" dims="+ds(in_attr_));
  const uint32_t sizes[3]={120,60,30};
  for(uint32_t i=0;i<3;i++){
   if(out_attrs_[i].fmt!=RKNN_TENSOR_NCHW || !dims_are(out_attrs_[i],{1,66,sizes[i],sizes[i]}))
    throw std::runtime_error("model output "+std::to_string(i)+" contract mismatch: expected NCHW [1,66,"+std::to_string(sizes[i])+","+std::to_string(sizes[i])+"], got fmt="+std::to_string(out_attrs_[i].fmt)+" dims="+ds(out_attrs_[i]));
  }
 }
 void load(){
  std::vector<uint8_t> mod; if(!rf(m_,mod)) throw std::runtime_error("read model failed");
  int ret=rknn_init(&ctx_,mod.data(),mod.size(),0,nullptr); if(ret) throw std::runtime_error("rknn_init failed");
  rknn_sdk_version v{}; query_or_throw(RKNN_QUERY_SDK_VERSION,&v,sizeof(v),"query_sdk_version");
  query_or_throw(RKNN_QUERY_IN_OUT_NUM,&io_,sizeof(io_),"query_in_out_num");
  if(io_.n_input!=1 || io_.n_output!=3) throw std::runtime_error("model contract mismatch: expected 1 input and 3 outputs, got "+std::to_string(io_.n_input)+" and "+std::to_string(io_.n_output));
  in_attr_={}; in_attr_.index=0; query_or_throw(RKNN_QUERY_INPUT_ATTR,&in_attr_,sizeof(in_attr_),"query_input_attr");
  out_attrs_.resize(io_.n_output); for(uint32_t i=0;i<io_.n_output;i++){ out_attrs_[i]={}; out_attrs_[i].index=i; query_or_throw(RKNN_QUERY_OUTPUT_ATTR,&out_attrs_[i],sizeof(rknn_tensor_attr),"query_output_attr_"+std::to_string(i)); ROS_INFO("out%u dims=%s",i,ds(out_attrs_[i]).c_str());}
  validate_model_contract();
  ROS_INFO("RKNN contract accepted sdk=%s driver=%s model_input=%s INT8/NHWC runtime_input=RGB/UINT8/NHWC",v.api_version,v.drv_version,ds(in_attr_).c_str());
 }
 std::vector<uint8_t> prp(const cv::Mat& b,const eggy_bringup::LetterboxTransform& transform){ cv::Mat r,g,canvas(transform.target_height,transform.target_width,CV_8UC3,cv::Scalar(114,114,114)); cv::resize(b,r,cv::Size(transform.resized_width,transform.resized_height)); r.copyTo(canvas(cv::Rect(transform.pad_left,transform.pad_top,transform.resized_width,transform.resized_height))); cv::cvtColor(canvas,g,cv::COLOR_BGR2RGB); if(!g.isContinuous()) g=g.clone(); return std::vector<uint8_t>(g.data,g.data+g.total()*g.elemSize()); }
 float dfl(const float* p,int base,int step){ float mx=-1e9f; for(int k=0;k<16;k++) mx=std::max(mx,p[base+k*step]); float s=0,e=0; for(int k=0;k<16;k++){ float v=std::exp(p[base+k*step]-mx); s+=v; e+=v*k; } return e/(s+1e-6f); }
 void decode(const float* p,int H,int W,int st,const eggy_bringup::LetterboxTransform& transform,std::vector<Detection>& de){
  int HW=H*W;
  for(int y=0;y<H;y++) for(int x=0;x<W;x++){ int idx=y*W+x; float l0=p[64*HW+idx],l1=p[65*HW+idx]; float s0=1.f/(1.f+std::exp(-l0)),s1=1.f/(1.f+std::exp(-l1)); int cl=s1>s0?1:0; float sc=std::max(s0,s1); if(sc<c_) continue; float l=dfl(p,0*16*HW+idx,HW),t=dfl(p,1*16*HW+idx,HW),r=dfl(p,2*16*HW+idx,HW),b=dfl(p,3*16*HW+idx,HW); auto box=transform.to_source({(x+0.5f-l)*st,(y+0.5f-t)*st,(x+0.5f+r)*st,(y+0.5f+b)*st}); Detection d{box.x1,box.y1,box.x2,box.y2,sc,cl}; if(d.x2>d.x1&&d.y2>d.y1){ float area=(d.x2-d.x1)*(d.y2-d.y1); if(area<3000) continue; de.push_back(d);} }
 }
 std::vector<Detection> pp(const std::vector<rknn_output>& os,const eggy_bringup::LetterboxTransform& transform){
  std::vector<Detection> de; for(size_t i=0;i<os.size();i++){ if(!os[i].buf) continue; const float* p=(const float*)os[i].buf; int W=i==0?120:i==1?60:30; int H=W; int st=960/W; decode(p,H,W,st,transform,de); }
  std::sort(de.begin(),de.end(),[](auto&a,auto&b){return a.score>b.score;});
  std::vector<Detection> d2; std::vector<char> rm(de.size());
  for(size_t i=0;i<de.size();i++){ if(rm[i]) continue; d2.push_back(de[i]); for(size_t j=i+1;j<de.size();j++) if(!rm[j]&&de[i].class_id==de[j].class_id&&iou(de[i],de[j])>n_) rm[j]=1; }
  // per class primary
  if(!d2.empty()){ std::vector<Detection> d3; int nc=2; for(int ci=0;ci<nc;ci++){ bool f=false; Detection b{}; for(auto&d:d2){if(d.class_id!=ci)continue; if(!f||d.score>b.score){b=d;f=true;}} if(f) d3.push_back(b);} d2.swap(d3); }
  if((int)d2.size()>md_) d2.resize(md_);
  return d2;
 }
 void draw(cv::Mat& im,const std::vector<Detection>& dd){ for(auto&d:dd){ cv::Scalar cl=d.class_id==1?cv::Scalar(0,255,0):cv::Scalar(255,128,0); cv::rectangle(im,{(int)d.x1,(int)d.y1},{(int)d.x2,(int)d.y2},cl,2); std::ostringstream ss; ss.setf(std::ios::fixed); ss<<names_[d.class_id]<<" "<<std::setprecision(2)<<d.score; int b=0; auto ts=cv::getTextSize(ss.str(),cv::FONT_HERSHEY_SIMPLEX,0.55,1,&b); int tx=std::max(0,(int)d.x1),ty=std::max(ts.height+4,(int)d.y1-4); cv::rectangle(im,{tx,ty-ts.height-4},{tx+ts.width+4,ty+b},cl,-1); cv::putText(im,ss.str(),{tx+2,ty-2},cv::FONT_HERSHEY_SIMPLEX,0.55,{0,0,0},1,cv::LINE_AA); } }
 sensor_msgs::CompressedImage cm(const cv::Mat& im,const std_msgs::Header& h){ sensor_msgs::CompressedImage m; m.header=h; m.format="jpeg"; std::vector<int> p={cv::IMWRITE_JPEG_QUALITY,overlay_jpeg_quality_}; cv::imencode(".jpg",im,m.data,p); return m; }
 cv::Mat decode_image(const sensor_msgs::CompressedImage& msg){ cv::Mat buf(1,(int)msg.data.size(),CV_8UC1,const_cast<uint8_t*>(msg.data.data())); return cv::imdecode(buf,cv::IMREAD_COLOR); }
 void publish_cached_overlay(const sensor_msgs::CompressedImage& msg){ cv::Mat im=decode_image(msg); if(im.empty()) return; draw(im,last_detections_); pub_comp_.publish(cm(im,msg.header)); }
 std::string jn(const cv::Mat& im,const std::vector<Detection>& dd,const eggy_bringup::LetterboxTransform& transform,int64_t npu,int64_t tt){ std::ostringstream o; o.setf(std::ios::fixed); o<<std::setprecision(4); o<<"{\"detected\":"<<(dd.empty()?"false":"true")<<",\"frame_id\":"<<f_<<",\"image_width\":"<<im.cols<<",\"image_height\":"<<im.rows<<",\"preprocess\":{\"method\":\"letterbox\",\"target_width\":"<<transform.target_width<<",\"target_height\":"<<transform.target_height<<",\"resized_width\":"<<transform.resized_width<<",\"resized_height\":"<<transform.resized_height<<",\"pad_left\":"<<transform.pad_left<<",\"pad_top\":"<<transform.pad_top<<",\"scale_x\":"<<transform.scale_x<<",\"scale_y\":"<<transform.scale_y<<"},\"npu_us\":"<<npu<<",\"total_ms\":"<<tt<<",\"detections\":["; for(size_t i=0;i<dd.size();i++){ if(i)o<<","; auto&d=dd[i]; o<<"{\"class_id\":"<<d.class_id<<",\"class_name\":\""<<names_[d.class_id]<<"\",\"score\":"<<d.score<<",\"x1\":"<<d.x1<<",\"y1\":"<<d.y1<<",\"x2\":"<<d.x2<<",\"y2\":"<<d.y2<<"}"; } o<<"]}"; return o.str(); }
 void er(const std::string&s,int c){ std_msgs::String m; m.data="{\"detected\":false,\"error_stage\":\""+s+"\",\"code\":"+std::to_string(c)+"}"; pub_json_.publish(m); ROS_ERROR_THROTTLE(2,"%s %d",s.c_str(),c); }
 void cb(const sensor_msgs::CompressedImageConstPtr& msg){ f_++; if(f_%std::max(1,fs_)!=0){ publish_cached_overlay(*msg); return; } auto t0=std::chrono::steady_clock::now(); cv::Mat im=decode_image(*msg); if(im.empty()) return; const auto transform=eggy_bringup::LetterboxTransform::make(im.cols,im.rows,960,960); auto inp=prp(im,transform); rknn_input in{}; in.index=0; in.buf=inp.data(); in.size=inp.size(); in.pass_through=0; in.type=RKNN_TENSOR_UINT8; in.fmt=RKNN_TENSOR_NHWC; int ret=rknn_inputs_set(ctx_,1,&in); if(ret){er("inputs_set",ret);return;} ret=rknn_run(ctx_,nullptr); if(ret){er("rknn_run",ret);return;} std::vector<rknn_output> os(io_.n_output); for(uint32_t i=0;i<io_.n_output;i++){ os[i]={}; os[i].index=i; os[i].want_float=1; } ret=rknn_outputs_get(ctx_,io_.n_output,os.data(),nullptr); if(ret){er("outputs_get",ret);return;} rknn_perf_run perf{}; ret=rknn_query(ctx_,RKNN_QUERY_PERF_RUN,&perf,sizeof(perf)); if(ret){rknn_outputs_release(ctx_,io_.n_output,os.data());er("query_perf_run",ret);return;} auto de=pp(os,transform); last_detections_=de; draw(im,de); std_msgs::String jm; jm.data=jn(im,de,transform,perf.run_duration,std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-t0).count()); pub_json_.publish(jm); pub_comp_.publish(cm(im,msg->header)); rknn_outputs_release(ctx_,io_.n_output,os.data()); }
 ros::NodeHandle nh_,pnh_; ros::Subscriber sub_; ros::Publisher pub_json_,pub_comp_; std::string m_,t_img_,t_json_,t_ov_; std::vector<std::string> names_; std::vector<Detection> last_detections_; int fs_=1,md_=2,overlay_jpeg_quality_=72; float c_=0.95f,n_=0.45f; uint64_t f_=0; rknn_context ctx_=0; rknn_input_output_num io_{}; rknn_tensor_attr in_attr_{}; std::vector<rknn_tensor_attr> out_attrs_;
};
int main(int argc,char**argv){ ros::init(argc,argv,"meter_rknn_detect_cpp"); try{ Node n; ros::spin(); }catch(const std::exception&e){ ROS_FATAL("%s",e.what()); return 1; } return 0; }

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
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

struct Detection{ float x1,y1,x2,y2,score; int class_id; };
static std::string ds(const rknn_tensor_attr& a){ std::ostringstream o; o<<"["; for(uint32_t i=0;i<a.n_dims;i++){ if(i)o<<","; o<<a.dims[i]; } o<<"]"; return o.str(); }
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
  pnh_.param<float>("conf",c_,0.95f);
  pnh_.param<float>("nms",n_,0.45f);
  pnh_.param<int>("max_det",md_,2);
  names_={"pressure_gauge","water_meter"}; load();
  pub_json_=nh_.advertise<std_msgs::String>(t_json_,1);
  pub_comp_=nh_.advertise<sensor_msgs::CompressedImage>(t_ov_,1);
  sub_=nh_.subscribe(t_img_,1,&Node::cb,this);
  ROS_INFO("raw-head node model=%s",m_.c_str());
 }
 ~Node(){ if(ctx_) rknn_destroy(ctx_); }
private:
 void load(){
  std::vector<uint8_t> mod; if(!rf(m_,mod)) throw std::runtime_error("read model failed");
  int ret=rknn_init(&ctx_,mod.data(),mod.size(),0,nullptr); if(ret) throw std::runtime_error("rknn_init failed");
  rknn_sdk_version v{}; rknn_query(ctx_,RKNN_QUERY_SDK_VERSION,&v,sizeof(v));
  ret=rknn_query(ctx_,RKNN_QUERY_IN_OUT_NUM,&io_,sizeof(io_)); if(ret) throw std::runtime_error("query io failed");
  in_attr_={}; in_attr_.index=0; rknn_query(ctx_,RKNN_QUERY_INPUT_ATTR,&in_attr_,sizeof(in_attr_));
  out_attrs_.resize(io_.n_output); for(uint32_t i=0;i<io_.n_output;i++){ out_attrs_[i]={}; out_attrs_[i].index=i; rknn_query(ctx_,RKNN_QUERY_OUTPUT_ATTR,&out_attrs_[i],sizeof(rknn_tensor_attr)); ROS_INFO("out%u dims=%s",i,ds(out_attrs_[i]).c_str());}
 }
 std::vector<uint8_t> prp(const cv::Mat& b){ cv::Mat r,g; cv::resize(b,r,cv::Size(960,960)); cv::cvtColor(r,g,cv::COLOR_BGR2RGB); if(!g.isContinuous()) g=g.clone(); return std::vector<uint8_t>(g.data,g.data+g.total()*g.elemSize()); }
 float dfl(const float* p,int base,int step){ float mx=-1e9f; for(int k=0;k<16;k++) mx=std::max(mx,p[base+k*step]); float s=0,e=0; for(int k=0;k<16;k++){ float v=std::exp(p[base+k*step]-mx); s+=v; e+=v*k; } return e/(s+1e-6f); }
 void decode(const float* p,int H,int W,int st,int iw,int ih,std::vector<Detection>& de){
  int HW=H*W; float sx=(float)iw/960.f,sy=(float)ih/960.f;
  for(int y=0;y<H;y++) for(int x=0;x<W;x++){ int idx=y*W+x; float l0=p[64*HW+idx],l1=p[65*HW+idx]; float s0=1.f/(1.f+std::exp(-l0)),s1=1.f/(1.f+std::exp(-l1)); int cl=s1>s0?1:0; float sc=std::max(s0,s1); if(sc<c_) continue; float l=dfl(p,0*16*HW+idx,HW),t=dfl(p,1*16*HW+idx,HW),r=dfl(p,2*16*HW+idx,HW),b=dfl(p,3*16*HW+idx,HW); Detection d; d.x1=(x+0.5f-l)*st*sx; d.y1=(y+0.5f-t)*st*sy; d.x2=(x+0.5f+r)*st*sx; d.y2=(y+0.5f+b)*st*sy; d.x1=std::max(0.f,std::min(d.x1,(float)iw-1)); d.y1=std::max(0.f,std::min(d.y1,(float)ih-1)); d.x2=std::max(0.f,std::min(d.x2,(float)iw-1)); d.y2=std::max(0.f,std::min(d.y2,(float)ih-1)); if(d.x2>d.x1&&d.y2>d.y1){ float area=(d.x2-d.x1)*(d.y2-d.y1); if(area<3000) continue; d.score=sc; d.class_id=cl; de.push_back(d);} }
 }
 std::vector<Detection> pp(const std::vector<rknn_output>& os,int iw,int ih){
  std::vector<Detection> de; for(size_t i=0;i<os.size();i++){ if(!os[i].buf) continue; const float* p=(const float*)os[i].buf; int W=i==0?120:i==1?60:30; int H=W; int st=960/W; decode(p,H,W,st,iw,ih,de); }
  std::sort(de.begin(),de.end(),[](auto&a,auto&b){return a.score>b.score;});
  std::vector<Detection> d2; std::vector<char> rm(de.size());
  for(size_t i=0;i<de.size();i++){ if(rm[i]) continue; d2.push_back(de[i]); for(size_t j=i+1;j<de.size();j++) if(!rm[j]&&de[i].class_id==de[j].class_id&&iou(de[i],de[j])>n_) rm[j]=1; }
  // per class primary
  if(!d2.empty()){ std::vector<Detection> d3; int nc=2; for(int ci=0;ci<nc;ci++){ bool f=false; Detection b{}; for(auto&d:d2){if(d.class_id!=ci)continue; if(!f||d.score>b.score){b=d;f=true;}} if(f) d3.push_back(b);} d2.swap(d3); }
  if((int)d2.size()>md_) d2.resize(md_);
  return d2;
 }
 void draw(cv::Mat& im,const std::vector<Detection>& dd){ for(auto&d:dd){ cv::Scalar cl=d.class_id==1?cv::Scalar(0,255,0):cv::Scalar(255,128,0); cv::rectangle(im,{(int)d.x1,(int)d.y1},{(int)d.x2,(int)d.y2},cl,2); std::ostringstream ss; ss.setf(std::ios::fixed); ss<<names_[d.class_id]<<" "<<std::setprecision(2)<<d.score; int b=0; auto ts=cv::getTextSize(ss.str(),cv::FONT_HERSHEY_SIMPLEX,0.55,1,&b); int tx=std::max(0,(int)d.x1),ty=std::max(ts.height+4,(int)d.y1-4); cv::rectangle(im,{tx,ty-ts.height-4},{tx+ts.width+4,ty+b},cl,-1); cv::putText(im,ss.str(),{tx+2,ty-2},cv::FONT_HERSHEY_SIMPLEX,0.55,{0,0,0},1,cv::LINE_AA); } }
 sensor_msgs::CompressedImage cm(const cv::Mat& im,const std_msgs::Header& h){ sensor_msgs::CompressedImage m; m.header=h; m.format="jpeg"; std::vector<int> p={cv::IMWRITE_JPEG_QUALITY,85}; cv::imencode(".jpg",im,m.data,p); return m; }
 std::string jn(const cv::Mat& im,const std::vector<Detection>& dd,int64_t npu,int64_t tt){ std::ostringstream o; o.setf(std::ios::fixed); o<<std::setprecision(4); o<<"{\"detected\":"<<(dd.empty()?"false":"true")<<",\"frame_id\":"<<f_<<",\"image_width\":"<<im.cols<<",\"image_height\":"<<im.rows<<",\"npu_us\":"<<npu<<",\"total_ms\":"<<tt<<",\"detections\":["; for(size_t i=0;i<dd.size();i++){ if(i)o<<","; auto&d=dd[i]; o<<"{\"class_id\":"<<d.class_id<<",\"class_name\":\""<<names_[d.class_id]<<"\",\"score\":"<<d.score<<",\"x1\":"<<d.x1<<",\"y1\":"<<d.y1<<",\"x2\":"<<d.x2<<",\"y2\":"<<d.y2<<"}"; } o<<"]}"; return o.str(); }
 void er(const std::string&s,int c){ std_msgs::String m; m.data="{\"detected\":false,\"error_stage\":\""+s+"\",\"code\":"+std::to_string(c)+"}"; pub_json_.publish(m); ROS_ERROR_THROTTLE(2,"%s %d",s.c_str(),c); }
 void cb(const sensor_msgs::CompressedImageConstPtr& msg){ f_++; if(f_%std::max(1,fs_)!=0) return; auto t0=std::chrono::steady_clock::now(); cv::Mat buf(1,(int)msg->data.size(),CV_8UC1,const_cast<uint8_t*>(msg->data.data())); cv::Mat im=cv::imdecode(buf,cv::IMREAD_COLOR); if(im.empty()) return; auto inp=prp(im); rknn_input in{}; in.index=0; in.buf=inp.data(); in.size=inp.size(); in.pass_through=0; in.type=RKNN_TENSOR_UINT8; in.fmt=RKNN_TENSOR_NHWC; int ret=rknn_inputs_set(ctx_,1,&in); if(ret){er("inputs_set",ret);return;} ret=rknn_run(ctx_,nullptr); if(ret) ROS_WARN_THROTTLE(2,"rknn_run %d",ret); std::vector<rknn_output> os(io_.n_output); for(uint32_t i=0;i<io_.n_output;i++){ os[i]={}; os[i].index=i; os[i].want_float=1; } ret=rknn_outputs_get(ctx_,io_.n_output,os.data(),nullptr); if(ret){er("outputs_get",ret);return;} rknn_perf_run perf{}; rknn_query(ctx_,RKNN_QUERY_PERF_RUN,&perf,sizeof(perf)); auto de=pp(os,im.cols,im.rows); draw(im,de); std_msgs::String jm; jm.data=jn(im,de,perf.run_duration,std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-t0).count()); pub_json_.publish(jm); pub_comp_.publish(cm(im,msg->header)); rknn_outputs_release(ctx_,io_.n_output,os.data()); }
 ros::NodeHandle nh_,pnh_; ros::Subscriber sub_; ros::Publisher pub_json_,pub_comp_; std::string m_,t_img_,t_json_,t_ov_; std::vector<std::string> names_; int fs_=1,md_=2; float c_=0.95f,n_=0.45f; uint64_t f_=0; rknn_context ctx_=0; rknn_input_output_num io_{}; rknn_tensor_attr in_attr_{}; std::vector<rknn_tensor_attr> out_attrs_;
};
int main(int argc,char**argv){ ros::init(argc,argv,"meter_rknn_detect_cpp"); try{ Node n; ros::spin(); }catch(const std::exception&e){ ROS_FATAL("%s",e.what()); return 1; } return 0; }

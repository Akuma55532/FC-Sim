#include "flight_rviz_plugins/flight_panel.hpp"
#include "flight_rviz_plugins/waypoint_file.hpp"
#include "flight_rviz_plugins/trajectory.hpp"
#include "flight_rviz_plugins/quadrotor_model.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/expand_topic_or_service_name.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <flight_interfaces/action/arm.hpp>
#include <flight_interfaces/action/disarm.hpp>
#include <flight_interfaces/action/takeoff.hpp>
#include <flight_interfaces/action/land.hpp>
#include <flight_interfaces/action/execute_waypoints.hpp>
#include <flight_interfaces/msg/control_status.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QFileDialog>
#include <QFile>
#include <QSaveFile>
#include <QHeaderView>
#include <QLabel>
#include <QFrame>
#include <QHBoxLayout>
#include <QMap>
#include <QLineEdit>
#include <QPushButton>
#include <QTableWidget>
#include <QTimer>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QScrollArea>
#include <QSignalBlocker>
#include <atomic>
#include <chrono>
#include <cmath>

namespace flight_rviz_plugins {
using Arm=flight_interfaces::action::Arm;
using Disarm=flight_interfaces::action::Disarm;
using Takeoff=flight_interfaces::action::Takeoff;
using Land=flight_interfaces::action::Land;
using Mission=flight_interfaces::action::ExecuteWaypoints;
using Status=flight_interfaces::msg::ControlStatus;
constexpr double pi=3.14159265358979323846;
static double yaw(const geometry_msgs::msg::Quaternion & q) {
  return std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
}
struct FlightPanel::Impl {
  FlightPanel * owner;
  QLineEdit * endpoint;
  QLabel * status_label,*result_label;
  QLabel *connection_badge,*telemetry_badge,*pose_badge,*armed_badge;
  QLabel *mode_value,*action_value,*phase_value,*ground_value,*position_values[4],*task_value,*detail_value;
  QFrame *status_card;
  QTableWidget * table;
  QDoubleSpinBox *height,*default_z,*position_tol,*yaw_tol,*dwell,*timeout;
  QCheckBox *check_yaw,*loop,*show_trajectory,*show_vehicle;
  Trajectory trajectory;
  QuadrotorModel vehicle_model;
  QSpinBox *capacity;
  QPushButton *arm_button,*disarm_button,*takeoff_button,*land_button,*run_button,*step_button,*stop_button,*bind_button;
  QTimer *timer;
  rclcpp::Node::SharedPtr node;
  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp_action::Client<Arm>::SharedPtr arm;
  rclcpp_action::Client<Disarm>::SharedPtr disarm;
  rclcpp_action::Client<Takeoff>::SharedPtr takeoff;
  rclcpp_action::Client<Land>::SharedPtr land;
  rclcpp_action::Client<Mission>::SharedPtr mission;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop;
  rclcpp::Subscription<Status>::SharedPtr status_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr clicked_sub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers;
  Status state;
  bool seen{},land_pending{},stop_pending{},dirty{true};
  int pending{};
  uint64_t revision{},generation{};
  std::chrono::steady_clock::time_point received;
  std::string core{"/flight_core"};
  Document document;
  std::vector<Point> executing;

  QPushButton * button(QGridLayout *layout,const QString &text,int row,int col,const char *name) {
    auto *b=new QPushButton(text);b->setObjectName(name);layout->addWidget(b,row,col);return b;
  }
  QDoubleSpinBox * number(QGridLayout *layout,const QString &text,int row,int col,double value,double min,double max) {
    layout->addWidget(new QLabel(text),row,col);
    auto *v=new QDoubleSpinBox;v->setDecimals(3);v->setRange(min,max);v->setValue(value);
    layout->addWidget(v,row,col+1);return v;
  }
  explicit Impl(FlightPanel *p):owner(p) {
    auto *outer=new QVBoxLayout(p);auto *scroll=new QScrollArea;scroll->setWidgetResizable(true);
    auto *body=new QWidget;auto *layout=new QVBoxLayout(body);outer->addWidget(scroll);scroll->setWidget(body);
    auto *connection=new QGridLayout;endpoint=new QLineEdit("/flight_core");endpoint->setObjectName("core_namespace");
    connection->addWidget(new QLabel("控制核心"),0,0);connection->addWidget(endpoint,0,1);
    bind_button=button(connection,"连接",0,2,"connect");layout->addLayout(connection);
    auto *notice=new QLabel("坐标：map / ENU（米）；偏航输入：度。\n关闭面板不会停止飞行或已提交任务。停止任务保持当前位置，降落请使用降落按钮。");
    notice->setWordWrap(true);layout->addWidget(notice);
    status_card=new QFrame;status_card->setObjectName("flight_status_card");
    status_card->setStyleSheet("QFrame#flight_status_card {background:#f8fafc;border:1px solid #dbe3ed;border-radius:10px;} "
      "QFrame#flight_status_card QLabel {background:transparent;border:0;color:#243247;}");
    auto *card=new QVBoxLayout(status_card);card->setContentsMargins(14,12,14,12);card->setSpacing(10);
    auto *heading=new QHBoxLayout;auto *title=new QLabel("飞行状态");title->setStyleSheet("font-size:16px;font-weight:600;");
    heading->addWidget(title);heading->addStretch();
    status_label=new QLabel("等待遥测");status_label->setObjectName("status");status_label->setTextFormat(Qt::PlainText);
    heading->addWidget(status_label);card->addLayout(heading);
    auto *badges=new QGridLayout;badges->setSpacing(6);
    const auto badge=[badges](int row,int col){auto *v=new QLabel;v->setAlignment(Qt::AlignCenter);v->setTextFormat(Qt::PlainText);badges->addWidget(v,row,col);return v;};
    connection_badge=badge(0,0);telemetry_badge=badge(0,1);pose_badge=badge(1,0);armed_badge=badge(1,1);card->addLayout(badges);
    auto *facts=new QGridLayout;facts->setHorizontalSpacing(16);facts->setVerticalSpacing(4);
    const auto fact=[facts](const QString &name,int col){auto *label=new QLabel(name);label->setStyleSheet("color:#66768b;font-size:11px;");facts->addWidget(label,0,col);
      auto *v=new QLabel("—");v->setTextFormat(Qt::PlainText);v->setWordWrap(true);v->setStyleSheet("font-weight:600;font-size:13px;");facts->addWidget(v,1,col);facts->setColumnStretch(col,1);return v;};
    mode_value=fact("飞控模式",0);action_value=fact("当前操作",1);phase_value=fact("执行阶段",2);ground_value=fact("着陆状态",3);card->addLayout(facts);
    auto *line=new QFrame;line->setFrameShape(QFrame::HLine);line->setStyleSheet("background:#dbe3ed;max-height:1px;");card->addWidget(line);
    auto *pose_grid=new QGridLayout;const QString names[]={"X · 东 / m","Y · 北 / m","Z · 上 / m","Yaw / °"};
    for(int i=0;i<4;++i){auto *label=new QLabel(names[i]);label->setStyleSheet("color:#66768b;font-size:11px;");pose_grid->addWidget(label,0,i);
      position_values[i]=new QLabel("—");position_values[i]->setObjectName(QString("status_position_%1").arg(i));position_values[i]->setStyleSheet("font-size:17px;font-weight:600;font-family:monospace;");
      pose_grid->addWidget(position_values[i],1,i);pose_grid->setColumnStretch(i,1);}
    card->addLayout(pose_grid);
    task_value=new QLabel("暂无航点任务");task_value->setWordWrap(true);task_value->setTextFormat(Qt::PlainText);
    task_value->setStyleSheet("color:#52657d;font-size:12px;");card->addWidget(task_value);
    detail_value=new QLabel;detail_value->setObjectName("status_detail");detail_value->setWordWrap(true);detail_value->setTextFormat(Qt::PlainText);detail_value->setTextInteractionFlags(Qt::TextSelectableByMouse);card->addWidget(detail_value);
    layout->addWidget(status_card);
    show_vehicle=new QCheckBox("显示无人机模型（红色箭头为机头）");show_vehicle->setObjectName("show_vehicle");show_vehicle->setChecked(true);
    layout->addWidget(show_vehicle);QObject::connect(show_vehicle,&QCheckBox::toggled,p,[this]{dirty=true;});
    auto *trail_controls=new QHBoxLayout;
    show_trajectory=new QCheckBox("显示实际轨迹（青绿色）");show_trajectory->setObjectName("show_trajectory");show_trajectory->setChecked(true);
    auto *clear_trail=new QPushButton("清空轨迹");clear_trail->setObjectName("clear_trajectory");
    trail_controls->addWidget(show_trajectory);trail_controls->addStretch();trail_controls->addWidget(clear_trail);layout->addLayout(trail_controls);
    QObject::connect(show_trajectory,&QCheckBox::toggled,p,[this]{dirty=true;});
    QObject::connect(clear_trail,&QPushButton::clicked,p,[this]{trajectory.clear();dirty=true;});
    result_label=new QLabel("操作反馈 · 尚未发送命令");result_label->setObjectName("result");result_label->setWordWrap(true);result_label->setTextFormat(Qt::PlainText);
    result_label->setStyleSheet("QLabel {background:#eef2f7;color:#334155;border-radius:6px;padding:8px;font-size:12px;}");layout->addWidget(result_label);
    auto *controls=new QGridLayout;
    arm_button=button(controls,"解锁",0,0,"arm");disarm_button=button(controls,"上锁",0,1,"disarm");
    takeoff_button=button(controls,"起飞",0,2,"takeoff");land_button=button(controls,"降落",0,3,"land");
    height=number(controls,"起飞相对高度",1,0,2,0.01,10);layout->addLayout(controls);
    auto *settings=new QGridLayout;
    default_z=number(settings,"新增点默认 Z",0,0,1,-10000,10000);
    position_tol=number(settings,"XYZ 阈值 (m)",1,0,0.1,0.001,10);
    yaw_tol=number(settings,"Yaw 阈值 (°)",1,2,10,0.001,180);
    dwell=number(settings,"到点停留 (s)",2,0,0.5,0,3600);
    timeout=number(settings,"单点超时 (s)",2,2,60,0.001,86400);
    check_yaw=new QCheckBox("检查偏航到达");check_yaw->setObjectName("check_yaw");check_yaw->setChecked(true);settings->addWidget(check_yaw,3,0,1,2);
    loop=new QCheckBox("循环执行");settings->addWidget(loop,3,2);
    capacity=new QSpinBox;capacity->setRange(1,500);capacity->setValue(50);
    settings->addWidget(new QLabel("最大航点数"),0,2);settings->addWidget(capacity,0,3);layout->addLayout(settings);
    table=new QTableWidget(0,4);table->setObjectName("waypoints");table->setHorizontalHeaderLabels({"X (m)","Y (m)","Z (m)","Yaw (°)"});
    table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);table->setSelectionBehavior(QAbstractItemView::SelectRows);
    table->setSelectionMode(QAbstractItemView::SingleSelection);table->setMinimumHeight(190);layout->addWidget(table);
    auto *edit=new QGridLayout;
    auto *add=button(edit,"添加",0,0,"add");auto *insert=button(edit,"插入",0,1,"insert");
    auto *remove=button(edit,"删除",0,2,"delete");auto *up=button(edit,"上移",0,3,"up");auto *down=button(edit,"下移",0,4,"down");
    auto *current=button(edit,"添加当前位置",1,0,"add_current");
    auto *save=button(edit,"保存 YAML",1,1,"save_file");auto *load=button(edit,"加载 YAML",1,2,"load_file");layout->addLayout(edit);
    auto *tasks=new QGridLayout;run_button=button(tasks,"执行全部",0,0,"execute");step_button=button(tasks,"单步执行选中点",0,1,"step");
    stop_button=button(tasks,"停止 / 保持",0,2,"stop");layout->addLayout(tasks);
    QObject::connect(add,&QPushButton::clicked,p,[this]{append({0,0,default_z->value(),0});});
    QObject::connect(current,&QPushButton::clicked,p,[this]{if(!fresh() || !state.pose_valid){message("当前位置不可用");return;}
      const auto &q=state.current_pose;append({q.position.x,q.position.y,q.position.z,yaw(q.orientation)*180/pi});});
    QObject::connect(insert,&QPushButton::clicked,p,[this]{append({0,0,default_z->value(),0},table->currentRow());});
    QObject::connect(remove,&QPushButton::clicked,p,[this]{if(table->currentRow()>=0){table->removeRow(table->currentRow());changed();}});
    QObject::connect(up,&QPushButton::clicked,p,[this]{move(-1);});QObject::connect(down,&QPushButton::clicked,p,[this]{move(1);});
    QObject::connect(table,&QTableWidget::cellChanged,p,[this](int,int){changed();});
    QObject::connect(check_yaw,&QCheckBox::toggled,p,[this](bool on){yaw_tol->setEnabled(on);});
    QObject::connect(bind_button,&QPushButton::clicked,p,[this]{bind();});
    QObject::connect(save,&QPushButton::clicked,p,[this]{save_file();});QObject::connect(load,&QPushButton::clicked,p,[this]{load_file();});
    QObject::connect(arm_button,&QPushButton::clicked,p,[this]{submit<Arm>(arm,Arm::Goal{},false);});
    QObject::connect(disarm_button,&QPushButton::clicked,p,[this]{submit<Disarm>(disarm,Disarm::Goal{},false);});
    QObject::connect(takeoff_button,&QPushButton::clicked,p,[this]{Takeoff::Goal g;g.height_m=height->value();submit<Takeoff>(takeoff,g,false);});
    QObject::connect(land_button,&QPushButton::clicked,p,[this]{submit<Land>(land,Land::Goal{},true);});
    QObject::connect(run_button,&QPushButton::clicked,p,[this]{run(false);});QObject::connect(step_button,&QPushButton::clicked,p,[this]{run(true);});
    QObject::connect(stop_button,&QPushButton::clicked,p,[this]{request_stop();});
    static std::atomic<unsigned> serial{0};
    rclcpp::NodeOptions options;options.use_global_arguments(false);
    node=std::make_shared<rclcpp::Node>("flight_panel_"+std::to_string(++serial),options);executor.add_node(node);
    clicked_sub=node->create_subscription<geometry_msgs::msg::PoseStamped>("/flight_panel/goal",10,[this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m){
      const auto &q=m->pose.orientation;const double n=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
      if(m->header.frame_id!="map" || !std::isfinite(n) || std::abs(n-1)>0.02){message("点选需要 map 帧和有效单位四元数");return;}
      append({m->pose.position.x,m->pose.position.y,default_z->value(),yaw(q)*180/pi});});
    bind();
    timer=new QTimer(p);QObject::connect(timer,&QTimer::timeout,p,[this]{
      executor.spin_some(std::chrono::milliseconds(2));if(!fresh())trajectory.break_segment();update_buttons();if(dirty || !fresh()){publish_markers();dirty=false;}});timer->start(50);
  }
  ~Impl(){timer->stop();executor.remove_node(node);}
  void message(const QString &text){result_label->setText(text);}
  bool fresh() const {
    if(!seen || std::chrono::steady_clock::now()-received>=std::chrono::seconds(1))return false;
    const double age=node->now().seconds()-rclcpp::Time(state.header.stamp).seconds();
    return age>=-0.1 && age<=1.0;
  }
  void changed(){++revision;dirty=true;}
  void bind() {
    if((pending || stop_pending) && fresh()){message("请求尚未结束，不能切换核心");return;}
    std::string next=endpoint->text().trimmed().toStdString();while(next.size()>1 && next.back()=='/')next.pop_back();
    if(next.empty() || next.front()!='/' || next=="/"){message("请输入绝对核心名称，例如 /flight_core");return;}
    try {
      // Validate before replacing any clients.
      rclcpp::expand_topic_or_service_name(next+"/control_status",node->get_name(),node->get_namespace());
      if(status_sub && next!=core) {
        document[core]=draft();
        set_draft(document.count(next)?document.at(next):Vehicle{});
      }
      if(markers){visualization_msgs::msg::MarkerArray a;visualization_msgs::msg::Marker m;m.action=m.DELETEALL;a.markers.push_back(m);markers->publish(a);}
      pending=0;stop_pending=false;land_pending=false;
      core=next;endpoint->setText(QString::fromStdString(core));++generation;seen=false;executing.clear();trajectory.clear();vehicle_model.clear();
      arm=rclcpp_action::create_client<Arm>(node,core+"/arm");disarm=rclcpp_action::create_client<Disarm>(node,core+"/disarm");
      takeoff=rclcpp_action::create_client<Takeoff>(node,core+"/takeoff");land=rclcpp_action::create_client<Land>(node,core+"/land_action");
      mission=rclcpp_action::create_client<Mission>(node,core+"/execute_waypoints");stop=node->create_client<std_srvs::srv::Trigger>(core+"/stop");
      auto qos=rclcpp::QoS(1).reliable().transient_local();
      status_sub=node->create_subscription<Status>(core+"/control_status",qos,[this](Status::ConstSharedPtr s){if(!fresh())trajectory.break_segment();
        state=*s;seen=true;received=std::chrono::steady_clock::now();dirty=true;
        vehicle_model.update(state.current_pose,fresh() && state.connected && state.telemetry_valid && state.pose_valid && state.header.frame_id=="map");
        trajectory.sample(state.current_pose.position,rclcpp::Time(state.header.stamp).seconds(),
          fresh() && state.connected && state.telemetry_valid && state.pose_valid && state.header.frame_id=="map");});
      markers=node->create_publisher<visualization_msgs::msg::MarkerArray>(core+"/panel_markers",qos);
      dirty=true;Q_EMIT owner->configChanged();message("已连接接口；等待新鲜状态。加载文件不会改变连接对象。");
    } catch(const std::exception &e){message(QString::fromUtf8(e.what()));}
    update_buttons();
  }
  void update_buttons() {
    const bool valid=fresh() && state.telemetry_valid;
    const bool idle=!state.mission_active && state.phase!="executing" && state.phase!="settling" && state.phase!="canceling" && state.phase!="failure_hold";
    const bool ground=valid && state.ground_state==1;
    arm_button->setEnabled(ground && !state.armed && idle && pending==0 && arm && arm->action_server_is_ready());
    disarm_button->setEnabled(ground && state.armed && idle && pending==0 && disarm && disarm->action_server_is_ready());
    takeoff_button->setEnabled(ground && state.pose_valid && idle && pending==0 && takeoff && takeoff->action_server_is_ready());
    land_button->setEnabled(valid && !land_pending && land && land->action_server_is_ready());
    const bool can_run=valid && state.pose_valid && state.armed && state.ground_state==2 && state.holding && idle && pending==0 && mission && mission->action_server_is_ready();
    run_button->setEnabled(can_run && table->rowCount()>0);step_button->setEnabled(can_run && table->currentRow()>=0);
    stop_button->setEnabled(valid && !stop_pending && stop && stop->service_is_ready() &&
      (state.mission_active || state.holding || (state.operation=="takeoff" && (state.phase=="executing" || state.phase=="settling"))));
    bind_button->setEnabled((pending==0 && !stop_pending) || !fresh());endpoint->setEnabled((pending==0 && !stop_pending) || !fresh());
    render_status();
  }
  static QString translated(const std::string &value,bool phase=false) {
    static const QMap<QString,QString> actions={{"arm","解锁"},{"disarm","上锁"},{"takeoff","起飞"},{"land","降落"},{"hold","位置保持"},{"waypoints","航点任务"}};
    static const QMap<QString,QString> phases={{"idle","空闲"},{"executing","执行中"},{"settling","到点停留"},{"succeeded","已完成"},{"failed","失败"},{"canceling","取消收尾"},{"canceled","已停止"},{"holding","保持中"},{"failure_hold","故障保持"}};
    const auto text=QString::fromStdString(value);return (phase?phases:actions).value(text,text.isEmpty()?"—":text);
  }
  static void paint_badge(QLabel *label,const QString &text,const QString &background,const QString &color) {
    label->setText(text);label->setStyleSheet(QString("QLabel {background:%1;color:%2;border-radius:5px;padding:5px 8px;font-size:12px;font-weight:600;}").arg(background,color));
  }
  void render_status() {
    const bool live=fresh();const bool valid=live && state.telemetry_valid;
    const auto indicator=[](QLabel *label,const QString &name,bool live,bool ok){
      paint_badge(label,name+(live?(ok?" · 正常":" · 无效"):" · 未知"),
        !live?"#e8edf3":(ok?"#e0f2eb":"#fff0da"),!live?"#63758a":(ok?"#176b4a":"#946000"));};
    status_label->setText(!seen?"等待遥测":(live?"实时更新":"遥测已过期"));
    status_label->setStyleSheet(QString("font-size:11px;color:%1;").arg(live?"#176b4a":"#946000"));
    indicator(connection_badge,"连接",live,state.connected);indicator(telemetry_badge,"状态遥测",live,state.telemetry_valid);
    indicator(pose_badge,"位置姿态",live,state.pose_valid);
    paint_badge(armed_badge,!valid?"解锁状态 · 未知":(state.armed?"动力 · 已解锁":"动力 · 已上锁"),
      !valid?"#e8edf3":(state.armed?"#fff0da":"#e0f2eb"),!valid?"#63758a":(state.armed?"#946000":"#176b4a"));
    mode_value->setText(valid?QString::fromStdString(state.backend_mode):"—");
    ground_value->setText(!valid?"—":(state.ground_state==1?"在地面":(state.ground_state==2?"在空中":"未知")));
    action_value->setText(live?translated(state.operation):"—");phase_value->setText(live?translated(state.phase,true):"—");
    action_value->setToolTip(QString::fromStdString(state.operation));phase_value->setToolTip(QString::fromStdString(state.phase));
    phase_value->setStyleSheet(QString("font-weight:600;font-size:13px;color:%1;").arg(live && (state.phase=="failed" || state.phase=="failure_hold")?"#b42318":"#243247"));
    const auto &p=state.current_pose.position;const double values[]={p.x,p.y,p.z,yaw(state.current_pose.orientation)*180/pi};
    for(int i=0;i<4;++i)position_values[i]->setText(live && state.pose_valid?QString::number(values[i],'f',i==3?1:2):"—");
    if(!live)task_value->setText("任务状态未知 · 等待新鲜遥测");
    else if(state.operation=="waypoints" || state.mission_active)
      task_value->setText(QString("航点 %1  ·  累计完成 %2  ·  循环 %3  ·  停留 %4 s%5")
        .arg(state.waypoint_index+1).arg(state.completed_count).arg(state.cycle_count)
        .arg(state.dwell_elapsed_sec,0,'f',1).arg(state.holding?"\n正在保持目标位置":""));
    else task_value->setText(state.holding?"正在保持目标位置":"暂无航点任务");
    const bool error=live && (state.phase=="failed" || state.phase=="failure_hold");
    detail_value->setText(!live?"控制按钮已禁用；恢复新鲜状态后自动更新。":
      (state.detail.empty()?"等待下一步操作":QString(error?"异常说明 · %1":"状态说明 · %1").arg(QString::fromStdString(state.detail))));
    detail_value->setStyleSheet(QString("QLabel {background:%1;color:%2;border-radius:5px;padding:7px;font-size:12px;}")
      .arg(error?"#fff0ee":"#edf2f7",error?"#b42318":"#52657d"));
  }

  void append(const Point &point,int row=-1) {
    if(table->rowCount()>=capacity->value()){message("达到航点容量；可提高容量，上限 500");return;}
    try{Vehicle v;v.points={point};validate(v);}catch(const std::exception &e){message(e.what());return;}
    if(row<0 || row>table->rowCount())row=table->rowCount();
    QSignalBlocker guard(table);table->insertRow(row);const double values[]={point.x,point.y,point.z,point.yaw_deg};
    for(int c=0;c<4;++c)table->setItem(row,c,new QTableWidgetItem(QString::number(values[c],'g',15)));
    table->selectRow(row);changed();
  }
  void move(int delta){int r=table->currentRow(),s=r+delta;if(r<0 || s<0 || s>=table->rowCount())return;
    QSignalBlocker guard(table);for(int c=0;c<4;++c){auto *a=table->takeItem(r,c);auto *b=table->takeItem(s,c);table->setItem(r,c,b);table->setItem(s,c,a);}table->selectRow(s);changed();}
  Vehicle draft() const {
    Vehicle v;v.options={position_tol->value(),yaw_tol->value(),dwell->value(),timeout->value(),check_yaw->isChecked(),loop->isChecked()};
    for(int r=0;r<table->rowCount();++r){double values[4];for(int c=0;c<4;++c){bool ok=false;auto *item=table->item(r,c);values[c]=item?item->text().toDouble(&ok):0;
      if(!ok)throw std::invalid_argument("航点表有无效数字");}v.points.push_back({values[0],values[1],values[2],values[3]});}
    validate(v);return v;
  }
  template<class A> void submit(typename rclcpp_action::Client<A>::SharedPtr client,const typename A::Goal &goal,bool is_land,int step=-1) {
    if(!fresh() || !state.telemetry_valid || (!is_land && pending) || (is_land && land_pending) || !client || !client->action_server_is_ready()) {message("状态过期、请求忙或服务端未就绪");return;}
    ++pending;if(is_land)land_pending=true;const auto rev=revision;const auto gen=generation;
    typename rclcpp_action::Client<A>::SendGoalOptions options;
    options.goal_response_callback=[this,is_land,gen](auto handle){if(gen!=generation)return;if(!handle){--pending;if(is_land)land_pending=false;message("核心拒绝接收请求");}else message("请求已接受；等待遥测确认与最终结果");};
    options.result_callback=[this,is_land,step,rev,gen](const auto &result){if(gen!=generation)return;--pending;if(is_land)land_pending=false;
      if(!result.result){message("没有收到有效动作结果");return;}
      const auto &r=result.result->result;message(QString("动作结果 %1：%2").arg(r.outcome_code).arg(QString::fromStdString(r.message)));
      if(result.code==rclcpp_action::ResultCode::SUCCEEDED && r.outcome_code==0 && step>=0 && revision==rev && step+1<table->rowCount())table->selectRow(step+1);};
    try {client->async_send_goal(goal,options);message("正在提交请求…");}
    catch(const std::exception &e){--pending;if(is_land)land_pending=false;message(e.what());}
    update_buttons();
  }
  void run(bool single) {
    try{auto v=draft();if(v.points.empty())throw std::invalid_argument("请先添加航点");int row=table->currentRow();
      if(single){if(row<0)throw std::invalid_argument("请选中航点");v.points={v.points.at(row)};v.options.loop=false;}
      Mission::Goal g;g.frame_id="map";g.position_tolerance_m=v.options.position;g.check_yaw=v.options.check_yaw;
      g.yaw_tolerance_rad=v.options.yaw_deg*pi/180;g.dwell_sec=v.options.dwell;g.waypoint_timeout_sec=v.options.timeout;g.loop=v.options.loop;
      for(const auto &p:v.points){flight_interfaces::msg::Waypoint w;w.x=p.x;w.y=p.y;w.z=p.z;w.yaw_rad=p.yaw_deg*pi/180;g.waypoints.push_back(w);}
      executing=v.points;dirty=true;submit<Mission>(mission,g,false,single?row:-1);
    }catch(const std::exception &e){message(e.what());}
  }
  void request_stop(){if(stop_pending || !fresh() || !stop->service_is_ready())return;stop_pending=true;const auto gen=generation;
    stop->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>(),[this,gen](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future){if(gen!=generation)return;stop_pending=false;
      try{const auto r=future.get();message(QString(r->success?"停止请求已接受：%1":"停止请求被拒绝：%1").arg(QString::fromStdString(r->message)));}catch(const std::exception &e){message(e.what());}});
  }
  void set_draft(const Vehicle &v) {
    capacity->setValue(std::max(capacity->value(),static_cast<int>(v.points.size())));
    table->setRowCount(0);for(const auto &p:v.points)append(p);
    position_tol->setValue(v.options.position);yaw_tol->setValue(v.options.yaw_deg);
    dwell->setValue(v.options.dwell);timeout->setValue(v.options.timeout);
    check_yaw->setChecked(v.options.check_yaw);loop->setChecked(v.options.loop);changed();
  }
  void save_file(){try{auto copy=document;copy[core]=draft();const auto text=encode(copy);
    const auto path=QFileDialog::getSaveFileName(owner,"保存航点",QString(),"YAML (*.yaml *.yml)");if(path.isEmpty())return;
    QSaveFile f(path);if(!f.open(QIODevice::WriteOnly) || f.write(text.data(),static_cast<qint64>(text.size()))!=static_cast<qint64>(text.size()) || !f.commit())throw std::runtime_error("文件保存失败，原文件保持不变");
    document=std::move(copy);message("航点已保存");}catch(const std::exception &e){message(e.what());}}
  void load_file(){try{const auto path=QFileDialog::getOpenFileName(owner,"加载航点",QString(),"YAML (*.yaml *.yml)");if(path.isEmpty())return;
    QFile f(path);if(!f.open(QIODevice::ReadOnly) || f.size()>4*1024*1024)throw std::runtime_error("文件无法读取或超过 4 MiB");
    auto next=decode(f.readAll().toStdString());if(!next.count(core))throw std::runtime_error("文件中没有当前核心的命名空间，不自动切换控制对象");
    const auto &v=next.at(core);if(v.options.dwell>3600 || v.options.timeout>86400 || v.options.position<0.001 || v.options.yaw_deg<0.001 || v.options.timeout<0.001)throw std::runtime_error("文件选项超过界面支持范围");
    set_draft(v);document=std::move(next);message("已加载草稿，不影响运行中的任务");
    }catch(const std::exception &e){message(e.what());}}
  void publish_markers(){if(!markers)return;visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker clear;clear.action=clear.DELETEALL;array.markers.push_back(clear);
    const auto draw=[&](const Point &p,int id,const std::string &ns,bool active){
      visualization_msgs::msg::Marker m;m.header.frame_id="map";m.header.stamp=node->now();m.ns=ns;m.id=id;m.type=m.ARROW;m.action=m.ADD;
      m.pose.position.x=p.x;m.pose.position.y=p.y;m.pose.position.z=p.z;m.pose.orientation.z=std::sin(p.yaw_deg*pi/360);m.pose.orientation.w=std::cos(p.yaw_deg*pi/360);
      m.scale.x=active?0.6:0.35;m.scale.y=0.08;m.scale.z=0.08;m.color.a=1;m.color.r=active?1:0.1;m.color.g=active?0.6:0.7;m.color.b=active?0:1;
      array.markers.push_back(m);m.ns+="_labels";m.type=m.TEXT_VIEW_FACING;m.pose.position.z+=0.3;m.scale.z=0.22;m.text=active?"当前目标":std::to_string(id+1);array.markers.push_back(m);};
    try{auto v=draft();for(size_t i=0;i<v.points.size();++i)draw(v.points[i],static_cast<int>(i),"draft",false);}catch(const std::exception &){}
    if(fresh() && state.pose_valid && (state.holding || state.mission_active || state.operation=="takeoff")){
      const auto &p=state.target_pose;draw({p.position.x,p.position.y,p.position.z,yaw(p.orientation)*180/pi},0,"active",true);}
    if(show_trajectory->isChecked()) {
      visualization_msgs::msg::Marker trail;trail.header.frame_id="map";trail.header.stamp=node->now();
      trail.ns="actual_trajectory";trail.id=0;trail.type=trail.LINE_LIST;trail.action=trail.ADD;
      trail.pose.orientation.w=1;trail.scale.x=0.035;
      trail.color.r=0.1;trail.color.g=1.0;trail.color.b=0.65;trail.color.a=1.0;
      trail.points=trajectory.points();if(!trail.points.empty())array.markers.push_back(trail);
    }
    if(show_vehicle->isChecked()) {
      std_msgs::msg::Header header;header.frame_id="map";header.stamp=node->now();
      auto model=vehicle_model.markers(header,core,fresh());
      array.markers.insert(array.markers.end(),model.markers.begin(),model.markers.end());
    }
    markers->publish(array);
  }
};
FlightPanel::FlightPanel(QWidget *parent):rviz_common::Panel(parent),impl_(std::make_unique<Impl>(this)){}
FlightPanel::~FlightPanel()=default;
void FlightPanel::save(rviz_common::Config c) const{rviz_common::Panel::save(c);c.mapSetValue("CoreNamespace",QString::fromStdString(impl_->core));}
void FlightPanel::load(const rviz_common::Config &c){rviz_common::Panel::load(c);QString ns;if(c.mapGetString("CoreNamespace",&ns)){impl_->endpoint->setText(ns);impl_->bind();}}
}
PLUGINLIB_EXPORT_CLASS(flight_rviz_plugins::FlightPanel,rviz_common::Panel)
